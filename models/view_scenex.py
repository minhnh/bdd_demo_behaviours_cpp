#!/usr/bin/env python3
"""Load a .scenex in mj_kdl_wrapper and view it in the wrapper's simulate UI.

The scene comes from the same reader the generated C++ uses (motion_spec's read_scene),
so what this shows is what a run of the model would compose -- robots, attachments,
objects and placement -- without generating or building anything. The control timestep is
the exception: it is authored in the .robmot's ENVIRONMENT, so a bare .scenex gets the
default.
"""

from __future__ import annotations

import argparse
import os
import random
from pathlib import Path

import mj_kdl_wrapper as mjk
import rdflib
from motion_spec.classes.scene import MjcfSceneSpec
from motion_spec.rdf_parser import resources
from motion_spec.rdf_parser.model import Model
from motion_spec_dsl.rdf.sampling import draw_samples
from motion_spec_dsl.rdf_parser.vocab import GEOM_COORD
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_PRED_LOWER,
    URI_DISTRIB_PRED_UPPER,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_DISTRIB_TYPE_UNIFORM,
)
from rdflib.namespace import RDF
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

# Asset paths are authored relative to the workspace root; these markers say which cache
# subdirectory holds the same file when the workspace does not (mirrors find_asset_path in
# motion-spec's backend_mj_kdl.stg).
_CACHE_MARKERS = (
    ("third_party/menagerie/", "menagerie"),
    ("src/mj_kdl_wrapper/assets/", "assets"),
    ("src/examples/assets/", "assets"),
)


def scenex_graph(scenex_path: Path, seed: int | None = None):
    """The .scenex as RDF, with every sampled placement drawn."""
    graph = create_scenex_model_graph(
        scenex_metamodel().model_from_file(str(scenex_path))
    )
    # The reader resolves poses numerically, so a sampled placement must carry its draw first.
    rng = random.Random(
        seed if seed is not None else random.SystemRandom().randrange(2**32)
    )
    draw_samples(graph, rng)
    return graph


def read_scenex(scenex_path: Path, seed: int | None = None) -> MjcfSceneSpec:
    """The scene a .scenex describes, read straight from its RDF with no app manifest."""
    return read_graph(scenex_graph(scenex_path, seed), scenex_path)


def read_graph(scene_graph, scenex_path: Path) -> MjcfSceneSpec:
    """The scene a drawn scenex graph describes."""
    graph = rdflib.Dataset(default_union=True)
    graph.default_graph += scene_graph
    model = Model(
        graph=graph,
        app_path=scenex_path.resolve(),
        imported_models=[],
        imported_provenance=[],
    )
    return resources.read_scene(model)


def find_asset(relative: str, start: Path) -> str:
    """An authored asset path as a file on this machine.

    Searched from the working directory and from the .scenex upwards -- the paths are
    workspace-relative -- then in the mj_kdl_wrapper caches the assets are fetched into.

    Raises:
        FileNotFoundError: no candidate exists, so the scene cannot be composed.
    """
    path = Path(relative)
    if path.is_absolute():
        if path.exists():
            return str(path)
        raise FileNotFoundError(relative)

    for root in (Path.cwd(), start.resolve().parent):
        for base in (root, *root.parents):
            if (base / path).exists():
                return str(base / path)

    text = path.as_posix()
    cache_root = mjk.menagerie.assets_cache_dir().parent
    for marker, subdir in _CACHE_MARKERS:
        if marker not in text:
            continue
        tail = text.split(marker, 1)[1]
        roots = [cache_root / subdir]
        if subdir == "menagerie" and os.environ.get("MJ_KDL_MENAGERIE"):
            roots.insert(0, Path(os.environ["MJ_KDL_MENAGERIE"]))
        for candidate in (root / tail for root in roots):
            if candidate.exists():
                return str(candidate)

    raise FileNotFoundError(
        f"asset '{relative}' was not found from {Path.cwd()}, from {start.resolve().parent} "
        f"or in {cache_root}; run 'mj-kdl-fetch-menagerie' or set MJ_KDL_MENAGERIE"
    )


def _target(kind: str, name: str) -> mjk.AttachTarget:
    return mjk.AttachTarget(getattr(mjk.AttachKind, kind), name)


_REGION_RGBA = (
    (0.90, 0.25, 0.25, 0.35),
    (0.25, 0.65, 0.90, 0.35),
    (0.35, 0.80, 0.40, 0.35),
    (0.95, 0.75, 0.20, 0.35),
)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_rot(q, v):
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + y * tz - z * ty,
        v[1] + w * ty + z * tx - x * tz,
        v[2] + w * tz + x * ty - y * tx,
    )


def region_objects(scene_graph, scene: MjcfSceneSpec) -> list:
    """Every 3-D uniform region of the scenex as a thin translucent slab.

    Placed in the frame the scene's sampled placements are seen by, so a cell no draw landed in
    is still shown where a run that drew from it would place.
    """
    # Each distribution is stated in the frame of the placement that draws from it, and the
    # scene draws from several frames, so the frame is resolved per distribution.
    seen_by_of = {}
    for node in scene_graph.subjects(RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY):
        if scene_graph.value(node, GEOM_COORD.x) is None:
            continue
        distrib = scene_graph.value(node, URI_DISTRIB_PRED_FROM_DISTRIB)
        seen_by = scene_graph.value(node, GEOM_COORD["as-seen-by"])
        if distrib is not None and seen_by is not None:
            seen_by_of.setdefault(distrib, str(seen_by).rsplit("/", 1)[-1])

    slabs = []
    for node in sorted(
        scene_graph.subjects(RDF.type, URI_DISTRIB_TYPE_UNIFORM), key=str
    ):
        lower = scene_graph.value(node, URI_DISTRIB_PRED_LOWER)
        upper = scene_graph.value(node, URI_DISTRIB_PRED_UPPER)
        if lower is None or upper is None:
            continue
        low = [float(v) for v in scene_graph.items(lower)]
        high = [float(v) for v in scene_graph.items(upper)]
        if len(low) != 3 or len(high) != 3:
            continue
        frame = next((f for f in scene.frames if f.name == seen_by_of.get(node)), None)
        if frame is None:
            continue
        body = next((o for o in scene.objects if o.body == frame.body), None)
        if body is None:
            continue
        frame_pos = (frame.pos_x, frame.pos_y, frame.pos_z)
        frame_quat = (frame.quat_x, frame.quat_y, frame.quat_z, frame.quat_w)
        body_pos = (body.pos_x, body.pos_y, body.pos_z)
        body_quat = (body.quat_x, body.quat_y, body.quat_z, body.quat_w)
        world_quat = _quat_mul(body_quat, frame_quat)
        centre = [(a + b) / 2.0 for a, b in zip(low, high)]
        on_body = _quat_rot(frame_quat, centre)
        in_world = _quat_rot(body_quat, [a + b for a, b in zip(frame_pos, on_body)])
        slab = mjk.SceneObject()
        slab.name = f"region_{str(node).rsplit('/', 1)[-1]}"
        slab.shape = mjk.Shape.BOX
        # A hairline z band would z-fight with the table, so the slab is given a visible thickness.
        slab.size = [max((b - a) / 2.0, 0.002) for a, b in zip(low, high)]
        slab.pos = [a + b for a, b in zip(body_pos, in_world)]
        slab.quat = list(world_quat)
        slab.rgba = list(_REGION_RGBA[len(slabs) % len(_REGION_RGBA)])
        slab.fixed = True
        slab.mass = 0.001
        slab.condim = mjk.Condim.Tangential
        slab.friction = [1.0, 0.005, 0.0001]
        slabs.append(slab)
    return slabs


def build_spec(
    scene: MjcfSceneSpec, start: Path, regions: list | None = None
) -> mjk.SceneSpec:
    """The scene as a wrapper SceneSpec, with every authored asset path resolved."""
    spec = mjk.SceneSpec()
    spec.timestep = scene.timestep_s
    spec.add_skybox = True

    robots = []
    for robot in scene.robots:
        robot_spec = mjk.RobotSpec()
        robot_spec.path = find_asset(robot.path, start)
        robot_spec.prefix = robot.prefix
        robot_spec.attach_to = _target(robot.attach_kind, robot.attach_name)
        robot_spec.pos = [robot.pos_x, robot.pos_y, robot.pos_z]
        robot_spec.quat = [robot.quat_x, robot.quat_y, robot.quat_z, robot.quat_w]
        attachments = []
        for attachment in robot.attachments:
            attachment_spec = mjk.AttachmentSpec()
            attachment_spec.mjcf_path = find_asset(attachment.path, start)
            attachment_spec.attach_to = _target(
                attachment.attach_kind, attachment.attach_to
            )
            attachment_spec.prefix = attachment.prefix
            attachment_spec.pos = [attachment.pos_x, attachment.pos_y, attachment.pos_z]
            attachment_spec.quat = [
                attachment.quat_x,
                attachment.quat_y,
                attachment.quat_z,
                attachment.quat_w,
            ]
            attachments.append(attachment_spec)
        robot_spec.attachments = attachments
        robots.append(robot_spec)
    spec.robots = robots

    objects = []
    for obj in scene.objects:
        object_spec = mjk.SceneObject()
        object_spec.name = obj.body
        object_spec.attach_to = _target(obj.attach_kind, obj.attach_name)
        object_spec.pos = [obj.pos_x, obj.pos_y, obj.pos_z]
        object_spec.quat = [obj.quat_x, obj.quat_y, obj.quat_z, obj.quat_w]
        object_spec.fixed = obj.fixed
        if obj.has_path:
            object_spec.mjcf_path = find_asset(obj.path, start)
        else:
            object_spec.shape = getattr(mjk.Shape, obj.shape)
            object_spec.size = obj.size
            object_spec.rgba = obj.color
            object_spec.mass = obj.mass
            object_spec.condim = mjk.Condim.Rolling
            object_spec.friction = obj.friction
        objects.append(object_spec)
    spec.objects = objects + list(regions or [])

    sites = []
    for frame in scene.frames:
        site = mjk.SiteSpec()
        site.body = frame.body
        site.name = frame.name
        site.pos = [frame.pos_x, frame.pos_y, frame.pos_z]
        site.quat = [frame.quat_x, frame.quat_y, frame.quat_z, frame.quat_w]
        sites.append(site)
    spec.sites = sites

    cameras = []
    for camera in scene.static_cameras:
        camera_spec = mjk.CameraSpec()
        camera_spec.name = camera.name
        camera_spec.body = camera.body
        camera_spec.pos = [camera.pos_x, camera.pos_y, camera.pos_z]
        camera_spec.quat = [camera.quat_x, camera.quat_y, camera.quat_z, camera.quat_w]
        camera_spec.fovy = camera.fovy_deg
        cameras.append(camera_spec)
    spec.cameras = cameras

    spec.add_floor = True
    return spec


def build_scene(scenex_path: Path, seed: int | None = None) -> mjk.Scene:
    """Compile the scene a .scenex describes."""
    return mjk.Scene.build(build_spec(read_scenex(scenex_path, seed), scenex_path))


def describe(spec: mjk.SceneSpec, scene: mjk.Scene) -> None:
    print(f"timestep: {spec.timestep} s, floor at the anchor (z = {spec.floor_z} m)")
    for robot in spec.robots:
        print(f"robot: {robot.path} @ {robot.attach_to.kind} '{robot.attach_to.name}'")
        for attachment in robot.attachments:
            print(
                f"  attachment: {attachment.mjcf_path} @ "
                f"{attachment.attach_to.kind} '{attachment.attach_to.name}'"
            )
    for obj in spec.objects:
        print(f"object: {obj.name} {obj.mjcf_path or obj.shape} at {list(obj.pos)}")
    print(f"cameras: {scene.camera_names()}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenex_path", type=Path, help="Path to a .scenex file")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Compile and describe the scene without opening the viewer",
    )
    parser.add_argument("--save-xml", type=Path, help="Write the composed MJCF here")
    parser.add_argument(
        "--seed",
        type=int,
        help="Seed the draws of any sampled placement, as 'motion-spec gen --seed' does",
    )
    parser.add_argument(
        "--show-regions",
        action="store_true",
        help="Draw every 3-D uniform distribution the scenex declares as a translucent slab",
    )
    args = parser.parse_args()

    graph = scenex_graph(args.scenex_path, args.seed)
    scene = read_graph(graph, args.scenex_path)
    regions = region_objects(graph, scene) if args.show_regions else []
    spec = build_spec(scene, args.scenex_path, regions)
    scene = mjk.Scene.build(spec)
    try:
        describe(spec, scene)
        if args.save_xml:
            scene.save_xml(str(args.save_xml))
        if args.headless:
            return 0
        viewer = mjk.SimulateViewer.open(scene, args.scenex_path.name)
        try:
            while viewer.is_running():
                if not viewer.step():
                    break
                viewer.pace()  # step() never sleeps; an unpaced loop starves the render thread
        finally:
            viewer.close()
    finally:
        scene.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
