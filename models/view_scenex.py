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
from pathlib import Path

import mj_kdl_wrapper as mjk
import rdflib
from motion_spec.classes.scene import MjcfSceneSpec
from motion_spec.rdf_parser import resources
from motion_spec.rdf_parser.model import Model
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


def read_scenex(scenex_path: Path) -> MjcfSceneSpec:
    """The scene a .scenex describes, read straight from its RDF with no app manifest."""
    graph = rdflib.Dataset(default_union=True)
    graph.default_graph += create_scenex_model_graph(
        scenex_metamodel().model_from_file(str(scenex_path))
    )
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


def build_spec(scene: MjcfSceneSpec, start: Path) -> mjk.SceneSpec:
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
            attachment_spec.attach_to = _target(attachment.attach_kind, attachment.attach_to)
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
    spec.objects = objects

    sites = []
    for frame in scene.frames:
        site = mjk.SiteSpec()
        site.body = frame.body
        site.name = frame.name
        site.pos = [frame.pos_x, frame.pos_y, frame.pos_z]
        site.quat = [frame.quat_x, frame.quat_y, frame.quat_z, frame.quat_w]
        sites.append(site)
    spec.sites = sites

    spec.add_floor = True
    return spec


def build_scene(scenex_path: Path) -> mjk.Scene:
    """Compile the scene a .scenex describes."""
    return mjk.Scene.build(build_spec(read_scenex(scenex_path), scenex_path))


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
    args = parser.parse_args()

    spec = build_spec(read_scenex(args.scenex_path), args.scenex_path)
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
