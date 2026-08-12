#!/usr/bin/env python3
"""View any MJCF (.xml) file with mj_kdl_wrapper and the official MuJoCo viewer."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import mj_kdl_wrapper as mjk

# XML comments may contain a bare "--" (e.g. tray.xml's "--mjcf=..."), which is
# invalid per spec and makes a strict parser (xml.etree) reject the whole file
# even though MuJoCo's own parser tolerates it. Strip comments and scan by
# regex instead of parsing, so a malformed-but-working MJCF still loads.
_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)
_FLOOR_GEOM_RE = re.compile(r'<geom\b[^>]*\bname\s*=\s*"floor"', re.DOTALL)

VIEWER_CODE = """
import sys
import mujoco
import mujoco.viewer

expected = sys.argv[2]
actual = mujoco.mj_versionString()
if actual != expected:
    raise SystemExit(
        f"This example exports a MuJoCo {expected} .mjb file. "
        f"Installed Python mujoco is {actual}; install the same mujoco version "
        "or rebuild the wrapper against the installed Python mujoco version."
    )

model = mujoco.MjModel.from_binary_path(sys.argv[1])
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
"""


def has_floor_geom(mjcf_path: str) -> bool:
    # mj_kdl_wrapper's own floor plane is a geom named "floor"; a file that
    # already has one (e.g. tray.xml) collides with it at compile time.
    text = _COMMENT_RE.sub("", Path(mjcf_path).read_text())
    return bool(_FLOOR_GEOM_RE.search(text))


def build_scene(mjcf_path: str, add_floor: bool = True) -> mjk.Scene:
    # SceneObject.mjcf_path attaches the file's own root body (and its joints)
    # as-is, with no assumption of an articulated chain or actuators -- unlike
    # RobotSpec, which expects a robot topology and segfaults on plain assets.
    spec = mjk.SceneSpec()
    spec.timestep = 0.002
    spec.add_floor = add_floor
    spec.add_skybox = True
    obj = mjk.SceneObject()
    obj.mjcf_path = mjcf_path
    spec.objects = [obj]
    return mjk.Scene.build(spec)


def launch_viewer(scene: mjk.Scene) -> None:
    """Show a built scene in the official viewer; closes the scene once it is exported."""
    try:
        tmp = tempfile.NamedTemporaryFile(suffix=".mjb", delete=False)
        tmp.close()
        mjb_path = Path(tmp.name)
        scene.save_binary(str(mjb_path))
    finally:
        scene.close()

    try:
        try:
            subprocess.run(
                [sys.executable, "-c", VIEWER_CODE, str(mjb_path), mjk.mujoco_version()],
                check=True,
            )
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(
                "MuJoCo viewer failed to start. If you are running from a sandbox, SSH session, "
                "or headless shell, run this command from a graphical session with DISPLAY or "
                "WAYLAND_DISPLAY available."
            ) from exc
    finally:
        if mjb_path.exists():
            mjb_path.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mjcf_path", type=Path, help="Path to an MJCF (.xml) file")
    parser.add_argument(
        "--no-floor",
        action="store_true",
        help="Skip the floor plane even if the file has none (default: add one when missing)",
    )
    args = parser.parse_args()

    add_floor = not args.no_floor and not has_floor_geom(str(args.mjcf_path))
    launch_viewer(build_scene(str(args.mjcf_path), add_floor=add_floor))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
