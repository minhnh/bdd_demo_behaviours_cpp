from pathlib import Path

import mujoco
import mujoco.viewer


SCENE = f"""
<mujoco model="tray_scene">
  <option timestep="0.002"/>
  <visual><headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.85 0.83 0.78" rgb2="0.78 0.76 0.71"
             width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0.6 -0.6 1.4" dir="-0.4 0.4 -1" directional="true"/>
    <geom name="floor" type="plane" size="1 1 0.05" material="grid" condim="3"
          friction="0.8 0.02 0.001"/>
  </worldbody>
</mujoco>
"""

def main(mjcf_path: Path, body_anchor: str):
    scene = mujoco.MjSpec.from_string(SCENE)
    obj = mujoco.MjSpec.from_file(str(mjcf_path))
    frame = scene.worldbody.add_frame(pos=[0, 0, 0])
    frame.attach_body(obj.body(body_anchor), "obj_", "")
    model = scene.compile()
    mujoco.viewer.launch(model, mujoco.MjData(model))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("mjcf_path")
    parser.add_argument("--body-anchor", default="root", help="name of body in MJCF to put in scene")
    args = parser.parse_args()
    p = Path(args.mjcf_path)
    if not p.is_file():
        raise RuntimeError(f"not a file: {args.mjcf_path}")

    main(mjcf_path=p, body_anchor=args.body_anchor)
