"""View tray.xml with a floor and lights. Usage: python view_tray.py"""

import pathlib

import mujoco
import mujoco.viewer

HERE = pathlib.Path(__file__).parent

SCENE = f"""
<mujoco model="tray_scene">
  <compiler meshdir="{HERE}" angle="radian" autolimits="true"/>
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

spec = mujoco.MjSpec.from_string(SCENE)
tray = mujoco.MjSpec.from_file(str(HERE / "tray.xml"))
frame = spec.worldbody.add_frame(pos=[0, 0, 0])
frame.attach_body(tray.body("root"), "tray_", "")

model = spec.compile()
mujoco.viewer.launch(model, mujoco.MjData(model))
