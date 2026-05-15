# Building a .robmot file into C++

Three steps: JSON-LD generation, IR generation, C++ code generation. Then cmake.

## Prerequisites

Install into the workspace virtualenv (`.venv/` at the workspace root):

- `motion-spec-dsl` — DSL parser and JSON-LD generator
- `motion-spec` — IR generator and C++ code generator

Runtime dependencies:

- `mj_kdl_wrapper` source tree (for the MuJoCo/KDL backend)
- MuJoCo 3.8.0 at `/opt/mujoco-3.8.0` (or override with `-DMUJOCO_ROOT=...`)
- Custom orocos_kdl build at `install/orocos_kdl/` in the workspace (the apt
  package lacks `chainhdsolver_vereshchagin_fixed_joint.hpp`)
- StringTemplate runner `stst` (STSTv4) — used by the code generator

## Step 1 — Generate JSON-LD

From `models/`:

```bash
.venv/bin/textx generate mj_fall.robmot \
    --target jsonld \
    -o gen/mj_fall
```

Produces `gen/mj_fall/mj_fall.json` and `mj_fall-app.json`.

## Step 2 — Generate IR

```bash
.venv/bin/motion-spec-ir-gen \
    gen/mj_fall/mj_fall-app.json \
    -o gen/mj_fall/ir.json
```

Produces `gen/mj_fall/ir.json`.

## Step 3 — Generate C++

```bash
.venv/bin/python -m motion_spec.codegen \
    gen/mj_fall/ir.json \
    -o gen/mj_fall \
    --stst-bin /home/batsy/util/STSTv4/stst \
    --backend mj-kdl
```

Produces under `gen/mj_fall/`:

```
CMakeLists.txt
ref_main.cpp
headers/
    runtime.hpp
    shared_state.hpp
    motion_<name>.hpp   (one per MOTION_SPEC in the .robmot)
```

## Step 4 — Build

```bash
cd gen/mj_fall

cmake -B build \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DMJ_KDL_WRAPPER_SOURCE_DIR=/path/to/mj_kdl_wrapper \
    -DMUJOCO_ROOT=/opt/mujoco-3.8.0 \
    -Dorocos_kdl_DIR=/path/to/install/orocos_kdl/share/orocos_kdl/cmake

cmake --build build --parallel $(nproc)
```

The binary is `build/main`. Run headless with:

```bash
./build/main --headless --steps 2000
```

## Running all steps from `models/`

```bash
WS=/home/batsy/work/ms
MODEL=mj_fall
GEN=gen/$MODEL

.venv/bin/textx generate $MODEL.robmot --target jsonld -o $GEN

.venv/bin/motion-spec-ir-gen $GEN/$MODEL-app.json -o $GEN/ir.json

.venv/bin/python -m motion_spec.codegen $GEN/ir.json -o $GEN \
    --stst-bin /home/batsy/util/STSTv4/stst --backend mj-kdl

cmake -S $GEN -B $GEN/build \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DMJ_KDL_WRAPPER_SOURCE_DIR=$WS/src/mj_kdl_wrapper \
    -DMUJOCO_ROOT=/opt/mujoco-3.8.0 \
    -Dorocos_kdl_DIR=$WS/install/orocos_kdl/share/orocos_kdl/cmake

cmake --build $GEN/build --parallel $(nproc)
```

## Notes

- All commands run from `models/` (this directory).
- The `--backend mj-kdl` flag selects the MuJoCo torque-control backend. It
  wires ACHD (Vereshchagin) for constrained joint accelerations followed by
  RNEA for full inverse dynamics, which is required when writing directly to
  `qfrc_applied`.
- `-Dorocos_kdl_DIR` must point to the local build, not the apt package. The
  apt package lacks the fixed-joint Vereshchagin solver header.
- The generated `CMakeLists.txt` links against `mj_kdl_wrapper` and
  `orocos-kdl`. Pass `-DMJ_KDL_WRAPPER_SOURCE_DIR` to build it from source, or
  install the package and omit that flag.
