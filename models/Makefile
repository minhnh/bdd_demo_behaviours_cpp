MODEL ?=
# Accept MODEL with or without the trailing .robmot extension.
MODEL_NAME := $(MODEL:.robmot=)
GEN ?= gen/$(or $(MODEL_NAME), $(error MODEL is not set. Usage: make MODEL=<name>[.robmot]))
STST ?= stst
STEPS ?= 45000
JOBS ?= $(shell nproc)
# Path to the colcon install tree (must contain mj_kdl_wrapper etc.). Read from the
# environment (export INSTALL=<path>) or passed on the command line. Required only for
# configure/build/run; errors with guidance if those targets are used without it.
INSTALL ?= $(error INSTALL is not set. Export it (e.g. `export INSTALL=<path-to-colcon-install>`) or pass INSTALL=<path> on the command line)

.PHONY: all jsonld ir codegen configure build run clean check-deps

all: build

check-deps:
	@command -v textx >/dev/null 2>&1 || { echo "Error: textx not found. Activate venv or run: pip install textX"; exit 1; }
	@command -v motion-spec-ir-gen >/dev/null 2>&1 || { echo "Error: motion-spec-ir-gen not found. Activate venv or run: pip install -e src/motion-spec"; exit 1; }

jsonld: $(MODEL_NAME).robmot check-deps
	textx generate $(MODEL_NAME).robmot --target jsonld -o $(GEN)

ir: jsonld
	motion-spec-ir-gen $(GEN)/$(MODEL_NAME)-app.json -o $(GEN)/ir.json

codegen: ir
	python -m motion_spec.codegen $(GEN)/ir.json -o $(GEN) --stst-bin $(STST)

OROCOS_DIR ?=
OROCOS_FLAG = $(if $(OROCOS_DIR),-Dorocos_kdl_DIR="$(OROCOS_DIR)",)

configure: codegen
	cmake -B $(GEN)/build $(GEN) \
	    -DCMAKE_PREFIX_PATH="$(INSTALL)" \
	    $(OROCOS_FLAG)

build: configure
	cmake --build $(GEN)/build --parallel $(JOBS)

run: build
	$(GEN)/build/main

run-headless: build
	$(GEN)/build/main --headless --steps $(STEPS)

just-run:
	cmake --build $(GEN)/build --target main --parallel $(JOBS)
	$(GEN)/build/main

jrh:
	cmake --build $(GEN)/build --target main --parallel $(JOBS)
	$(GEN)/build/main --headless --steps $(STEPS)

clean:
	$(RM) -r $(GEN)
