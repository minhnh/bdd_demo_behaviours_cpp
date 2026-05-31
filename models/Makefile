MODEL ?=
# Accept MODEL with or without the trailing .robmot extension.
MODEL_NAME := $(MODEL:.robmot=)
GEN ?= gen/$(or $(MODEL_NAME), $(error MODEL is not set. Usage: make MODEL=<name>[.robmot]))
STST ?= stst
STEPS ?= 45000
JOBS ?= $(shell nproc)
# Path to the colcon install tree (must contain mj_kdl_wrapper etc.). Read from the
# environment (export INSTALL=<path>) or passed on the command line. Required only for
# configure/build/run; check-deps validates it (with guidance) for those targets only.
INSTALL ?=
# Goals that need the colcon install tree. Empty MAKECMDGOALS means the default goal
# (all -> build), which needs it.
NEEDS_INSTALL := $(if $(MAKECMDGOALS),$(filter all configure build run run-headless just-run jrh,$(MAKECMDGOALS)),all)

.PHONY: all jsonld check ir codegen configure build run clean check-deps

all: build

check-deps:
	@command -v textx >/dev/null 2>&1 || { echo "Error: textx not found. Activate venv or run: pip install textX"; exit 1; }
	@command -v motion-spec-ir-gen >/dev/null 2>&1 || { echo "Error: motion-spec-ir-gen not found. Activate venv or run: pip install -e src/motion-spec"; exit 1; }
	@command -v motion-spec-check >/dev/null 2>&1 || { echo "Error: motion-spec-check not found. Activate venv or run: pip install -e src/motion-spec"; exit 1; }
	@[ -n "$(METAMODELS_PATH)" ] || { echo "Error: METAMODELS_PATH is not set. The JSON-LD generator needs it to resolve metamodel/ontology imports. Export it to your metamodels checkout, e.g. export METAMODELS_PATH=<path-to>/metamodels"; exit 1; }
	@[ -d "$(METAMODELS_PATH)" ] || { echo "Error: METAMODELS_PATH=$(METAMODELS_PATH) is not a directory. Point it at your metamodels checkout."; exit 1; }
	@if [ -n "$(NEEDS_INSTALL)" ]; then \
	    [ -n "$(INSTALL)" ] || { echo "Error: INSTALL is not set. Export it to your colcon install tree (must contain mj_kdl_wrapper etc.), e.g. export INSTALL=<path-to-colcon-install>"; exit 1; }; \
	    [ -d "$(INSTALL)" ] || { echo "Error: INSTALL=$(INSTALL) is not a directory. Point it at your colcon install tree."; exit 1; }; \
	fi

jsonld: $(MODEL_NAME).robmot check-deps
	textx generate $(MODEL_NAME).robmot --target jsonld -o $(GEN)

# SHACL conformance gate: validate the generated graph against the metamodels
# before it can flow into IR/codegen/build. Run standalone with `make check`.
check: jsonld
	motion-spec-check $(GEN)/$(MODEL_NAME)-app.json

ir: check
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
