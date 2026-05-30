MODEL ?=
# Accept MODEL with or without the trailing .robmot extension.
MODEL_NAME := $(MODEL:.robmot=)
GEN ?= gen/$(or $(MODEL_NAME), $(error MODEL is not set. Usage: make MODEL=<name>[.robmot]))
ROOT ?= $(CURDIR)
STST ?= stst
STEPS ?= 45000
JOBS ?= $(shell nproc)
INSTALL ?= $(ROOT)/install

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
