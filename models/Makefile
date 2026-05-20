MODEL ?= pick_place
GEN ?= gen/$(MODEL)
ROOT ?= /home/batsy/work/ms
VENV ?= $(ROOT)/.venv
STST ?= stst
STEPS ?= 45000
JOBS ?= 4

.PHONY: all jsonld ir codegen build run clean

all: build

jsonld: $(MODEL).robmot
	$(VENV)/bin/textx generate $(MODEL).robmot --target jsonld -o $(GEN)

ir: jsonld
	$(VENV)/bin/motion-spec-ir-gen $(GEN)/$(MODEL)-app.json -o $(GEN)/ir.json

codegen: ir
	$(VENV)/bin/python -m motion_spec.codegen $(GEN)/ir.json -o $(GEN) --stst-bin $(STST)

build: codegen
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
	$(RM) -r $(GEN)/build
