#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Generate JSON-LD from a .robmot file using the motion-spec-dsl generator."""

import sys
from pathlib import Path

from motion_spec_dsl.generators.registration import (
    _build_manifest,
    _gen_graph,
    _merged_context,
    motion_spec_metamodel,
    SUPPORTED_FORMATS,
)
from motion_spec_dsl.generators.motion_spec_graph import MotionSpecDatasetBuilder
import json


def main():
    if len(sys.argv) < 3:
        print("Usage: gen_jsonld.py <input.robmot> <output_dir>")
        sys.exit(1)

    input_path = Path(sys.argv[1]).resolve()
    output_dir = Path(sys.argv[2]).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(str(input_path))

    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()

    stem = input_path.stem
    graph_path = output_dir / f"{stem}.json"
    serialized = dataset.default_graph.serialize(
        format="json-ld", indent=2, context=_merged_context(context)
    )
    graph_path.write_text(serialized.decode() if isinstance(serialized, bytes) else serialized)
    print(f"  wrote {graph_path}")

    manifest_path = output_dir / f"{stem}-app.json"
    manifest_path.write_text(
        json.dumps(_build_manifest(dataset, [graph_path.name]), indent=2)
    )
    print(f"  wrote {manifest_path}")


if __name__ == "__main__":
    main()
