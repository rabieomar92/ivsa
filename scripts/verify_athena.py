"""Verify the saved Athena source and artifacts without building or uploading."""

import argparse
import hashlib
import json
from pathlib import Path


def verify(root, manifest):
    entries = json.loads(manifest.read_text(encoding="utf-8"))["files"]
    for entry in entries:
        path = (root / entry["path"]).resolve()
        if not path.is_relative_to(root):
            raise SystemExit("Invalid manifest path: " + entry["path"])
        if not path.is_file():
            raise SystemExit("Missing file: " + entry["path"])
        data = path.read_bytes()
        if len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
            raise SystemExit("Checksum mismatch: " + entry["path"])
    print(f"Athena verified: {len(entries)} files ({manifest.name})")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", nargs="?", default="ATHENA_MANIFEST.json")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    verify(root, root / args.manifest)
