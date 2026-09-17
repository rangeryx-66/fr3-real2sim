#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--objects", nargs="+", required=True)
    args = ap.parse_args()
    root = Path(args.root)
    rows = []
    for obj in args.objects:
        p = root / obj / "views_08" / "run_result.json"
        if p.exists():
            rows.append(json.loads(p.read_text()))
        else:
            rows.append({"object": obj, "status": "missing", "output_dir": str(p.parent)})
    (root / "results_all.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(json.dumps(rows, indent=2))
    return 0
if __name__ == "__main__":
    raise SystemExit(main())
