"""Compare an R02 execution's immutable hash ledgers to a checkout; no Isaac.

Private evidence stays outside Git. This reads source ledgers, not observations,
grasp poses, authored dynamics or camera targets.
"""
import argparse
import hashlib
import json
from pathlib import Path


def audit(checkout, evidence):
    result = {}
    for name, key in (
        ('V81_PRELAUNCH.json', 'source_sha256'),
        ('V81_LOADED_SOURCES.json', 'sources'),
        ('SOURCE_PROVENANCE.json', 'sources'),
    ):
        raw = (evidence / name).read_bytes()
        ledger = json.loads(raw)[key]
        mismatches = []
        for relative, expected in ledger.items():
            path = checkout / relative
            actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
            if actual != expected:
                mismatches.append(dict(path=relative, expected=expected, actual=actual))
        result[name] = dict(manifest_sha256=hashlib.sha256(raw).hexdigest(),
                            checked=len(ledger), mismatches=mismatches)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkout', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.checkout, args.evidence)
    print(json.dumps(result, indent=2))
    raise SystemExit(int(any(item['mismatches'] for item in result.values())))
