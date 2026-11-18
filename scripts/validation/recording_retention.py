"""Apply one reviewed checkpoint retention plan; shared Docker cleanup is separate."""

import argparse
import json
from pathlib import Path

from common.logging.checkpoint_retention import apply_checkpoint_retention


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(apply_checkpoint_retention(args.output_root, args.plan, args.report)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
