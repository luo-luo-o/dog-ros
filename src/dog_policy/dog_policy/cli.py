"""查看、校验、切换或回滚 ONNX 策略版本。"""

import argparse
import os
import sys
from pathlib import Path
from tempfile import NamedTemporaryFile

from .bundle import ACTIVE, MANIFEST, Bundle


def activate(root: Path, version: str) -> None:
    Bundle.load(root, version)
    with NamedTemporaryFile(mode="w", dir=root, encoding="utf-8", delete=False) as handle:
        temp = Path(handle.name)
        try:
            handle.write(version + "\n")
            handle.close()
            temp.chmod(0o644)
            temp.replace(root / ACTIVE)
        finally:
            temp.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(
        description="List, validate, or activate ONNX policy versions.",
        usage="policy [--assets DIR] {list,check,activate} [VERSION]",
    )
    parser.add_argument(
        "--assets", type=Path, help="policy deployment directory",
        default=Path(os.environ.get("DOG_POLICY_DIR", "policies/deploy")),
    )
    parser.add_argument("command", metavar="COMMAND", help="list, check, or activate")
    parser.add_argument("version", nargs="?", metavar="VERSION", help="check defaults to the active version")
    args = parser.parse_args()
    root = args.assets
    try:
        if args.command == "list" and args.version is None:
            active = (root / ACTIVE).read_text(encoding="utf-8").strip() if (root / ACTIVE).is_file() else None
            for path in sorted(root.iterdir()):
                if (path / MANIFEST).is_file():
                    print(path.name + (" (active)" if path.name == active else ""))
        elif args.command == "check":
            bundle = Bundle.load(root, args.version)
            print(f"{bundle.name} passed validation; default clip: {bundle.clip}")
        elif args.command == "activate" and args.version:
            activate(root, args.version)
            print(f"Activated {args.version}; restart to apply")
        else:
            raise ValueError("Use list, check [VERSION], or activate VERSION")
    except Exception as error:
        print(f"Policy command failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
