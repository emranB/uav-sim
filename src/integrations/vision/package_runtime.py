"""Copy a built estimator and its resolved libraries into a small runtime layer."""

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path


def package_runtime(binary: Path, destination: Path) -> None:
    """Resolve trusted build artifacts; do not include compiler caches or source trees."""
    result = subprocess.run(
        ["ldd", str(binary)], capture_output=True, text=True, timeout=30, check=True
    )
    if "not found" in result.stdout:
        raise RuntimeError(f"Estimator has unresolved runtime libraries: {result.stdout}")
    files = [binary]
    for line in result.stdout.splitlines():
        if "=> /" in line:
            files.append(Path(line.split("=>", 1)[1].strip().split()[0]))
    for source in files:
        if not source.is_absolute():
            raise ValueError("Runtime artifact paths must be absolute")
        # Preserve each requested library name while avoiding /lib -> /usr/lib merge conflicts.
        installed_path = source.parent.resolve() / source.name
        target = destination / installed_path.relative_to("/")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target, follow_symlinks=True)
    extension = Path("/opt/vision-extension")
    if extension.is_dir():
        target = destination / "opt/orb-slam3/observation-extension"
        target.mkdir(parents=True, exist_ok=True)
        sources = [
            *extension.glob("*.h"),
            extension / "reference_pose_adapter.cpp",
            extension / "patch_tracking.py",
        ]
        manifest = {
            "name": "reference-image-pose-seeding",
            "upstream_revision": Path("/opt/orb-slam3/revision.txt").read_text().strip(),
            "sources": {},
        }
        for source in sources:
            shutil.copy2(source, target / source.name)
            manifest["sources"][source.name] = hashlib.sha256(source.read_bytes()).hexdigest()
        (target / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("destination", type=Path)
    arguments = parser.parse_args()
    package_runtime(arguments.binary, arguments.destination)


if __name__ == "__main__":
    main()
