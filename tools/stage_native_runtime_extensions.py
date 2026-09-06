"""Read-only, explicit five-library staging for the legacy Python runtime.

This is an environment cache, NOT the source overlay or an algorithm change.
Never write into the old cache; never bulk-copy its untracked directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    files = []
    for name in ('kinematics_fused_cu', 'geom_cu', 'lbfgs_step_cu', 'line_search_cu', 'tensor_step_cu'):
        source = args.source / name / (name + '.so')
        if source.is_symlink() or not source.is_file():
            raise ValueError(f'Expected regular shared library: {source}')
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        destination = args.output / name / source.name
        destination.parent.mkdir()
        shutil.copyfile(source, destination)
        assert hashlib.sha256(destination.read_bytes()).hexdigest() == digest
        files.append({'source': str(source), 'staged': str(destination), 'sha256': digest})
    print(json.dumps({'cache_only': True, 'source_written': False, 'files': files}, indent=2))


if __name__ == '__main__':
    main()
