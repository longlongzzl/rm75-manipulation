"""Explicit Python-script bootstrap; does not replace an environment's sitecustomize."""
from pathlib import Path
import os
import runpy
import sys

# Load only the task-local prompt bridge, not cuRobo or any hardware adapter.
_root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_root))
from rm75_app.workcell.input_bridge import install, install_subprocess_bridge


def main():
    if len(sys.argv) < 2:
        raise ValueError('The original Python script path is required')
    directory = os.environ.get('RM75_WORKCELL_INPUT_DIR')
    if not directory:
        raise ValueError('This bootstrap must be launched by the workcell worker')
    script = Path(sys.argv[1]).resolve()
    sys.argv = [str(script), *sys.argv[2:]]
    sys.path[0] = str(script.parent)
    install(directory)
    install_subprocess_bridge(directory)
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
