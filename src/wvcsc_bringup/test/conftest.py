"""Make the source Python package importable before the first colcon build."""

from pathlib import Path
import sys


PACKAGE_ROOT = str(Path(__file__).resolve().parents[1])
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)
