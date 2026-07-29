"""WVCSC Gazebo simulation helpers.

``data_acquisition`` is installed beside this package because its capture
scripts are also exposed as standalone ROS executables.  Extending the package
path keeps the same ``wvcsc_simulation.data_acquisition`` import valid when a
developer runs launch tests directly from the source tree and when ROS loads
the installed package.
"""

from pathlib import Path


_source_package_parent = Path(__file__).resolve().parents[1]
if (_source_package_parent / 'data_acquisition').is_dir():
    __path__.append(str(_source_package_parent))
