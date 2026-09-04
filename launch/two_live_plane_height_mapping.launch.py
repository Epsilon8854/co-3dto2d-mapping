"""Public installed alias for the config-driven two-live launch.

CMake preserves ``two_live_mapping.launch.py`` as
``two_live_mapping_base.launch.py`` and installs this lightweight wrapper under
the public name. All alignment parameters are handled directly by the base
launch and the selected occupancy YAML; no runtime monkey-patching is required.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory


def _load_base_module():
    package_share = get_package_share_directory("co_3dto2d_mapping")
    path = os.path.join(package_share, "launch", "two_live_mapping_base.launch.py")
    spec = importlib.util.spec_from_file_location("co3dto2d_two_live_base", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load two-live base launch: %s" % path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_BASE = _load_base_module()


def generate_launch_description():
    return _BASE.generate_launch_description()
