"""Translate the shared occupancy switch into merged-map filter parameters.

The YAML is read at launch time, not watched while nodes are running. Keep
``dynamic_*`` under ``/** -> ros__parameters`` in the same file on both robots.
Local mappers read these values directly; the record republisher needs the
legacy ``merged_*`` names. Never forward the whole occupancy configuration to
that node: unrelated wildcard parameters can change its output behaviour.
"""

from pathlib import Path
from typing import Dict, Union
import math

import yaml


ParameterValue = Union[bool, int, float]
# Match the C++ mapper defaults when a custom YAML omits an optional setting.
_COUNTERS = {
    "dynamic_free_clear_count": (4, 1, 65535),
    "dynamic_occupied_confirm_count": (3, 1, 65535),
    "dynamic_counter_decay": (1, 0, 65535),
    "dynamic_evidence_timeout_frames": (30, 0, 4294967295),
}


def merged_dynamic_parameters(config_file: str) -> Dict[str, ParameterValue]:
    """Load and validate the common switch; shared values override old aliases.

    An explicit false disables temporal fusion even if an obsolete
    ``merged_temporal_filter_enabled: true`` remains in the configuration.
    Numeric strings and string booleans are rejected instead of silently
    treating, for example, ``"false"`` as true.
    """
    path = Path(config_file)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"Cannot read dynamic-filter config {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError(f"{path}: expected a ROS parameter YAML mapping")
    wildcard = document.get("/**")
    if not isinstance(wildcard, dict) or not isinstance(
        wildcard.get("ros__parameters"), dict
    ):
        raise ValueError(f"{path}: shared settings must be under /** -> ros__parameters")
    params = wildcard["ros__parameters"]
    enabled = params.get("dynamic_filter_enabled", False)
    if type(enabled) is not bool:
        raise ValueError(f"{path}: dynamic_filter_enabled must be YAML true or false")
    raycast = params.get("enable_raycast_free_space", True)
    if type(raycast) is not bool:
        raise ValueError(f"{path}: enable_raycast_free_space must be YAML true or false")
    if enabled and not raycast:
        raise ValueError(
            f"{path}: dynamic_filter_enabled requires enable_raycast_free_space=true; "
            "without free-space observations old obstacles cannot be cleared"
        )
    result: Dict[str, ParameterValue] = {"merged_temporal_filter_enabled": enabled}
    for key, (default, minimum, maximum) in _COUNTERS.items():
        value = params.get(key, default)
        if type(value) is not int or not minimum <= value <= maximum:
            raise ValueError(f"{path}: {key} must be an integer in [{minimum}, {maximum}]")
        result["merged_" + key] = value

    # Preserve the existing optional merged-map geometry tuning, but use only
    # genuinely observed free cells by default (no implicit free-space dilation).
    for key, default in (
        ("merged_free_observation_inflation_m", 0.0),
        ("merged_alignment_reset_translation_m", 0.05),
        ("merged_alignment_reset_yaw_deg", 0.5),
    ):
        value = params.get(key, default)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{path}: {key} must be finite and non-negative")
        result[key] = float(value)
    return result
