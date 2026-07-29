# 中文说明：实机默认地图路径解析工具。
# 只选择符合时间戳目录约定的地图，不修改地图文件本身；导航点资产由 Qt/任务文件另行管理。
# 任何默认路径变化都必须通过 launch `--show-args` 和实际参数日志确认。
"""Default timestamped map paths for real bringup."""

from __future__ import annotations

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory


_TIMESTAMP = re.compile(r'^map_(\d{8}_\d{6})$')


def _workspace_source_root() -> Path:
    return Path.home() / 'WVCSC_S2Z_UTB_ARM' / 'src'


def _candidate_roots(package: str, relative: str) -> tuple[Path, ...]:
    """Prefer the checked-out workspace, then the installed package share."""
    roots = [_workspace_source_root() / package / relative]
    try:
        roots.append(Path(get_package_share_directory(package)) / relative)
    except Exception:
        pass
    unique = []
    for root in roots:
        if root not in unique:
            unique.append(root)
    return tuple(unique)


def _latest_timestamp_file(
        root: Path, prefix: str, pattern: str) -> Path | None:
    candidates = []
    for directory in root.glob(f'{prefix}_*'):
        if not directory.is_dir():
            continue
        match = _TIMESTAMP.fullmatch(directory.name)
        if match is None or not match.group(1):
            continue
        for path in directory.glob(pattern):
            if path.is_file() and not path.name.endswith('.example.yaml'):
                candidates.append((match.group(1), path.name, path))
    if not candidates:
        return None
    return max(candidates, key=lambda item: (item[0], item[1]))[2]


def latest_map_yaml() -> str:
    """Return the newest timestamped ``orchard.yaml`` path."""
    for root in _candidate_roots('wvcsc_bringup', 'maps'):
        path = _latest_timestamp_file(root, 'map', 'orchard.yaml')
        if path is not None:
            return str(path)
    searched = ', '.join(str(root) for root in _candidate_roots(
        'wvcsc_bringup', 'maps'))
    raise RuntimeError(
        f'no timestamped map found; expected map_YYYYMMDD_HHMMSS/orchard.yaml '
        f'in: {searched}')
