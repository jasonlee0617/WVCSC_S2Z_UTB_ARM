"""读取实机机械臂任务配置中的 launch 默认参数。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class RealArmDefaults:
    """实机 launch 与机械臂节点共用的运行默认值。"""

    observation_mode: str
    velocity_scaling: float
    acceleration_scaling: float


def load_real_arm_defaults(path: str | Path) -> RealArmDefaults:
    """从 ``arm_task_real.yaml`` 读取并校验观察模式和轨迹缩放。"""

    config_path = Path(path)
    try:
        data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(
            f'cannot read real arm task configuration: {config_path}') from exc

    try:
        parameters = data['wvcsc_spray_task']['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f'{config_path} must define '
            'wvcsc_spray_task.ros__parameters') from exc
    if not isinstance(parameters, dict):
        raise RuntimeError(
            f'{config_path} wvcsc_spray_task.ros__parameters must be a mapping')

    observation_mode = parameters.get('observation_mode')
    if observation_mode not in ('joint_presets', 'ik'):
        raise RuntimeError(
            f'{config_path} observation_mode must be joint_presets or ik')

    scalings = {}
    for name in ('velocity_scaling', 'acceleration_scaling'):
        value = parameters.get(name)
        if isinstance(value, bool):
            raise RuntimeError(f'{config_path} {name} must be in (0, 1]')
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f'{config_path} {name} must be in (0, 1]') from exc
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            raise RuntimeError(f'{config_path} {name} must be in (0, 1]')
        scalings[name] = value

    return RealArmDefaults(
        observation_mode=observation_mode,
        velocity_scaling=scalings['velocity_scaling'],
        acceleration_scaling=scalings['acceleration_scaling'],
    )
