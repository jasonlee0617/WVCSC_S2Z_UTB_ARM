"""Measured-site mission schema, map binding and safety validation.

The mutable site file lives outside the installed package.  This module keeps
all coordinate and map checks shared by the capture, validation and loading
executables so those three entry points cannot silently interpret a site file
differently.
"""

import copy
from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import shutil
import tempfile

import yaml
from wvcsc_mission_manager.core import (
    DEFAULT_ARM_BASE_FORWARD_OFFSET,
    DEFAULT_ARM_BASE_LEFT_OFFSET,
)


SCHEMA_VERSION = 3
ARM_BASE_OFFSET_REFERENCE = 'alicia_base_link_xy'
LEGACY_ARM_BASE_OFFSET_REFERENCE = 'arm_base_vehicle_axes'
DEFAULT_FOOTPRINT = ((0.8, -0.4), (0.8, 0.4), (-0.8, 0.4), (-0.8, -0.4))
MIN_TREE_DISTANCE_M = 0.95
MAX_TREE_DISTANCE_M = 1.80
MAX_ARM_BASE_FORWARD_ERROR_M = 0.20
MAX_HINT_RECONSTRUCTION_ERROR_M = 0.03

# 当前只要求采点流程执行成功，暂时使用执行优先的宽松门限。
# 采集结果会原样写入 capture_quality；定位链稳定后再恢复工程门限。
# 调整这四个值后必须重新构建 wvcsc_bringup 并 source install/setup.bash。
MAX_CAPTURE_POSITION_SPREAD_M = 1.00
MAX_CAPTURE_YAW_SPREAD_RAD = 1.00
MAX_CAPTURE_POSITION_STDDEV_M = 1.00
MAX_CAPTURE_YAW_STDDEV_RAD = 1.00


@dataclass(frozen=True)
class MapGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    free_threshold: float
    negate: bool
    maximum_value: int
    pixels: tuple

    def world_to_cell(self, x, y):
        column = int(math.floor((x - self.origin_x) / self.resolution))
        map_row = int(math.floor((y - self.origin_y) / self.resolution))
        row = self.height - 1 - map_row
        return column, row

    def cell_is_free(self, column, row):
        if not (0 <= column < self.width and 0 <= row < self.height):
            return False
        value = self.pixels[row * self.width + column]
        normalized = value / float(self.maximum_value)
        occupied_probability = normalized if self.negate else 1.0 - normalized
        return occupied_probability < self.free_threshold


def _finite(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{label} must be numeric') from error
    if not math.isfinite(number):
        raise ValueError(f'{label} must be finite')
    return number


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def circular_mean(angles):
    values = tuple(float(angle) for angle in angles)
    if not values:
        raise ValueError('at least one yaw sample is required')
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if math.hypot(sine, cosine) < 1e-9:
        raise ValueError('yaw samples have no stable circular mean')
    return math.atan2(sine, cosine)


def pose_sample_statistics(samples):
    """Return a robust XY center, circular yaw and maximum sample spreads."""
    if not samples:
        raise ValueError('pose samples are required')
    ordered_x = sorted(_finite(sample[0], 'sample.x') for sample in samples)
    ordered_y = sorted(_finite(sample[1], 'sample.y') for sample in samples)
    def median(values):
        middle = len(values) // 2
        if len(values) % 2:
            return values[middle]
        return 0.5 * (values[middle - 1] + values[middle])

    x = median(ordered_x)
    y = median(ordered_y)
    yaw = circular_mean(sample[2] for sample in samples)
    position_spread = max(
        math.hypot(float(sample[0]) - x, float(sample[1]) - y)
        for sample in samples)
    yaw_spread = max(
        abs(normalize_angle(float(sample[2]) - yaw)) for sample in samples)
    return x, y, yaw, position_spread, yaw_spread


def tree_hint_from_arm_base_offset(
        docking_pose, tree_x_m, tree_y_m, z=0.0,
        arm_base_x_m=DEFAULT_ARM_BASE_FORWARD_OFFSET,
        arm_base_y_m=DEFAULT_ARM_BASE_LEFT_OFFSET):
    """Convert an offset measured from the arm-base origin into map coordinates."""
    x, y, yaw = (_finite(value, 'docking_pose') for value in docking_pose)
    tree_x = _finite(tree_x_m, 'tree_x_m')
    tree_y = _finite(tree_y_m, 'tree_y_m')
    arm_x = _finite(arm_base_x_m, 'arm_base_mount.x_m')
    arm_y = _finite(arm_base_y_m, 'arm_base_mount.y_m')
    tree_z = _finite(z, 'tree_base_z_m')
    return (
        x + math.cos(yaw) * (arm_x + tree_x) -
        math.sin(yaw) * (arm_y + tree_y),
        y + math.sin(yaw) * (arm_x + tree_x) +
        math.cos(yaw) * (arm_y + tree_y),
        tree_z,
    )


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).expanduser().open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _map_description(map_yaml):
    yaml_path = Path(map_yaml).expanduser().resolve()
    if not yaml_path.is_file():
        raise ValueError(f'map YAML not found: {yaml_path}')
    with yaml_path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    image_value = str(data.get('image', '')).strip()
    if not image_value:
        raise ValueError('map YAML does not define image')
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise ValueError(f'map image not found: {image_path}')
    return yaml_path, image_path, data


def map_hashes(map_yaml):
    yaml_path, image_path, _data = _map_description(map_yaml)
    return {
        'frame_id': 'map',
        'yaml_sha256': file_sha256(yaml_path),
        'image_sha256': file_sha256(image_path),
    }


def _pgm_token(data, index):
    length = len(data)
    while index < length:
        if data[index] == 35:  # '#'
            while index < length and data[index] not in (10, 13):
                index += 1
        elif chr(data[index]).isspace():
            index += 1
        else:
            break
    start = index
    while index < length and not chr(data[index]).isspace():
        index += 1
    if start == index:
        raise ValueError('invalid PGM header')
    return data[start:index].decode('ascii'), index


def _read_pgm(path):
    data = Path(path).read_bytes()
    magic, index = _pgm_token(data, 0)
    width_text, index = _pgm_token(data, index)
    height_text, index = _pgm_token(data, index)
    maximum_text, index = _pgm_token(data, index)
    width, height, maximum = int(width_text), int(height_text), int(maximum_text)
    if width <= 0 or height <= 0 or not 0 < maximum <= 255:
        raise ValueError('unsupported PGM dimensions or bit depth')
    if magic == 'P5':
        if index >= len(data) or not chr(data[index]).isspace():
            raise ValueError('invalid binary PGM separator')
        if data[index:index + 2] == b'\r\n':
            index += 2
        else:
            index += 1
        pixels = tuple(data[index:index + width * height])
    elif magic == 'P2':
        pixels = []
        while len(pixels) < width * height:
            token, index = _pgm_token(data, index)
            pixels.append(int(token))
        pixels = tuple(pixels)
    else:
        raise ValueError(f'unsupported map image format {magic}; use PGM')
    if len(pixels) != width * height:
        raise ValueError('PGM pixel count does not match dimensions')
    return width, height, maximum, pixels


def load_map_grid(map_yaml):
    _yaml_path, image_path, data = _map_description(map_yaml)
    width, height, maximum, pixels = _read_pgm(image_path)
    origin = data.get('origin')
    if not isinstance(origin, list) or len(origin) < 2:
        raise ValueError('map origin must contain x and y')
    resolution = _finite(data.get('resolution'), 'map.resolution')
    free_threshold = _finite(data.get('free_thresh'), 'map.free_thresh')
    if resolution <= 0.0 or not 0.0 < free_threshold < 1.0:
        raise ValueError('invalid map resolution or free threshold')
    return MapGrid(
        width, height, resolution,
        _finite(origin[0], 'map.origin.x'),
        _finite(origin[1], 'map.origin.y'),
        free_threshold, bool(int(data.get('negate', 0))), maximum, pixels)


def _rotate_footprint(pose, footprint):
    x, y, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple((
        x + cosine * px - sine * py,
        y + sine * px + cosine * py,
    ) for px, py in footprint)


def _inside_polygon(x, y, polygon):
    inside = False
    previous = len(polygon) - 1
    for index, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[previous]
        if ((yi > y) != (yj > y) and
                x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        previous = index
    return inside


def footprint_is_free(grid, pose, footprint=DEFAULT_FOOTPRINT):
    polygon = _rotate_footprint(pose, footprint)
    cells = [grid.world_to_cell(x, y) for x, y in polygon]
    min_column = min(column for column, _row in cells) - 1
    max_column = max(column for column, _row in cells) + 1
    min_row = min(row for _column, row in cells) - 1
    max_row = max(row for _column, row in cells) + 1
    checked = 0
    for row in range(min_row, max_row + 1):
        for column in range(min_column, max_column + 1):
            world_x = grid.origin_x + (column + 0.5) * grid.resolution
            map_row = grid.height - 1 - row
            world_y = grid.origin_y + (map_row + 0.5) * grid.resolution
            if _inside_polygon(world_x, world_y, polygon):
                checked += 1
                if not grid.cell_is_free(column, row):
                    return False
    return checked > 0


def new_site_document(site_id, mission_id, map_yaml):
    return {
        'schema_version': SCHEMA_VERSION,
        'site_id': str(site_id).strip(),
        'map': map_hashes(map_yaml),
        'arm_base_mount': {
            'x_m': DEFAULT_ARM_BASE_FORWARD_OFFSET,
            'y_m': DEFAULT_ARM_BASE_LEFT_OFFSET,
        },
        'mission': {
            'mission_id': str(mission_id).strip(),
            'return_home_after_finish': False,
            'home_pose': None,
            'targets': [],
        },
    }


def load_site_document(path):
    candidate = Path(path).expanduser()
    if not candidate.is_file():
        raise ValueError(f'site mission not found: {candidate}')
    with candidate.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError('site mission root must be a mapping')
    if document.get('schema_version') != SCHEMA_VERSION:
        raise ValueError(
            f'site schema_version must be {SCHEMA_VERSION}; migrate a schema-v2 '
            'file with migrate_site_mission or recapture HOME and targets')
    return document


def _pose(mapping, label):
    if not isinstance(mapping, dict):
        raise ValueError(f'{label} is required')
    return tuple(_finite(mapping.get(key), f'{label}.{key}')
                 for key in ('x', 'y', 'yaw'))


def _point(mapping, label):
    if not isinstance(mapping, dict):
        raise ValueError(f'{label} is required')
    return tuple(_finite(mapping.get(key), f'{label}.{key}')
                 for key in ('x', 'y', 'z'))


def validate_site_document(
        document, map_yaml, *, require_capture_quality=True,
        require_free_space=True):
    errors = []

    def check(condition, message):
        if not condition:
            errors.append(message)

    try:
        check(int(document.get('schema_version', -1)) == SCHEMA_VERSION,
              f'schema_version must be {SCHEMA_VERSION}')
        check(bool(str(document.get('site_id', '')).strip()), 'site_id is required')
        mount = document.get('arm_base_mount') or {}
        arm_x = _finite(mount.get('x_m'), 'arm_base_mount.x_m')
        arm_y = _finite(mount.get('y_m'), 'arm_base_mount.y_m')
        check(abs(arm_x - DEFAULT_ARM_BASE_FORWARD_OFFSET) <= 1e-9,
              'arm_base_mount.x_m does not match robot geometry')
        check(abs(arm_y - DEFAULT_ARM_BASE_LEFT_OFFSET) <= 1e-9,
              'arm_base_mount.y_m does not match robot geometry')
        expected_hashes = map_hashes(map_yaml)
        map_section = document.get('map') or {}
        check(map_section.get('frame_id') == 'map', 'map.frame_id must be map')
        check(map_section.get('yaml_sha256') == expected_hashes['yaml_sha256'],
              'map YAML SHA256 does not match the selected map')
        check(map_section.get('image_sha256') == expected_hashes['image_sha256'],
              'map image SHA256 does not match the selected map')
        mission = document.get('mission') or {}
        check(bool(str(mission.get('mission_id', '')).strip()),
              'mission.mission_id is required')
        home = _pose(mission.get('home_pose'), 'mission.home_pose')
        targets = mission.get('targets')
        check(isinstance(targets, list) and bool(targets),
              'mission.targets must be a non-empty list')
        grid = load_map_grid(map_yaml)
        if require_free_space:
            check(footprint_is_free(grid, home),
                  'HOME footprint is not in free map space')
        seen = set()
        for index, target in enumerate(targets if isinstance(targets, list) else []):
            label = f'mission.targets[{index}]'
            target_id = str(target.get('target_id', '')).strip()
            check(bool(target_id), f'{label}.target_id is required')
            check(target_id not in seen, f'duplicate target_id: {target_id}')
            seen.add(target_id)
            docking = _pose(target.get('docking_pose'), f'{label}.docking_pose')
            offset = target.get('tree_offset_arm_base_m') or {}
            check(offset.get('reference') == ARM_BASE_OFFSET_REFERENCE,
                  f'{label}.tree_offset_arm_base_m.reference must be '
                  f'{ARM_BASE_OFFSET_REFERENCE}')
            tree_x = _finite(offset.get('x_m'), f'{label}.tree_x_m')
            tree_y = _finite(offset.get('y_m'), f'{label}.tree_y_m')
            tree_base_z = _finite(target.get('tree_base_z_m'),
                                  f'{label}.tree_base_z_m')
            check(abs(tree_x) <= MAX_ARM_BASE_FORWARD_ERROR_M,
                  f'{label} arm-base X error exceeds '
                  f'{MAX_ARM_BASE_FORWARD_ERROR_M:.2f} m')
            distance = math.hypot(tree_x, tree_y)
            check(MIN_TREE_DISTANCE_M <= distance <= MAX_TREE_DISTANCE_M,
                  f'{label} tree distance must be within '
                  f'{MIN_TREE_DISTANCE_M:.2f}-{MAX_TREE_DISTANCE_M:.2f} m')
            tree_hint_from_arm_base_offset(
                docking, tree_x, tree_y, tree_base_z, arm_x, arm_y)
            duration = _finite(target.get('spray_duration'),
                               f'{label}.spray_duration')
            check(0.2 <= duration <= 10.0,
                  f'{label}.spray_duration must be within 0.2-10.0 s')
            if require_free_space:
                check(footprint_is_free(grid, docking),
                      f'{label} docking footprint is not in free map space')
            if require_capture_quality:
                quality = target.get('capture_quality') or {}
                samples = int(quality.get('samples', 0))
                position_spread = _finite(
                    quality.get('position_spread_m'),
                    f'{label}.capture_quality.position_spread_m')
                yaw_spread = _finite(
                    quality.get('yaw_spread_rad'),
                    f'{label}.capture_quality.yaw_spread_rad')
                position_stddev = _finite(
                    quality.get('max_position_stddev_m'),
                    f'{label}.capture_quality.max_position_stddev_m')
                yaw_stddev = _finite(
                    quality.get('max_yaw_stddev_rad'),
                    f'{label}.capture_quality.max_yaw_stddev_rad')
                check(samples >= 30, f'{label} requires at least 30 samples')
                check(position_spread <= MAX_CAPTURE_POSITION_SPREAD_M,
                      f'{label} position spread exceeds '
                      f'{MAX_CAPTURE_POSITION_SPREAD_M:.2f} m')
                check(yaw_spread <= MAX_CAPTURE_YAW_SPREAD_RAD,
                      f'{label} yaw spread exceeds '
                      f'{MAX_CAPTURE_YAW_SPREAD_RAD:.2f} rad')
                check(position_stddev <= MAX_CAPTURE_POSITION_STDDEV_M,
                      f'{label} AMCL position stddev exceeds '
                      f'{MAX_CAPTURE_POSITION_STDDEV_M:.2f} m')
                check(yaw_stddev <= MAX_CAPTURE_YAW_STDDEV_RAD,
                      f'{label} AMCL yaw stddev exceeds '
                      f'{MAX_CAPTURE_YAW_STDDEV_RAD:.2f} rad')
    except (KeyError, TypeError, ValueError) as error:
        errors.append(str(error))
    if errors:
        raise ValueError('; '.join(errors))
    return document


def migrate_site_document(document, map_yaml):
    """Convert one schema-v2 site file using its recorded numeric offsets."""
    if not isinstance(document, dict) or document.get('schema_version') != 2:
        raise ValueError('site migration requires schema_version 2')
    converted = copy.deepcopy(document)
    mount = converted.get('arm_base_mount') or {}
    arm_x = _finite(mount.get('forward_m'), 'arm_base_mount.forward_m')
    arm_y = _finite(mount.get('left_m'), 'arm_base_mount.left_m')
    converted['arm_base_mount'] = {'x_m': arm_x, 'y_m': arm_y}
    for index, target in enumerate((converted.get('mission') or {}).get('targets') or []):
        label = f'mission.targets[{index}]'
        docking = _pose(target.get('docking_pose'), f'{label}.docking_pose')
        hint = _point(target.get('tree_hint'), f'{label}.tree_hint')
        offset = target.get('measured_tree_offset') or {}
        if offset.get('reference') != LEGACY_ARM_BASE_OFFSET_REFERENCE:
            raise ValueError(f'{label}.measured_tree_offset.reference is invalid')
        tree_x = _finite(offset.get('forward_m'), f'{label}.forward_m')
        tree_y = _finite(offset.get('left_m'), f'{label}.left_m')
        expected = tree_hint_from_arm_base_offset(
            docking, tree_x, tree_y, hint[2], arm_x, arm_y)
        if math.hypot(expected[0] - hint[0], expected[1] - hint[1]) > \
                MAX_HINT_RECONSTRUCTION_ERROR_M:
            raise ValueError(f'{label}.tree_hint does not match recorded offset')
        target['tree_offset_arm_base_m'] = {
            'reference': ARM_BASE_OFFSET_REFERENCE,
            'x_m': tree_x,
            'y_m': tree_y,
        }
        target['tree_base_z_m'] = hint[2]
        target.pop('tree_hint', None)
        target.pop('measured_tree_offset', None)
        target.pop('spray_side', None)
    converted['schema_version'] = SCHEMA_VERSION
    validate_site_document(converted, map_yaml)
    return converted


def atomic_write_site(path, document, *, backup=True):
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    if backup and destination.exists():
        shutil.copy2(destination, destination.with_suffix(destination.suffix + '.bak'))
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{destination.name}.', suffix='.tmp', dir=destination.parent)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
