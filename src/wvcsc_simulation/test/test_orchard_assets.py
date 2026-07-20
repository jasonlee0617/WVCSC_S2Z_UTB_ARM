import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from wvcsc_simulation.data_acquisition.orchard_assets import (
    CAMERA_FACING_CANDIDATE_COUNT,
    EXPECTED_COMPLETE_FRUIT_COUNT,
    EXPECTED_LEAF_COMPONENT_COUNT,
    FRUIT_LEAF_CLEARANCE,
    FRUIT_COUNT_PER_TREE,
    MIN_CAMERA_VISIBLE_FRUIT_Z,
    MAX_DISEASED_FRUIT_COUNT,
    MIN_DISEASED_FRUIT_COUNT,
    RETAINED_LEAF_COMPONENT_COUNT,
    TREE_SCALE,
    _complete_fruits,
    _fruit_components,
    _tree_mesh_components,
    generate_orchard_assets,
)


PACKAGE_DIR = Path(__file__).parents[1]
WORLD = PACKAGE_DIR / 'worlds' / 'orchard.world'
MODEL = PACKAGE_DIR / 'models' / 'apple_tree'
MAP = PACKAGE_DIR / 'maps' / 'orchard.pgm'
MAP_YAML = PACKAGE_DIR / 'maps' / 'orchard.yaml'
ROBOT_URDF = PACKAGE_DIR.parents[0] / 'Alicia-M-ROS2' / 'alicia_m_descriptions' / 'urdf' / 'Alicia_M_v1_1' / 'Alicia_M_v1_1_follower.urdf'


def _generate(tmp_path, seed, ratio=0.50):
    return generate_orchard_assets(
        WORLD, MODEL, seed=seed, diseased_ratio=ratio,
        output_dir=tmp_path / f'orchard_{seed}',
    )


def _manifest(world):
    return json.loads((world.parent / 'manifest.json').read_text(encoding='utf-8'))


def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_pgm(path):
    tokens = []
    for line in path.read_text(encoding='ascii').splitlines():
        tokens.extend(line.split('#', 1)[0].split())
    assert tokens[0] == 'P2'
    width, height, maximum = (int(value) for value in tokens[1:4])
    return width, height, maximum, [int(value) for value in tokens[4:]]


def test_static_map_contains_all_tree_trunks_and_keeps_dock_poses_free():
    width, height, maximum, pixels = _read_pgm(MAP)
    metadata = yaml.safe_load(MAP_YAML.read_text(encoding='utf-8'))
    assert (width, height, maximum, len(pixels)) == (60, 40, 255, 2400)
    assert metadata['resolution'] == 0.5
    assert metadata['origin'] == [-10.0, -10.0, 0.0]

    occupied = {
        (column, row)
        for row in range(height)
        for column in range(width)
        if pixels[row * width + column] == 0
    }
    border = {
        (column, row)
        for row in range(height)
        for column in range(width)
        if column in (0, width - 1) or row in (0, height - 1)
    }
    tree_positions = [
        tuple(float(value) for value in include.findtext('pose').split()[:2])
        for include in ET.parse(WORLD).getroot().findall('.//include')
        if include.findtext('uri') == 'model://apple_tree'
    ]
    expected_trunks = set()
    origin_x, origin_y = metadata['origin'][:2]
    resolution = metadata['resolution']
    for x, y in tree_positions:
        column = int((x - origin_x) // resolution)
        grid_row = int((y - origin_y) // resolution)
        image_row = height - 1 - grid_row
        expected_trunks.update(
            (trunk_column, trunk_row)
            for trunk_column in (column - 1, column)
            for trunk_row in (image_row, image_row + 1)
        )
    assert len(border) == 196
    assert len(tree_positions) == 8
    assert len(expected_trunks) == 32
    assert occupied == border | expected_trunks

    for x, y in ((3.0, 0.5), (5.0, -0.5),
                 (11.0, 0.5), (13.0, -0.5)):
        column = int((x - origin_x) // resolution)
        grid_row = int((y - origin_y) // resolution)
        image_row = height - 1 - grid_row
        assert (column, image_row) not in occupied


def test_tree_scale_limits_source_mesh_to_1_8_m():
    vertices = [
        float(line.split()[3])
        for line in (MODEL / 'meshes' / 'apple_tree.obj').read_text(
            encoding='utf-8').splitlines()
        if line.startswith('v ')
    ]
    assert max(vertices) * TREE_SCALE == pytest.approx(1.8, abs=0.01)
    model = ET.parse(MODEL / 'model.sdf').getroot()
    visual_scales = [
        [float(value) for value in scale.text.split()]
        for scale in model.findall('.//visual/geometry/mesh/scale')
    ]
    assert len(visual_scales) == 2
    for scale in visual_scales:
        assert scale == pytest.approx([TREE_SCALE] * 3, abs=1e-6)
    collision = model.find('.//collision')
    pose_z = float(collision.findtext('pose').split()[2])
    assert pose_z == pytest.approx(0.75 * TREE_SCALE, abs=1e-6)
    assert float(collision.findtext('.//radius')) == pytest.approx(
        0.25 * TREE_SCALE, abs=1e-6)
    assert float(collision.findtext('.//length')) == pytest.approx(
        1.5 * TREE_SCALE, abs=1e-6)


def test_source_assets_contain_complete_fruits_and_leaf_components():
    fruit_lines = (MODEL / 'meshes' / 'apple_tree_apples.obj').read_text(
        encoding='utf-8').splitlines(keepends=True)
    complete_fruits = _complete_fruits(_fruit_components(fruit_lines))
    assert len(complete_fruits) == EXPECTED_COMPLETE_FRUIT_COUNT
    assert all(len(fruit['source_components']) == 2 for fruit in complete_fruits)
    tree_lines = (MODEL / 'meshes' / 'apple_tree.obj').read_text(
        encoding='utf-8').splitlines(keepends=True)
    _, leaves = _tree_mesh_components(tree_lines)
    assert len(leaves) == EXPECTED_LEAF_COMPONENT_COUNT


def test_tool0_frame_is_preserved_without_a_visual_marker():
    robot = ET.parse(ROBOT_URDF).getroot()
    tool0 = next(link for link in robot.findall('link') if link.get('name') == 'tool0')
    assert tool0.find('visual') is None
    joint = next(
        joint for joint in robot.findall('joint')
        if joint.get('name') == 'tool0_fixed_joint')
    assert joint.find('child').get('link') == 'tool0'


def test_each_tree_has_reproducible_healthy_and_diseased_fruits(tmp_path):
    first = _generate(tmp_path / 'first', 42)
    second = _generate(tmp_path / 'second', 42)
    first_manifest = _manifest(first)
    second_manifest = _manifest(second)
    assert len(first_manifest['trees']) == 8
    assert first_manifest['fruit_count_per_tree'] == FRUIT_COUNT_PER_TREE
    assert (first_manifest['camera_facing_candidate_count'] ==
            CAMERA_FACING_CANDIDATE_COUNT)
    for tree_name, data in first_manifest['trees'].items():
        assert data['healthy_count'] in (2, 3)
        assert data['diseased_count'] in (
            MIN_DISEASED_FRUIT_COUNT, MAX_DISEASED_FRUIT_COUNT)
        selected = set(data['selected_fruits'])
        healthy = set(data['healthy_fruits'])
        diseased = set(data['diseased_fruits'])
        assert len(selected) == FRUIT_COUNT_PER_TREE
        assert selected == healthy | diseased
        assert not healthy & diseased
        assert selected <= set(data['candidate_fruits'])
        assert len(data['candidate_fruits']) == CAMERA_FACING_CANDIDATE_COUNT
        assert len(data['selected_source_components']) == FRUIT_COUNT_PER_TREE
        assert all(len(group) == 2 for group in data['selected_source_components'])
        assert all(center[2] >= MIN_CAMERA_VISIBLE_FRUIT_Z
                   for center in data['selected_fruit_centers'])
        assert data['retained_leaf_count'] == RETAINED_LEAF_COMPONENT_COUNT
        assert all(
            sum((leaf[axis] - fruit[axis]) ** 2 for axis in range(3)) >=
            FRUIT_LEAF_CLEARANCE ** 2
            for leaf in data['retained_leaf_centers']
            for fruit in data['selected_fruit_centers'])
        assert data == second_manifest['trees'][tree_name]
        model_name = f'orchard_{tree_name}'
        for mesh in ('healthy_apples.obj', 'diseased_apples.obj', 'sparse_tree.obj'):
            assert _digest(first.parent / 'models' / model_name / mesh) == _digest(
                second.parent / 'models' / model_name / mesh)
        model = ET.parse(
            first.parent / 'models' / model_name / 'model.sdf').getroot()
        fruit_uris = [
            visual.findtext('./geometry/mesh/uri')
            for visual in model.findall('.//visual')
            if 'apples' in visual.get('name', '')
        ]
        assert fruit_uris == [
            f'model://{model_name}/healthy_apples.obj',
            f'model://{model_name}/diseased_apples.obj',
        ]
        colors = {
            visual.get('name'): visual.findtext('./material/diffuse')
            for visual in model.findall('.//visual')
            if 'apples' in visual.get('name', '')
        }
        assert colors == {
            'healthy_apples_visual': '1.00 0.04 0.02 1.00',
            'diseased_apples_visual': '1.00 0.95 0.02 1.00',
        }
        assert all(
            visual.findtext('./material/emissive') is not None
            for visual in model.findall('.//visual')
            if 'apples' in visual.get('name', ''))


def test_different_seed_changes_diseased_fruit_selection(tmp_path):
    first = _manifest(_generate(tmp_path / 'first', 42))
    second = _manifest(_generate(tmp_path / 'second', 43))
    assert any(
        first['trees'][name]['selected_fruits'] !=
        second['trees'][name]['selected_fruits']
        for name in first['trees']
    )


def test_generated_world_preserves_tree_names_and_poses(tmp_path):
    generated = _generate(tmp_path, 42)
    source_includes = ET.parse(WORLD).getroot().findall('.//include')
    generated_includes = ET.parse(generated).getroot().findall('.//include')
    source = {
        include.findtext('name'): include.findtext('pose')
        for include in source_includes if include.findtext('name')
    }
    result = {
        include.findtext('name'): include.findtext('pose')
        for include in generated_includes if include.findtext('name')
    }
    assert result == source
    assert all(
        include.findtext('uri').startswith('model://orchard_')
        for include in generated_includes
        if include.findtext('name', '').endswith(('01', '02', '03', '04'))
    )
