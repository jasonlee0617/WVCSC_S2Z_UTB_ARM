"""Generate deterministic sparse orchard assets for Gazebo."""

import copy
import hashlib
import json
import math
import os
from pathlib import Path
import random
import shutil
import tempfile
import xml.etree.ElementTree as ET


TREE_SCALE = 1.8 / 2.2833495
EXPECTED_FRUIT_COUNT = 134
EXPECTED_COMPLETE_FRUIT_COUNT = 67
FRUIT_COMPONENTS_PER_FRUIT = 2
FRUIT_COUNT_PER_TREE = 5
MIN_DISEASED_FRUIT_COUNT = 2
MAX_DISEASED_FRUIT_COUNT = 3
CAMERA_FACING_CANDIDATE_COUNT = 32
MIN_FRUIT_SPACING = 0.15
MIN_CAMERA_VISIBLE_FRUIT_Z = 0.90
EXPECTED_LEAF_COMPONENT_COUNT = 5376
RETAINED_LEAF_COMPONENT_COUNT = 269
FRUIT_LEAF_CLEARANCE = 0.18
FRUIT_PAIR_MAX_DISTANCE = 0.025


def _face_vertices(line, vertex_count):
    indices = []
    for token in line.split()[1:]:
        value = int(token.split('/', 1)[0])
        indices.append(value - 1 if value > 0 else vertex_count + value)
    return indices


def _points(lines):
    return [
        tuple(float(value) for value in line.split()[1:4])
        for line in lines if line.startswith('v ')
    ]


def _components(points, faces):
    """Return connected face components with source-space centers."""
    vertex_count = len(points)
    parent = list(range(vertex_count))

    def find(value):
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    def union(left, right):
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    face_vertices = []
    for face in faces:
        vertices = _face_vertices(face, vertex_count)
        if not vertices:
            raise ValueError('OBJ contains an empty face')
        if any(index < 0 or index >= vertex_count for index in vertices):
            raise ValueError('OBJ face references an invalid vertex')
        for vertex in vertices[1:]:
            union(vertices[0], vertex)
        face_vertices.append(vertices)

    grouped = {}
    for face, vertices in zip(faces, face_vertices):
        component = grouped.setdefault(
            find(vertices[0]), {'faces': [], 'vertices': set()})
        component['faces'].append(face)
        component['vertices'].update(vertices)
    return [
        {
            'faces': grouped[key]['faces'],
            'center': tuple(
                sum(points[index][axis] for index in grouped[key]['vertices']) /
                len(grouped[key]['vertices'])
                for axis in range(3)),
        }
        for key in sorted(grouped)
    ]


def _fruit_components(lines):
    return _components(_points(lines), [
        line for line in lines if line.startswith('f ')
    ])


def _distance(left, right, axes=(0, 1, 2)):
    return math.sqrt(sum((left[axis] - right[axis]) ** 2 for axis in axes))


def _complete_fruits(components):
    """Pair the two disconnected mesh components that make up one apple."""
    if len(components) != EXPECTED_FRUIT_COUNT:
        raise ValueError(
            f'expected {EXPECTED_FRUIT_COUNT} fruit components, found '
            f'{len(components)}')
    unpaired = set(range(len(components)))
    groups = []
    while unpaired:
        first = min(unpaired)
        unpaired.remove(first)
        second = min(
            unpaired,
            key=lambda index: _distance(
                components[first]['center'], components[index]['center']))
        separation = _distance(
            components[first]['center'], components[second]['center'])
        if separation > FRUIT_PAIR_MAX_DISTANCE:
            raise ValueError(
                f'fruit component {first} has no paired half within '
                f'{FRUIT_PAIR_MAX_DISTANCE}')
        unpaired.remove(second)
        source_components = sorted((first, second))
        centers = [components[index]['center'] for index in source_components]
        groups.append({
            'source_components': source_components,
            'faces': [
                face for index in source_components
                for face in components[index]['faces']
            ],
            'center': tuple(
                sum(center[axis] for center in centers) / len(centers)
                for axis in range(3)),
        })
    if len(groups) != EXPECTED_COMPLETE_FRUIT_COUNT:
        raise ValueError(
            f'expected {EXPECTED_COMPLETE_FRUIT_COUNT} complete fruits, '
            f'found {len(groups)}')
    return groups


def _faces_by_material(lines):
    material = None
    grouped = {}
    for line in lines:
        if line.startswith('usemtl '):
            material = line.split(maxsplit=1)[1].strip()
        elif line.startswith('f '):
            grouped.setdefault(material, []).append(line)
    return grouped


def _tree_mesh_components(lines):
    grouped = _faces_by_material(lines)
    if set(grouped) != {'BranchMAT', 'LeafMAT'}:
        raise ValueError(f'unexpected tree OBJ materials: {sorted(grouped)}')
    points = _points(lines)
    leaves = _components(points, grouped['LeafMAT'])
    if len(leaves) != EXPECTED_LEAF_COMPONENT_COUNT:
        raise ValueError(
            f'expected {EXPECTED_LEAF_COMPONENT_COUNT} leaf components, '
            f'found {len(leaves)}')
    return grouped['BranchMAT'], leaves


def _write_obj(path, source_lines, faces, material_name, material_file):
    geometry = [
        line for line in source_lines
        if line.startswith(('v ', 'vt ', 'vn '))
    ]
    path.write_text(''.join([
        '# Generated by wvcsc_simulation.orchard_assets\n',
        f'mtllib {material_file}\n',
        f'o {material_name}\n',
        *geometry,
        f'usemtl {material_name}\n',
        *faces,
    ]), encoding='utf-8')


def _write_sparse_tree_obj(path, source_lines, branch_faces, leaf_components):
    geometry = [
        line for line in source_lines
        if line.startswith(('v ', 'vt ', 'vn '))
    ]
    leaf_faces = [
        face for component in leaf_components for face in component['faces']
    ]
    path.write_text(''.join([
        '# Generated sparse orchard tree\n',
        'mtllib apple_tree.mtl\n',
        'o SparseAppleTree\n',
        *geometry,
        'usemtl BranchMAT\n',
        *branch_faces,
        'usemtl LeafMAT\n',
        *leaf_faces,
    ]), encoding='utf-8')


def _write_material(path, name, diffuse):
    red, green, blue = diffuse
    path.write_text(
        '# Generated fruit material\n'
        f'newmtl {name}\n'
        f'Ka {red * 0.55:.2f} {green * 0.55:.2f} {blue * 0.55:.2f}\n'
        f'Kd {red:.2f} {green:.2f} {blue:.2f}\n'
        f'Ke {red * 0.35:.2f} {green * 0.35:.2f} {blue * 0.35:.2f}\n'
        'Ks 0.05 0.05 0.05\n'
        'Ns 8.0\n'
        'illum 2\n',
        encoding='utf-8',
    )


def _tree_seed(seed, tree_name):
    digest = hashlib.sha256(f'{seed}:{tree_name}'.encode()).digest()
    return int.from_bytes(digest[:8], byteorder='big')


def _road_facing_components(fruits, pose):
    """Rank complete fruits by road-facing depth and outer-canopy radius."""
    values = [float(value) for value in pose.split()]
    if len(values) != 6:
        raise ValueError('apple tree pose must contain six values')
    _, tree_y, _, _, _, yaw = values
    road_direction_y = math.cos(yaw) * (-tree_y)
    if abs(road_direction_y) < 1e-9:
        raise ValueError('apple tree must not be placed on the road center')
    direction = 1.0 if road_direction_y > 0.0 else -1.0
    visible = [
        index for index, fruit in enumerate(fruits)
        if fruit['center'][2] >= MIN_CAMERA_VISIBLE_FRUIT_Z]
    if len(visible) < FRUIT_COUNT_PER_TREE:
        raise ValueError('not enough elevated fruits for the camera view')
    return sorted(
        visible,
        key=lambda index: (
            direction * fruits[index]['center'][1],
            math.hypot(fruits[index]['center'][0], fruits[index]['center'][1]),
            -index,
        ),
        reverse=True,
    )


def _select_fruits(fruits, ranked, seed, diseased_ratio):
    """Select five separated exterior fruits and deterministically label disease."""
    rng = random.Random(seed)
    candidates = ranked[:min(CAMERA_FACING_CANDIDATE_COUNT, len(ranked))]
    selection_order = candidates[:]
    rng.shuffle(selection_order)
    selection_order.extend(index for index in ranked if index not in candidates)
    selected = []
    for index in selection_order:
        center = fruits[index]['center']
        if all(_distance(center, fruits[other]['center'], axes=(0, 2)) >=
               MIN_FRUIT_SPACING for other in selected):
            selected.append(index)
        if len(selected) == FRUIT_COUNT_PER_TREE:
            break
    if len(selected) != FRUIT_COUNT_PER_TREE:
        raise ValueError('could not select five separated exterior fruits')

    diseased = [index for index in selected if rng.random() < diseased_ratio]
    while len(diseased) < MIN_DISEASED_FRUIT_COUNT:
        diseased.append(rng.choice([index for index in selected if index not in diseased]))
    while len(diseased) > MAX_DISEASED_FRUIT_COUNT:
        diseased.pop(rng.randrange(len(diseased)))
    diseased = sorted(diseased)
    healthy = sorted(set(selected) - set(diseased))
    return sorted(selected), healthy, diseased, candidates


def _select_leaves(leaves, selected_fruits, seed):
    selected_centers = [fruit['center'] for fruit in selected_fruits]
    available = [
        leaf for leaf in leaves
        if all(_distance(leaf['center'], center) >= FRUIT_LEAF_CLEARANCE
               for center in selected_centers)
    ]
    if len(available) < RETAINED_LEAF_COMPONENT_COUNT:
        raise ValueError('insufficient leaves remain after fruit clearance')
    rng = random.Random(seed ^ 0xA5A5A5A5)
    return sorted(
        rng.sample(available, RETAINED_LEAF_COMPONENT_COUNT),
        key=lambda leaf: leaf['center'])


def _set_visual_material(visual, color):
    material = visual.find('material')
    if material is None:
        material = ET.SubElement(visual, 'material')
    for name, value in (
            ('ambient', tuple(component * 0.55 for component in color[:3]) + (1.0,)),
            ('diffuse', color),
            ('emissive', tuple(component * 0.35 for component in color[:3]) + (1.0,)),
            ('specular', (0.05, 0.05, 0.05, 1.0))):
        node = material.find(name)
        if node is None:
            node = ET.SubElement(material, name)
        node.text = ' '.join(f'{component:.2f}' for component in value)


def _write_tree_model(
        model_dir, source_sdf, tree_mesh_uri, healthy_mesh_uri,
        diseased_mesh_uri):
    model_dir.mkdir(parents=True, exist_ok=True)
    root = ET.parse(source_sdf).getroot()
    model = root.find('model')
    model.set('name', model_dir.name)
    link = model.find('link')
    visuals = {visual.get('name'): visual for visual in link.findall('visual')}
    tree_visual = visuals['tree_visual']
    healthy_visual = visuals['apples_visual']
    healthy_visual.set('name', 'healthy_apples_visual')
    tree_visual.find('./geometry/mesh/uri').text = tree_mesh_uri
    healthy_visual.find('./geometry/mesh/uri').text = healthy_mesh_uri
    _set_visual_material(healthy_visual, (1.0, 0.04, 0.02, 1.0))
    diseased_visual = copy.deepcopy(healthy_visual)
    diseased_visual.set('name', 'diseased_apples_visual')
    diseased_visual.find('./geometry/mesh/uri').text = diseased_mesh_uri
    _set_visual_material(diseased_visual, (1.0, 0.95, 0.02, 1.0))
    link.append(diseased_visual)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(
        model_dir / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (model_dir / 'model.config').write_text(
        '<?xml version="1.0"?>\n'
        '<model>\n'
        f'  <name>{model_dir.name}</name>\n'
        '  <version>1.0</version>\n'
        '  <sdf version="1.6">model.sdf</sdf>\n'
        '</model>\n',
        encoding='utf-8',
    )


def _copy_tree_materials(apple_tree_dir, model_dir):
    mesh_dir = apple_tree_dir / 'meshes'
    shutil.copy2(mesh_dir / 'apple_tree.mtl', model_dir / 'apple_tree.mtl')
    shutil.copytree(mesh_dir / 'textures', model_dir / 'textures')


def generate_orchard_assets(world_path, apple_tree_dir, seed=42,
                            diseased_ratio=0.50, output_dir=None):
    """Return a generated world with deterministic sparse, visible fruit."""
    world_path = Path(world_path).resolve()
    apple_tree_dir = Path(apple_tree_dir).resolve()
    seed = int(seed)
    diseased_ratio = float(diseased_ratio)
    if not math.isfinite(diseased_ratio) or not 0.0 <= diseased_ratio <= 1.0:
        raise ValueError('diseased_fruit_ratio must be between 0.0 and 1.0')

    ratio_tag = f'{diseased_ratio:.6f}'.rstrip('0').rstrip('.')
    destination = (
        Path(output_dir) if output_dir else
        Path(tempfile.gettempdir()) / f'wvcsc_orchard_{seed}_{ratio_tag}'
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(
        prefix=f'.{destination.name}_', dir=destination.parent))
    try:
        models_dir = staging / 'models'
        models_dir.mkdir()
        fruit_source = apple_tree_dir / 'meshes' / 'apple_tree_apples.obj'
        fruit_lines = fruit_source.read_text(encoding='utf-8').splitlines(keepends=True)
        fruits = _complete_fruits(_fruit_components(fruit_lines))
        tree_source = apple_tree_dir / 'meshes' / 'apple_tree.obj'
        tree_lines = tree_source.read_text(encoding='utf-8').splitlines(keepends=True)
        branch_faces, leaf_components = _tree_mesh_components(tree_lines)
        world_tree = ET.parse(world_path)
        manifest = {
            'seed': seed,
            'diseased_fruit_ratio': diseased_ratio,
            'tree_scale': TREE_SCALE,
            'source_fruit_component_count': EXPECTED_FRUIT_COUNT,
            'complete_fruit_count': EXPECTED_COMPLETE_FRUIT_COUNT,
            'fruit_count_per_tree': FRUIT_COUNT_PER_TREE,
            'camera_facing_candidate_count': CAMERA_FACING_CANDIDATE_COUNT,
            'retained_leaf_component_count': RETAINED_LEAF_COMPONENT_COUNT,
            'fruit_leaf_clearance': FRUIT_LEAF_CLEARANCE,
            'minimum_camera_visible_fruit_z': MIN_CAMERA_VISIBLE_FRUIT_Z,
            'trees': {},
        }
        for include in world_tree.getroot().findall('.//include'):
            uri = include.find('uri')
            name_node = include.find('name')
            if uri is None or uri.text != 'model://apple_tree' or name_node is None:
                continue
            tree_name = name_node.text.strip()
            model_name = f'orchard_{tree_name}'
            model_dir = models_dir / model_name
            model_dir.mkdir()
            pose = include.findtext('pose')
            if pose is None:
                raise ValueError(f'{tree_name}: apple tree pose is required')
            tree_seed = _tree_seed(seed, tree_name)
            ranked = _road_facing_components(fruits, pose)
            selected, healthy, diseased, candidates = _select_fruits(
                fruits, ranked, tree_seed, diseased_ratio)
            selected_fruits = [fruits[index] for index in selected]
            retained_leaves = _select_leaves(leaf_components, selected_fruits, tree_seed)
            _write_sparse_tree_obj(
                model_dir / 'sparse_tree.obj', tree_lines, branch_faces, retained_leaves)
            _copy_tree_materials(apple_tree_dir, model_dir)
            _write_obj(
                model_dir / 'healthy_apples.obj', fruit_lines,
                [face for index in healthy for face in fruits[index]['faces']],
                'HealthyFruit', 'healthy_apples.mtl')
            _write_obj(
                model_dir / 'diseased_apples.obj', fruit_lines,
                [face for index in diseased for face in fruits[index]['faces']],
                'DiseasedFruit', 'diseased_apples.mtl')
            _write_material(
                model_dir / 'healthy_apples.mtl', 'HealthyFruit',
                (1.00, 0.04, 0.02))
            _write_material(
                model_dir / 'diseased_apples.mtl', 'DiseasedFruit',
                (1.00, 0.95, 0.02))
            _write_tree_model(
                model_dir,
                apple_tree_dir / 'model.sdf',
                f'model://{model_name}/sparse_tree.obj',
                f'model://{model_name}/healthy_apples.obj',
                f'model://{model_name}/diseased_apples.obj')
            uri.text = f'model://{model_name}'
            manifest['trees'][tree_name] = {
                'healthy_count': len(healthy),
                'diseased_count': len(diseased),
                'candidate_fruits': candidates,
                'selected_fruits': selected,
                'healthy_fruits': healthy,
                'diseased_fruits': diseased,
                'selected_source_components': [
                    fruits[index]['source_components'] for index in selected],
                'selected_fruit_centers': [
                    fruits[index]['center'] for index in selected],
                'retained_leaf_count': len(retained_leaves),
                'retained_leaf_centers': [
                    leaf['center'] for leaf in retained_leaves],
            }

        if not manifest['trees']:
            raise ValueError('orchard world does not include any apple_tree models')
        ET.indent(world_tree.getroot(), space='  ')
        generated_world = staging / 'orchard.world'
        world_tree.write(generated_world, encoding='utf-8', xml_declaration=True)
        (staging / 'manifest.json').write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding='utf-8')
        if destination.exists():
            shutil.rmtree(destination)
        os.replace(staging, destination)
        return destination / 'orchard.world'
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
