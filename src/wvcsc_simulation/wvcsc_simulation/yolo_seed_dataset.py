"""Write and validate raw Gazebo C10 captures for later manual annotation."""

from datetime import datetime, timezone
from pathlib import Path
import re

import cv2
import yaml


CLASS_NAMES = {0: 'tree', 1: 'healthy_fruit', 2: 'diseased_fruit'}
IMAGE_SIZE = (1280, 720)


def _sample_name(value):
    value = re.sub(r'[^A-Za-z0-9_.-]+', '_', value).strip('_.')
    if not value:
        raise ValueError('sample_name must not be empty')
    return value


def _manifest(path):
    if path.exists():
        return yaml.safe_load(path.read_text(encoding='utf-8'))
    return {
        'version': 1,
        'source_topic': '/camera/camera/color/image_rect_raw',
        'image_size': list(IMAGE_SIZE),
        'classes': CLASS_NAMES,
        'samples': [],
    }


def _write_data_yaml(root):
    (root / 'data.yaml').write_text(yaml.safe_dump({
        'path': '.',
        'unlabeled': 'images/unlabeled',
        'names': CLASS_NAMES,
    }, sort_keys=False), encoding='utf-8')


def write_unlabeled_sample(root, image, sample_name, metadata):
    """Write one 1280x720 C10 frame without labels or automatic segmentation."""
    if image is None or image.shape[:2] != (IMAGE_SIZE[1], IMAGE_SIZE[0]):
        raise ValueError('unlabeled image must be 1280x720')
    root = Path(root)
    image_path = root / 'images' / 'unlabeled' / f'{_sample_name(sample_name)}.png'
    image_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(image_path), image):
        raise OSError(f'failed to write {image_path}')
    manifest_path = root / 'manifest.yaml'
    manifest = _manifest(manifest_path)
    record = {
        'image': image_path.relative_to(root).as_posix(),
        'annotation_status': 'unlabeled',
        'captured_at': datetime.now(timezone.utc).isoformat(),
        **metadata,
    }
    manifest['samples'] = [
        sample for sample in manifest['samples'] if sample.get('image') != record['image']]
    manifest['samples'].append(record)
    manifest['samples'].sort(key=lambda sample: sample['image'])
    manifest_path.write_text(yaml.safe_dump(
        manifest, allow_unicode=True, sort_keys=False), encoding='utf-8')
    _write_data_yaml(root)
    return record


def validate_unlabeled_dataset(root, expected=30):
    """Validate only the intentional raw-capture contract."""
    root = Path(root)
    errors = []
    images = sorted((root / 'images' / 'unlabeled').glob('*.png'))
    if len(images) != expected:
        errors.append(f'unlabeled: expected {expected} images, found {len(images)}')
    for image_path in images:
        image = cv2.imread(str(image_path))
        if image is None or image.shape[:2] != (IMAGE_SIZE[1], IMAGE_SIZE[0]):
            errors.append(f'{image_path}: expected 1280x720 image')
    for forbidden in (root / 'labels', root / 'previews',
                      root / 'images' / 'train', root / 'images' / 'val'):
        if forbidden.exists():
            errors.append(f'{forbidden}: must not exist')
    if list(root.rglob('*.txt')):
        errors.append('dataset must not contain .txt labels')
    data = yaml.safe_load((root / 'data.yaml').read_text(encoding='utf-8')) \
        if (root / 'data.yaml').exists() else {}
    if data != {'path': '.', 'unlabeled': 'images/unlabeled', 'names': CLASS_NAMES}:
        errors.append('data.yaml does not match the unlabelled class contract')
    samples = _manifest(root / 'manifest.yaml').get('samples', [])
    image_paths = {path.relative_to(root).as_posix() for path in images}
    if {sample.get('image') for sample in samples} != image_paths:
        errors.append('manifest image paths do not match captured images')
    for sample in samples:
        if sample.get('annotation_status') != 'unlabeled':
            errors.append(f'{sample.get("image")}: must be unlabelled')
        if set(sample).intersection({'label', 'preview', 'instances'}):
            errors.append(f'{sample.get("image")}: contains annotation metadata')
    if expected == 30:
        seed_counts = {}
        side_counts = {'left': 0, 'right': 0}
        for sample in samples:
            seed = sample.get('orchard_seed')
            seed_counts[seed] = seed_counts.get(seed, 0) + 1
            side = sample.get('spray_side')
            if side in side_counts:
                side_counts[side] += 1
            else:
                errors.append(f'{sample.get("image")}: invalid spray_side {side}')
            pose = sample.get('camera_pose')
            if not isinstance(pose, dict) or pose.get('frame_id') != 'map':
                errors.append(f'{sample.get("image")}: invalid camera_pose')
        if seed_counts != {seed: 6 for seed in range(50, 55)}:
            errors.append(f'expected seeds 50..54 with 6 images each, found {seed_counts}')
        if side_counts != {'left': 15, 'right': 15}:
            errors.append(f'expected 15 images per side, found {side_counts}')
    if errors:
        raise ValueError('\n'.join(errors))
    return {'images': len(images)}
