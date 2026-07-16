import cv2
import numpy as np
import yaml

from wvcsc_simulation.yolo_seed_dataset import (
    FRUIT_SEG_CLASS_NAMES,
    validate_fruit_seg_dataset,
    validate_unlabeled_dataset,
    write_fruit_seg_sample,
    write_unlabeled_sample,
)


def _image():
    return np.full((720, 1280, 3), (90, 125, 95), dtype=np.uint8)


def test_raw_capture_contains_no_automatic_label_artifacts(tmp_path):
    record = write_unlabeled_sample(
        tmp_path, _image(), 'raw_tree', {'tree_id': 'tree_01'})
    assert record['image'] == 'images/unlabeled/raw_tree.png'
    assert set(record).isdisjoint({'label', 'preview', 'instances'})
    assert not (tmp_path / 'labels').exists()
    assert not (tmp_path / 'previews').exists()


def test_validation_accepts_only_raw_images_and_matching_manifest(tmp_path):
    for index in range(2):
        write_unlabeled_sample(tmp_path, _image(), f'capture_{index}', {})
    assert validate_unlabeled_dataset(tmp_path, expected=2) == {'images': 2}
    cv2.imwrite(str(tmp_path / 'images' / 'unlabeled' / 'wrong_size.png'),
                np.zeros((10, 10, 3), dtype=np.uint8))
    try:
        validate_unlabeled_dataset(tmp_path, expected=3)
    except ValueError as error:
        assert '1280x720' in str(error)
    else:
        raise AssertionError('invalid image size must fail validation')


def test_fruit_seg_capture_is_ready_for_manual_annotation(tmp_path):
    train = write_fruit_seg_sample(
        tmp_path, _image(), 'train_tree', 'train', {'tree_id': 'left_tree_01'})
    val = write_fruit_seg_sample(
        tmp_path, _image(), 'val_tree', 'val', {'tree_id': 'right_tree_01'})
    assert train['image'] == 'images/train/train_tree.png'
    assert val['annotation_status'] == 'pending'
    assert validate_fruit_seg_dataset(
        tmp_path, expected_train=1, expected_val=1) == {'train': 1, 'val': 1}
    data = yaml.safe_load(
        (tmp_path / 'data.yaml').read_text(encoding='utf-8'))
    assert data['names'] == FRUIT_SEG_CLASS_NAMES
    assert not list((tmp_path / 'labels').rglob('*.txt'))
