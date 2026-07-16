import cv2
import numpy as np

from wvcsc_simulation.yolo_seed_dataset import (
    validate_unlabeled_dataset,
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
