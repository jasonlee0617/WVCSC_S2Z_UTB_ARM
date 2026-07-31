"""预检视觉模型契约的纯文件测试，不启动任何真实硬件。"""

from pathlib import Path
import sys

import yaml


SCRIPT_DIR = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))
import preflight_check  # noqa: E402


def _write_config(path, *, backend, model, class_id, class_name, strict=True):
    path.write_text(yaml.safe_dump({
        'wvcsc_perception_pipeline': {
            'ros__parameters': {
                'disease_model_backend': backend,
                'disease_model_path': model,
                'target_class_id': class_id,
                'model_target_class_name': class_name,
                'strict_model_classes': strict,
            },
        },
    }), encoding='utf-8')


def test_preflight_reads_detect_contract(tmp_path, monkeypatch):
    model_dir = tmp_path / 'models'
    model_dir.mkdir()
    config = tmp_path / 'vision.yaml'
    _write_config(
        config, backend='detect', model='best.pt', class_id=0,
        class_name='illness')
    monkeypatch.setattr(
        preflight_check, 'get_package_share_directory',
        lambda package: str(tmp_path) if package == 'wvcsc_rgb_vision'
        else (_ for _ in ()).throw(AssertionError(package)))

    failures = []
    contract = preflight_check._vision_contract(str(config), failures)

    assert failures == []
    assert contract[0] == tmp_path / 'models' / 'best.pt'
    assert contract[1:] == ('detect', {0: 'illness'}, True)


def test_preflight_reads_segment_contract_and_non_strict_classes(tmp_path, monkeypatch):
    model_dir = tmp_path / 'models'
    model_dir.mkdir()
    config = tmp_path / 'vision.yaml'
    _write_config(
        config, backend='segment', model='seg.pt', class_id=0,
        class_name='disease_leaf', strict=False)
    monkeypatch.setattr(
        preflight_check, 'get_package_share_directory',
        lambda package: str(tmp_path) if package == 'wvcsc_rgb_vision'
        else (_ for _ in ()).throw(AssertionError(package)))

    failures = []
    contract = preflight_check._vision_contract(str(config), failures)

    assert failures == []
    assert contract[0] == tmp_path / 'models' / 'seg.pt'
    assert contract[1:] == ('segment', {0: 'disease_leaf'}, False)


def test_preflight_rejects_unknown_backend(tmp_path):
    config = tmp_path / 'vision.yaml'
    _write_config(
        config, backend='classify', model='model.pt', class_id=0,
        class_name='illness')

    failures = []
    assert preflight_check._vision_contract(str(config), failures) is None
    assert any('unsupported vision backend' in item for item in failures)
