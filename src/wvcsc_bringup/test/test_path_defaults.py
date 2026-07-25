import pytest

from wvcsc_bringup import path_defaults


def test_latest_timestamp_file_uses_timestamp_not_directory_order(tmp_path):
    older = tmp_path / 'map_20260724_090000'
    newer = tmp_path / 'map_20260724_100000'
    older.mkdir()
    newer.mkdir()
    (older / 'orchard.yaml').write_text('old', encoding='utf-8')
    (newer / 'orchard.yaml').write_text('new', encoding='utf-8')
    (tmp_path / 'map_invalid').mkdir()
    assert path_defaults._latest_timestamp_file(
        tmp_path, 'map', 'orchard.yaml') == newer / 'orchard.yaml'


def test_missing_timestamped_resources_fail_clearly(tmp_path, monkeypatch):
    monkeypatch.setattr(
        path_defaults, '_candidate_roots', lambda _package, _relative: (tmp_path,))
    with pytest.raises(RuntimeError, match='no timestamped map found'):
        path_defaults.latest_map_yaml()
