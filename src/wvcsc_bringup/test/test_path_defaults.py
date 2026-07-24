from pathlib import Path

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


def test_latest_route_ignores_example_files(tmp_path, monkeypatch):
    root = tmp_path / 'real'
    route_dir = root / 'mission_20260724_100000'
    route_dir.mkdir(parents=True)
    (route_dir / 'field_route_corn.example.yaml').write_text(
        'example', encoding='utf-8')
    route = route_dir / 'field_route_corn.yaml'
    route.write_text('route', encoding='utf-8')
    monkeypatch.setattr(
        path_defaults, '_candidate_roots', lambda _package, _relative: (root,))
    assert Path(path_defaults.latest_field_route()) == route


def test_missing_timestamped_resources_fail_clearly(tmp_path, monkeypatch):
    monkeypatch.setattr(
        path_defaults, '_candidate_roots', lambda _package, _relative: (tmp_path,))
    with pytest.raises(RuntimeError, match='no timestamped map found'):
        path_defaults.latest_map_yaml()
    with pytest.raises(RuntimeError, match='no timestamped field route found'):
        path_defaults.latest_field_route()
