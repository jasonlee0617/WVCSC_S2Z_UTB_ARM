#!/usr/bin/env python3
"""Safely migrate a schema-v2 measured-site mission to schema v3."""

import argparse

from wvcsc_bringup.site_mission import (
    atomic_write_site,
    migrate_site_document,
)
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', required=True)
    parser.add_argument('--map', required=True)
    args = parser.parse_args()
    try:
        with open(args.file, encoding='utf-8') as stream:
            legacy = yaml.safe_load(stream) or {}
        document = migrate_site_document(legacy, args.map)
        atomic_write_site(args.file, document, backup=True)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'[FAIL] site migration: {error}')
        return 1
    print(f'[OK] migrated {args.file} to schema_version 3')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
