#!/usr/bin/env python3
"""Validate a schema-v4 real five-point field route before hardware launch."""

import argparse
import sys

from wvcsc_bringup.field_route import (
    load_field_route_document,
    validate_field_route_document,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', required=True)
    parser.add_argument('--map', required=True)
    args = parser.parse_args()
    try:
        document = load_field_route_document(args.file)
        steps = validate_field_route_document(document, args.map)
    except ValueError as error:
        print(f'[FIELD_ROUTE][INVALID] {error}', file=sys.stderr)
        return 1
    print(
        f'[FIELD_ROUTE][VALID] mission={document["mission"]["mission_id"]} '
        f'steps={",".join(step.point_id for step in steps)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
