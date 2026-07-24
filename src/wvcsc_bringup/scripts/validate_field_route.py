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
    parser.add_argument(
        '--strict', action='store_true',
        help='also enforce capture quality and map footprint gates')
    args = parser.parse_args()
    try:
        document = load_field_route_document(args.file)
        steps = validate_field_route_document(
            document,
            args.map,
            require_capture_quality=args.strict,
            require_free_space=args.strict,
        )
    except ValueError as error:
        print(f'[FIELD_ROUTE][INVALID] {error}', file=sys.stderr)
        return 1
    print(
        f'[FIELD_ROUTE][VALID] mission={document["mission"]["mission_id"]} '
        f'steps={",".join(step.point_id for step in steps)}')
    if not args.strict:
        print('[FIELD_ROUTE][WARN] capture quality and footprint gates are disabled')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
