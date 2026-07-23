#!/usr/bin/env python3
"""Validate a measured mission against the exact selected occupancy map."""

import argparse
from pathlib import Path

from wvcsc_bringup.site_mission import load_site_document, validate_site_document


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', default='~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml')
    parser.add_argument('--map', required=True)
    args = parser.parse_args()
    try:
        document = load_site_document(args.file)
        validate_site_document(document, args.map)
    except ValueError as error:
        print(f'[FAIL] measured site mission: {error}')
        return 1
    mission = document['mission']
    print(
        f"[OK] site={document['site_id']} mission={mission['mission_id']} "
        f"targets={len(mission['targets'])} file={Path(args.file).expanduser()}")
    for target in mission['targets']:
        docking = target['docking_pose']
        offset = target['tree_offset_arm_base_m']
        print(
            f"  {target['target_id']}: dock=({docking['x']:.3f},"
            f"{docking['y']:.3f},{docking['yaw']:.3f}) "
            f"tree_base_xy=({offset['x_m']:.3f},{offset['y_m']:.3f}) "
            f"tree_z={target['tree_base_z_m']:.3f}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
