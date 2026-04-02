#!/usr/bin/env python3
"""Lightweight self-check for MapReader output."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from MapReader import MapReader


def main() -> None:
    map_file = REPO_ROOT / 'v2.smap'
    reader = MapReader(str(map_file))
    reader.run()

    if not reader.map_x or not reader.map_y:
        print('No map points loaded.')
        return

    print(f"map_x count: {len(reader.map_x)}")
    print(f"map_y count: {len(reader.map_y)}")
    print(f"x range: [{min(reader.map_x):.3f}, {max(reader.map_x):.3f}]")
    print(f"y range: [{min(reader.map_y):.3f}, {max(reader.map_y):.3f}]")


if __name__ == '__main__':
    main()
