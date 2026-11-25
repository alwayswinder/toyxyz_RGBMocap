"""Compatibility entry point for existing BVH-to-Max automation."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

if __name__ == '__main__':
    from _MyTools.export.bvh_to_max import main

    main()
