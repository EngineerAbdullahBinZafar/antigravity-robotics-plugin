"""
CLI entrypoint for running the dashboard via `python -m ros2_mcp.dashboard`.
"""

from __future__ import annotations

import sys

from .server import main

if __name__ == "__main__":
    sys.exit(main())
