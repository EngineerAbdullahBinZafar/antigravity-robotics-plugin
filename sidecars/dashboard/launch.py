"""
Sidecar Launcher for ROS 2 Robotics Dashboard in Antigravity.

Guarantees flawless sidecar execution inside Antigravity by:
1. Resolving the absolute plugin root and adding it to sys.path.
2. Ingesting .venv site-packages to ensure all dependencies (aiohttp, websocket-client) are present.
3. Gracefully falling back to the isolated virtual environment interpreter if needed.
"""

from __future__ import annotations

import os
import sys

# 1. Resolve absolute paths
sidecar_dir = os.path.dirname(os.path.abspath(__file__))
plugin_root = os.path.abspath(os.path.join(sidecar_dir, "..", ".."))

if plugin_root not in sys.path:
    sys.path.insert(0, plugin_root)

# 2. Ingest virtual environment packages
venv_site_packages = os.path.join(plugin_root, ".venv", "Lib", "site-packages")
if os.path.isdir(venv_site_packages) and venv_site_packages not in sys.path:
    sys.path.insert(0, venv_site_packages)

# 3. Verify aiohttp availability or re-exec with venv python
try:
    import aiohttp
except ImportError:
    venv_python = os.path.join(plugin_root, ".venv", "Scripts", "python.exe")
    if os.path.isfile(venv_python) and sys.executable.lower() != venv_python.lower():
        import subprocess
        result = subprocess.run([venv_python, os.path.abspath(__file__)] + sys.argv[1:])
        sys.exit(result.returncode)
    else:
        raise

# 4. Launch the dashboard server
from ros2_mcp.dashboard.server import main

if __name__ == "__main__":
    sys.exit(main())
