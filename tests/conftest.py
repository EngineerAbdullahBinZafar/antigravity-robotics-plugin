"""
Pytest configuration and shared fixtures for antigravity-robotics-plugin tests.
"""

import os
import sys

# Ensure plugin root is on sys.path
PLUGIN_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PLUGIN_ROOT not in sys.path:
    sys.path.insert(0, PLUGIN_ROOT)
