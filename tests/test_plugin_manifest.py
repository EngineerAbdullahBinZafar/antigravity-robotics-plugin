"""
Automated manifest, configuration, and packaging verification tests
for antigravity-robotics-plugin.
"""

import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

PLUGIN_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def test_plugin_json_schema_and_fields():
    """Verify plugin.json exists and adheres to the Antigravity plugin manifest specification."""
    plugin_path = os.path.join(PLUGIN_DIR, "plugin.json")
    assert os.path.isfile(plugin_path), "plugin.json must exist at plugin root"

    with open(plugin_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Core required fields
    assert data["name"] == "antigravity-robotics-plugin", "name must match plugin directory name"
    assert data["displayName"] == "Antigravity ROS 2 Robotics"
    assert data["version"] == "1.2.0"
    assert len(data.get("description", "")) > 10

    # Logo must be relative path
    logo = data.get("logo")
    assert logo, "plugin.json must specify a logo"
    assert not logo.startswith("http://") and not logo.startswith("https://"), (
        "logo cannot be a URL"
    )
    assert not os.path.isabs(logo), "logo cannot be an absolute path"
    assert os.path.isfile(os.path.join(PLUGIN_DIR, logo)), (
        f"Logo file {logo} does not exist on disk"
    )

    # Suggested prompts: exactly 3 starter prompts recommended for UI cards
    prompts = data.get("suggestedPrompts", [])
    assert isinstance(prompts, list), "suggestedPrompts must be a list"
    assert len(prompts) == 3, f"suggestedPrompts must contain exactly 3 items, found {len(prompts)}"
    for p in prompts:
        assert isinstance(p, str) and len(p.strip()) > 0

    assert data.get("disabled") is False


def test_mcp_config_json_structure():
    """Verify mcp_config.json contains valid stdio server declaration."""
    mcp_path = os.path.join(PLUGIN_DIR, "mcp_config.json")
    assert os.path.isfile(mcp_path), "mcp_config.json must exist at plugin root"

    with open(mcp_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    assert "mcpServers" in data, "mcp_config.json must have 'mcpServers' object"
    assert "ros2" in data["mcpServers"], "mcpServers must contain 'ros2' server entry"

    ros2_cfg = data["mcpServers"]["ros2"]
    assert ros2_cfg.get("command") in ("python", "python3", sys.executable)
    assert "-m" in ros2_cfg.get("args", [])
    assert "ros2_mcp.server" in ros2_cfg.get("args", [])

    env = ros2_cfg.get("env", {})
    assert env.get("PYTHONUNBUFFERED") == "1", (
        "PYTHONUNBUFFERED must be set to 1 for unbuffered stdio"
    )
    assert env.get("PYTHONPATH") == ".", "PYTHONPATH must be set to '.' for module resolution"
    assert env.get("SAFETY_LEVEL") == "safe_write", "Default SAFETY_LEVEL must be 'safe_write'"


def test_rules_agents_md():
    """Verify rules/AGENTS.md meets size constraints and includes sidecar link convention."""
    rules_path = os.path.join(PLUGIN_DIR, "rules", "AGENTS.md")
    assert os.path.isfile(rules_path), "rules/AGENTS.md must exist"

    size = os.path.getsize(rules_path)
    assert size <= 24000, f"rules/AGENTS.md exceeds 24,000 bytes limit (got {size} bytes)"

    with open(rules_path, "r", encoding="utf-8") as f:
        content = f.read()

    assert "sidecar://antigravity-robotics-plugin/dashboard/index.html" in content, (
        "AGENTS.md must mandate the sidecar:// dashboard pill link"
    )
    assert "read_only" in content and "safe_write" in content and "full" in content, (
        "AGENTS.md must document 3-tier safety sandbox levels"
    )


def test_skill_markdown():
    """Verify skills/ros2-robotics/SKILL.md frontmatter and content."""
    skill_path = os.path.join(PLUGIN_DIR, "skills", "ros2-robotics", "SKILL.md")
    assert os.path.isfile(skill_path), "skills/ros2-robotics/SKILL.md must exist"

    with open(skill_path, "r", encoding="utf-8") as f:
        content = f.read()

    match = re.match(r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL)
    assert match, "SKILL.md must have valid YAML frontmatter enclosed in '---'"

    frontmatter = match.group(1)
    assert "name: ros2-robotics" in frontmatter
    assert "description:" in frontmatter
    assert "sidecar://antigravity-robotics-plugin/dashboard/index.html" in content


def test_logo_svg_validity():
    """Verify assets/logo.svg is valid XML and conforms to square viewBox."""
    logo_path = os.path.join(PLUGIN_DIR, "assets", "logo.svg")
    assert os.path.isfile(logo_path), "assets/logo.svg must exist"

    tree = ET.parse(logo_path)
    root = tree.getroot()
    assert root.tag.endswith("svg"), "Root tag of logo.svg must be <svg>"
    view_box = root.attrib.get("viewBox", "")
    assert "0 0 512 512" in view_box, f"viewBox should be '0 0 512 512', got '{view_box}'"


def test_packaging_metadata_files():
    """Verify pyproject.toml, setup.py, LICENSE, and README.md exist and are valid."""
    for filename in ("pyproject.toml", "setup.py", "LICENSE", "README.md", ".gitignore"):
        filepath = os.path.join(PLUGIN_DIR, filename)
        assert os.path.isfile(filepath), f"{filename} must exist at plugin root"

    with open(os.path.join(PLUGIN_DIR, "pyproject.toml"), "r", encoding="utf-8") as f:
        pyproj = f.read()
    assert 'name = "antigravity-robotics-plugin"' in pyproj
    assert 'version = "1.2.0"' in pyproj

    with open(os.path.join(PLUGIN_DIR, "setup.py"), "r", encoding="utf-8") as f:
        setup_content = f.read()
    assert 'name="antigravity-robotics-plugin"' in setup_content
    assert 'version="1.2.0"' in setup_content


def test_ros2_mcp_package_exports():
    """Verify ros2_mcp top-level package exports public API."""
    import ros2_mcp

    expected_exports = [
        "VERSION",
        "ROS2MCPServer",
        "CommandSandbox",
        "SafetyLevel",
        "SandboxBlockedError",
        "create_interface",
        "MockInterface",
        "ROS2NativeInterface",
        "ROS2_AVAILABLE",
        "ROSBRIDGE_AVAILABLE",
        "TOOL_SCHEMAS",
        "main",
    ]
    for export_name in expected_exports:
        assert hasattr(ros2_mcp, export_name), f"ros2_mcp missing export {export_name}"
    assert ros2_mcp.VERSION == "1.2.0"
    assert len(ros2_mcp.TOOL_SCHEMAS) == 16


def test_cli_doctor():
    """Verify python -m ros2_mcp.server --doctor executes with exit code 0."""
    result = subprocess.run(
        [sys.executable, "-m", "ros2_mcp.server", "--doctor"],
        cwd=PLUGIN_DIR,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"Doctor failed: {result.stderr}"
    assert "Diagnostic Doctor" in result.stdout
    assert "16 tools available" in result.stdout
    assert "All systems operational" in result.stdout


def test_cli_list_tools():
    """Verify python -m ros2_mcp.server --list-tools returns 16 valid tools as JSON."""
    result = subprocess.run(
        [sys.executable, "-m", "ros2_mcp.server", "--list-tools"],
        cwd=PLUGIN_DIR,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"--list-tools failed: {result.stderr}"
    tools = json.loads(result.stdout)
    assert len(tools) == 16
    tool_names = {t["name"] for t in tools}
    expected = {
        "ping",
        "system_diagnostics",
        "list_topics",
        "read_topic",
        "publish_topic",
        "get_robot_snapshot",
        "list_nodes",
        "get_node_info",
        "get_parameter",
        "set_parameter",
        "get_pid_state",
        "tune_pid",
        "predict_trajectory",
        "predictive_safety_check",
        "get_spatial_map",
        "swarm_fleet_status",
    }
    assert tool_names == expected
    for t in tools:
        assert "inputSchema" in t and isinstance(t["inputSchema"], dict)


def test_stdio_jsonrpc_handshake_and_hygiene():
    """Verify JSON-RPC 2.0 initialize, tools/list, and ping calls via stdin/stdout."""
    proc = subprocess.Popen(
        [sys.executable, "-m", "ros2_mcp.server"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=PLUGIN_DIR,
    )

    try:
        # 1. Initialize Request
        req_init = {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "initialize",
            "params": {"protocolVersion": "2024-11-05"},
        }
        proc.stdin.write(json.dumps(req_init) + "\n")
        proc.stdin.flush()
        line_init = proc.stdout.readline()
        assert line_init, "No output on stdout for initialize"
        resp_init = json.loads(line_init)
        assert resp_init.get("id") == 1
        assert resp_init.get("result", {}).get("protocolVersion") == "2024-11-05"

        # 2. Tools/List Request
        req_list = {"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}}
        proc.stdin.write(json.dumps(req_list) + "\n")
        proc.stdin.flush()
        line_list = proc.stdout.readline()
        resp_list = json.loads(line_list)
        assert len(resp_list.get("result", {}).get("tools", [])) == 16

        # 3. Ping Tool Call
        req_ping = {
            "jsonrpc": "2.0",
            "id": 3,
            "method": "tools/call",
            "params": {"name": "ping", "arguments": {}},
        }
        proc.stdin.write(json.dumps(req_ping) + "\n")
        proc.stdin.flush()
        line_ping = proc.stdout.readline()
        resp_ping = json.loads(line_ping)
        assert not resp_ping.get("result", {}).get("isError")
        payload = json.loads(resp_ping["result"]["content"][0]["text"])
        assert payload.get("status") == "ok"
        assert payload.get("mode") == "simulation"

        # 4. Diagnostics Tool Call
        req_diag = {
            "jsonrpc": "2.0",
            "id": 4,
            "method": "tools/call",
            "params": {"name": "system_diagnostics", "arguments": {}},
        }
        proc.stdin.write(json.dumps(req_diag) + "\n")
        proc.stdin.flush()
        line_diag = proc.stdout.readline()
        resp_diag = json.loads(line_diag)
        assert not resp_diag.get("result", {}).get("isError")
        diag_data = json.loads(resp_diag["result"]["content"][0]["text"])
        assert "system_status" in diag_data and "healthy_checks" in diag_data

        # 5. Malformed JSON handling
        proc.stdin.write("{invalid-json-payload\n")
        proc.stdin.flush()
        line_err = proc.stdout.readline()
        resp_err = json.loads(line_err)
        assert resp_err.get("error", {}).get("code") == -32700

    finally:
        proc.stdin.close()
        proc.wait(timeout=5)

    assert proc.returncode == 0
