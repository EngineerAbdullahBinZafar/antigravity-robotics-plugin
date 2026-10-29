"""
MCP Tools for ROS2 Node introspection.

Fix BUG-11: get_node_info previously used a case-insensitive substring match
on the string representation of the entire topic dict — which could match
a node name appearing anywhere in any value (including unrelated fields).

Now uses proper ROS2 naming: exact match on the node name component
extracted from the (name, namespace) tuples returned by list_nodes().
"""

from __future__ import annotations

from typing import Any, Dict, List


def handle_list_nodes(ros2: Any) -> Dict:
    """List all currently running ROS2 nodes with their namespaces."""
    raw = ros2.list_nodes()
    nodes = []
    for entry in raw:
        if isinstance(entry, (tuple, list)) and len(entry) == 2:
            name, ns = entry
            nodes.append(
                {
                    "name": name,
                    "namespace": ns,
                    "full_name": f"{ns}{name}" if ns != "/" else f"/{name}",
                }
            )
        else:
            # Fallback: raw string node name (old mock format)
            nodes.append({"name": str(entry), "namespace": "/", "full_name": f"/{entry}"})
    return {
        "total": len(nodes),
        "nodes": sorted(nodes, key=lambda n: n["full_name"]),
    }


def handle_get_node_info(ros2: Any, node_name: str) -> Dict:
    """
    Inspect a specific ROS2 node and return its related topics.

    FIX BUG-11: Uses exact name matching against the (name, namespace) tuples
    from list_nodes(), NOT a string-repr substring match.
    """
    # Normalize: strip leading slash if the caller included it
    clean_name = node_name.lstrip("/")

    # Exact match on node name field
    raw = ros2.list_nodes()
    found = False
    for entry in raw:
        if isinstance(entry, (tuple, list)) and len(entry) == 2:
            name, _ = entry
            if name == clean_name:
                found = True
                break
        elif str(entry) == clean_name:
            found = True
            break

    if not found:
        return {
            "node": node_name,
            "error": f"Node '{clean_name}' not found. Use list_nodes to see active nodes.",
            "hint": "ROS2 node names are case-sensitive.",
        }

    # Find topics whose name contains the node name as a path component
    all_topics = ros2.list_topics()
    related: List[Dict] = []
    for t in all_topics:
        topic_name: str = t.get("topic", "")
        # Match on path segment, not substring of the whole dict
        segments = topic_name.strip("/").split("/")
        if clean_name in segments or topic_name.startswith(f"/{clean_name}/"):
            related.append(t)

    return {
        "node": node_name,
        "found": True,
        "related_topics": related,
        "related_count": len(related),
        "note": (
            "Topic associations are inferred from naming conventions. "
            "For the authoritative publisher/subscriber graph, "
            "use 'ros2 node info' in a live ROS2 environment."
        ),
    }
