#!/usr/bin/env python3
"""Validate canonical GO2 launch graphs without starting ROS nodes."""

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / "src" / "go2_system_bringup" / "launch"

EXPECTED = {
    "go2_mapping.launch": {"laserMapping", "go2_pointcloud_mapper"},
    "go2_localization.launch": {
        "laserMapping",
        "fast_lio_localization",
        "go2_localization_guard",
    },
    "go2_navigation.launch": {"move_base", "go2_cmd_watchdog"},
}

FORBIDDEN_PACKAGES = {
    "ego_planner",
    "ego_planner_node",
    "go2_sdk_bridge",
}

FORBIDDEN_TOKENS = ("ego", "sdk_bridge_real")


def expanded_nodes(launch_name):
    import roslaunch.config

    launch_path = LAUNCH_DIR / launch_name
    config = roslaunch.config.load_config_default(
        [(str(launch_path), ["start_lidar:=false"])],
        None,
    )
    return config.nodes


def validate():
    failures = []
    for launch_name, expected in EXPECTED.items():
        launch_text = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")
        if "ego" in launch_text.lower():
            failures.append(f"{launch_name}: active EGO reference")
        try:
            nodes = expanded_nodes(launch_name)
        except Exception as error:
            failures.append(f"{launch_name}: roslaunch failed: {error}")
            continue
        actual = {node.name.rsplit("/", 1)[-1] for node in nodes}
        missing = expected - actual
        if missing:
            failures.append(f"{launch_name}: missing nodes {sorted(missing)}")
        for node in nodes:
            identity = f"{node.package}/{node.type}/{node.name}".lower()
            if node.package.lower() in FORBIDDEN_PACKAGES:
                failures.append(
                    f"{launch_name}: forbidden package {node.package}"
                )
            if any(token in identity for token in FORBIDDEN_TOKENS):
                failures.append(
                    f"{launch_name}: forbidden expanded node {identity}"
                )
        print(f"PASS {launch_name}: {', '.join(sorted(actual))}")
    return failures


def main():
    failures = validate()
    if failures:
        for failure in failures:
            print(f"FAIL {failure}", file=sys.stderr)
        return 1
    print("All canonical launch contracts passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
