#!/usr/bin/env python3
"""Validate canonical GO2 launch graphs without starting ROS nodes."""

import subprocess
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


def listed_nodes(launch_name):
    result = subprocess.run(
        [
            "roslaunch",
            "--nodes",
            "go2_system_bringup",
            launch_name,
            "start_lidar:=false",
        ],
        check=False,
        text=True,
        capture_output=True,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return {
        line.strip().rsplit("/", 1)[-1]
        for line in result.stdout.splitlines()
        if line.strip().startswith("/")
    }


def validate():
    failures = []
    for launch_name, expected in EXPECTED.items():
        launch_text = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")
        if "ego" in launch_text.lower():
            failures.append(f"{launch_name}: active EGO reference")
        try:
            actual = listed_nodes(launch_name)
        except RuntimeError as error:
            failures.append(f"{launch_name}: roslaunch failed: {error}")
            continue
        missing = expected - actual
        if missing:
            failures.append(f"{launch_name}: missing nodes {sorted(missing)}")
        if any("ego" in node.lower() for node in actual):
            failures.append(f"{launch_name}: EGO node remains active")
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
