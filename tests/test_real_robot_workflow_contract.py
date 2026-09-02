from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_real_sdk_launch_is_explicit_disabled_by_default_interface():
    launch = (
        ROOT / "src/go2_sdk_bridge/launch/sdk_bridge_real.launch"
    ).read_text(encoding="utf-8")
    for value in (
        'type="go2_sdk_bridge_real_node"',
        'name="go2_sdk_bridge_real"',
        'name="network_interface" default="eth1"',
        'name="command_topic" value="/cmd_vel_safe"',
        'name="localization_ok_topic" value="/localization/ok"',
        'name="max_vy" value="0.0"',
    ):
        assert value in launch
    assert 'name="enabled"' not in launch.lower()


def test_canonical_navigation_does_not_auto_start_real_sdk():
    launch = (
        ROOT / "src/go2_system_bringup/launch/go2_navigation.launch"
    ).read_text(encoding="utf-8")
    assert "sdk_bridge_real" not in launch
    assert "go2_sdk_bridge" not in launch


def test_real_sdk_shutdown_and_low_speed_defaults_are_safe():
    source = (
        ROOT / "src/go2_sdk_bridge/src/sdk_bridge_real.cpp"
    ).read_text(encoding="utf-8")
    assert "~Go2SdkBridgeReal" in source
    assert "GO2 SDK bridge shutdown: StopMove sent" in source
    assert '"min_walk_vx"' in source
    assert "0.05" in source


def test_manual_contains_complete_real_robot_command_sequence():
    manual = (
        ROOT / "docs/GO2_使用手册_V1.md"
    ).read_text(encoding="utf-8")
    required = (
        "git branch --show-current",
        "roslaunch go2_system_bringup go2_mapping.launch",
        "rosservice call /go2_pointcloud_mapper/save_map",
        "python3 scripts/finalize_map.py",
        "roslaunch go2_system_bringup go2_localization.launch",
        "rosrun tf tf_echo map base_link",
        "roslaunch go2_system_bringup go2_navigation.launch",
        "python3 scripts/check_cmd_vel.py",
        "roslaunch go2_sdk_bridge sdk_bridge_real.launch",
        'rosservice call /go2_sdk_bridge_real/enable "data: true"',
        'rosservice call /go2_sdk_bridge_real/enable "data: false"',
        "Ctrl+C",
        "eth1",
        "/localization/ok",
        "/cmd_vel_nav",
        "/cmd_vel_safe",
    )
    for value in required:
        assert value in manual


def test_manual_requires_planning_dry_run_before_sdk_enable():
    manual = (
        ROOT / "docs/GO2_使用手册_V1.md"
    ).read_text(encoding="utf-8")
    dry_run = manual.index("规划空跑")
    sdk_start = manual.index("启动真实 SDK2")
    sdk_enable = manual.index(
        'rosservice call /go2_sdk_bridge_real/enable "data: true"'
    )
    assert dry_run < sdk_start < sdk_enable
