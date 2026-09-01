from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_yaml(path):
    return yaml.safe_load((ROOT / path).read_text(encoding="utf-8"))


def test_go2_geometry_is_robot_specific():
    cfg = load_yaml("src/go2_system_bringup/config/go2_geometry.yaml")
    assert cfg["robot"]["name"] == "go2"
    assert cfg["robot"]["footprint"]["length"] == 0.70
    assert cfg["robot"]["footprint"]["width"] == 0.31
    assert cfg["robot"]["holonomic"] is False


def test_public_topic_and_frame_contract():
    cfg = load_yaml("src/go2_system_bringup/config/go2_geometry.yaml")
    assert cfg["topics"] == {
        "lio_odom": "/lio/odometry",
        "lio_cloud_body": "/lio/cloud_registered_body",
        "nav_odom": "/odom_nav",
        "cloud_base": "/cloud_registered_base",
        "planner_cmd": "/cmd_vel_nav",
        "safe_cmd": "/cmd_vel_safe",
    }
    assert cfg["frames"] == {
        "map": "map",
        "odom": "odom",
        "base_footprint": "base_footprint",
        "base_link": "base_link",
        "lidar": "lidar_link",
    }


def test_motion_limits_are_consistent_and_lateral_motion_is_disabled():
    expected = {
        "max_vx": 0.20,
        "max_vy": 0.0,
        "max_wz": 0.30,
        "cmd_timeout_sec": 0.50,
    }
    geometry = load_yaml("src/go2_system_bringup/config/go2_geometry.yaml")
    watchdog = load_yaml("src/go2_cmd_watchdog/config/watchdog.yaml")
    assert geometry["motion_limits"] == expected
    assert watchdog == expected

    watchdog_cpp = (
        ROOT / "src/go2_cmd_watchdog/src/cmd_watchdog.cpp"
    ).read_text(encoding="utf-8")
    bridge_cpp = (
        ROOT / "src/go2_sdk_bridge/src/sdk_bridge_real.cpp"
    ).read_text(encoding="utf-8")
    for source in (watchdog_cpp, bridge_cpp):
        assert '"max_vy",\n        max_vy_,\n        0.00' in source
        assert '"max_vx",\n        max_vx_,\n        0.20' in source
        assert '"max_wz",\n        max_wz_,\n        0.30' in source


def test_unitree_sdk_path_is_repository_relative_and_validated():
    cmake = (ROOT / "src/go2_sdk_bridge/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    assert "${CMAKE_CURRENT_SOURCE_DIR}/../Unitree_SDK2" in cmake
    assert "CACHE PATH" in cmake
    assert "if(NOT EXISTS" in cmake
    assert "/home/nvidia/unitree_sdk2_src" not in cmake


def test_extrinsics_are_kept_in_the_active_workspace():
    cfg = load_yaml("src/go2_system_bringup/config/extrinsics.yaml")
    assert cfg["robot_id"] == "go2_edu_02"
    assert cfg["mid360_mount"]["base_link_to_lidar_link"]["pitch_deg"] == 39
