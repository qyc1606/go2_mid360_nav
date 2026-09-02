from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_yaml(path):
    return yaml.safe_load((ROOT / path).read_text(encoding="utf-8"))


def test_teb_is_non_holonomic_and_bounded():
    teb = load_yaml(
        "src/go2_navigation/config/teb_local_planner.yaml"
    )["TebLocalPlannerROS"]
    assert teb["max_vel_y"] == 0.0
    assert teb["acc_lim_y"] == 0.0
    assert teb["max_vel_x"] <= 0.20
    assert teb["max_vel_theta"] <= 0.30
    assert teb["odom_topic"] == "/odom_nav"
    assert teb["min_turning_radius"] == 0.0


def test_navigation_uses_go2_footprint_and_frames():
    geometry = load_yaml(
        "src/go2_system_bringup/config/go2_geometry.yaml"
    )
    costmap = load_yaml("src/go2_navigation/config/costmap_common.yaml")
    teb = load_yaml(
        "src/go2_navigation/config/teb_local_planner.yaml"
    )["TebLocalPlannerROS"]
    expected = geometry["robot"]["footprint"]["vertices"]
    assert costmap["footprint"] == expected
    assert teb["footprint_model"]["vertices"] == expected
    assert load_yaml("src/go2_navigation/config/global_costmap.yaml")[
        "global_frame"
    ] == "map"
    assert load_yaml("src/go2_navigation/config/local_costmap.yaml")[
        "global_frame"
    ] == "odom"


def test_move_base_output_enters_watchdog_only():
    text = (
        ROOT / "src/go2_navigation/launch/navigation_teb.launch"
    ).read_text(encoding="utf-8")
    assert 'from="cmd_vel" to="/cmd_vel_nav"' in text
    assert "/cmd_vel_safe" not in text
    assert "go2_cmd_watchdog" in text
    assert "go2_sdk_bridge" not in text


def test_navigation_uses_named_go2_map_and_public_cloud():
    text = (
        ROOT / "src/go2_navigation/launch/navigation_teb.launch"
    ).read_text(encoding="utf-8")
    assert 'name="map_root" default="$(env HOME)/go2_mid360_nav/maps"' in text
    assert "/cloud_registered_base" in text
    assert "livox_fastlio" not in text
    assert "/scout/odom" not in text


def test_navigation_declares_runtime_dependencies():
    manifest = (
        ROOT / "src/go2_navigation/package.xml"
    ).read_text(encoding="utf-8")
    for package in (
        "move_base",
        "map_server",
        "global_planner",
        "costmap_2d",
        "teb_local_planner",
        "go2_cmd_watchdog",
    ):
        assert f"<exec_depend>{package}</exec_depend>" in manifest
