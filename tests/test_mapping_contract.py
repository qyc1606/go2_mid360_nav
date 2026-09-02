import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mapping_disables_fast_lio_native_pcd():
    text = (
        ROOT / "src/go2_system_bringup/launch/go2_mapping.launch"
    ).read_text(encoding="utf-8")
    assert 'name="pcd_save/pcd_save_en" value="false"' in text
    assert "go2_pointcloud_mapper" in text


def test_mapping_uses_named_map_directory_and_go2_topics():
    text = (
        ROOT / "src/go2_system_bringup/launch/go2_mapping.launch"
    ).read_text(encoding="utf-8")
    for fragment in (
        '<arg name="map_name" default="current_mapping"/>',
        '<arg name="map_root" default="$(env HOME)/go2_mid360_nav/maps"/>',
        '<arg name="map_dir" default="$(arg map_root)/$(arg map_name)"/>',
        "/cloud_registered_odom",
        "/odom_nav",
        "$(arg map_dir)/public_map.pcd",
    ):
        assert fragment in text


def test_mapping_expands_mapper_with_world_cloud_matching_odom_frame():
    import roslaunch.config

    launch_path = (
        ROOT / "src/go2_system_bringup/launch/go2_mapping.launch"
    )
    config = roslaunch.config.load_config_default(
        [
            (
                str(launch_path),
                ["start_lidar:=false", "map_name:=contract_test"],
            )
        ],
        None,
    )
    assert (
        config.params["/go2_pointcloud_mapper/input_cloud"].value
        == "/cloud_registered_odom"
    )
    assert (
        config.params["/go2_pointcloud_mapper/input_odom"].value
        == "/odom_nav"
    )


def test_mapper_only_writes_on_explicit_save_request():
    cfg = yaml.safe_load(
        (
            ROOT / "src/go2_pointcloud_mapper/config/mapper.yaml"
        ).read_text(encoding="utf-8")
    )
    assert cfg["input_cloud"] == "/cloud_registered_odom"
    assert cfg["input_odom"] == "/odom_nav"
    assert cfg["map"]["autosave_period"] == 0.0
    assert cfg["map"]["save_on_shutdown"] is False


def test_mapping_has_no_absolute_project_path():
    for base in (
        ROOT / "src/go2_pointcloud_mapper",
        ROOT / "src/go2_map_tools",
    ):
        for path in base.rglob("*"):
            if path.is_file():
                text = path.read_text(errors="ignore")
                assert "/home/nvidia/go2_mid360_nav" not in text, path
                assert "livox_fastlio/maps" not in text, path


def test_finalize_map_uses_atomic_replacement():
    script = (ROOT / "scripts/finalize_map.py").read_text(encoding="utf-8")
    assert "--map-root" in script
    assert "os.replace" in script
    assert "public_map.pcd" in script
    assert "map_raw.pgm" in script
    assert "map_raw.yaml" in script


def test_finalize_map_applies_distinct_raw_and_navigation_configs():
    finalize = load_script("finalize_map.py")
    source = Path("/maps/site01/public_map.pcd")
    raw = finalize.build_converter_command(
        source,
        Path("/tmp/map_raw.pgm"),
        Path("/tmp/map_raw.yaml"),
        ROOT / "src/go2_map_tools/config/corridor_raw.yaml",
    )
    nav = finalize.build_converter_command(
        source,
        Path("/tmp/map.pgm"),
        Path("/tmp/map.yaml"),
        ROOT / "src/go2_map_tools/config/corridor_nav.yaml",
    )
    for command in (raw, nav):
        assert "_floor_min_z:=-0.55" in command
        assert "_floor_max_z:=-0.05" in command
        assert "_free_dilation_m:=0.2" in command
    assert "_obstacle_inflation_m:=0.0" in raw
    assert "_obstacle_inflation_m:=0.3" in nav
