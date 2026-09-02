import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_canonical_launch_files_exist_without_ego_references():
    launch_dir = ROOT / "src/go2_system_bringup/launch"
    for name in (
        "go2_mapping.launch",
        "go2_localization.launch",
        "go2_navigation.launch",
    ):
        text = (launch_dir / name).read_text(encoding="utf-8")
        assert "ego" not in text.lower()
        assert "go2_sdk_bridge_real" not in text


def test_launch_validator_defines_required_node_subsets():
    validator = load_script("validate_launch_contracts.py")
    assert validator.EXPECTED == {
        "go2_mapping.launch": {
            "laserMapping",
            "go2_pointcloud_mapper",
        },
        "go2_localization.launch": {
            "laserMapping",
            "fast_lio_localization",
            "go2_localization_guard",
        },
        "go2_navigation.launch": {
            "move_base",
            "go2_cmd_watchdog",
        },
    }


def test_command_monitor_rejects_lateral_nonfinite_and_excess_commands():
    monitor = load_script("check_cmd_vel.py")
    assert monitor.validate_command(0.20, 0.0, 0.30) is None
    assert "lateral" in monitor.validate_command(0.0, 0.01, 0.0)
    assert "non-finite" in monitor.validate_command(float("nan"), 0.0, 0.0)
    assert "max_vx" in monitor.validate_command(0.21, 0.0, 0.0)
    assert "max_wz" in monitor.validate_command(0.0, 0.0, 0.31)


def test_navigation_bringup_requires_localization_and_keeps_sdk_separate():
    text = (
        ROOT / "src/go2_system_bringup/launch/go2_navigation.launch"
    ).read_text(encoding="utf-8")
    assert "go2_localization.launch" in text
    assert "navigation_teb.launch" in text
    assert "go2_sdk_bridge" not in text


def test_readme_documents_the_three_canonical_workflows():
    text = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "go2_mapping.launch" in text
    assert "go2_localization.launch" in text
    assert "go2_navigation.launch" in text
    assert "real Unitree bridge is separate" in text
