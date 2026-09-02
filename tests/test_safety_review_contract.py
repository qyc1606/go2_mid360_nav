import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_watchdog_rejects_nonfinite_twist_and_times_out_localization():
    text = (
        ROOT / "src/go2_cmd_watchdog/src/cmd_watchdog.cpp"
    ).read_text(encoding="utf-8")
    assert "finiteTwist" in text
    assert "std::isfinite" in text
    assert "localization_timeout_sec" in text
    assert "last_localization_wall_time" in text


def test_real_bridge_times_out_localization_health():
    text = (
        ROOT / "src/go2_sdk_bridge/src/sdk_bridge_real.cpp"
    ).read_text(encoding="utf-8")
    assert "localization_timeout_sec" in text
    assert "last_localization_wall_stamp" in text
    assert "localizationFresh" in text


def test_ndt_rejects_bad_alignment_and_exposes_success_age():
    text = (
        ROOT / "src/fast_lio_localization/src/fast_lio_localization.cpp"
    ).read_text(encoding="utf-8")
    for value in (
        "hasConverged",
        "allFinite",
        "max_fitness_score",
        "/localization/last_success_age",
        "publishLocalization",
        "finitePose",
        "finiteTransform",
        "max_alignment_age_sec",
    ):
        assert value in text


def test_guard_consumes_ndt_quality_and_heartbeats_state():
    text = (
        ROOT / "src/go2_localization_guard/src/localization_guard.cpp"
    ).read_text(encoding="utf-8")
    for value in (
        "/localization/ndt_score",
        "/localization/ndt_iterations",
        "/localization/translation_jump",
        "/localization/rotation_jump",
        "/localization/last_success_age",
        "publishCurrentState",
        "finitePose",
    ):
        assert value in text


def test_command_monitor_checks_all_components_and_requires_evidence():
    monitor = load_script("check_cmd_vel.py")
    assert "non-finite" in monitor.validate_command(
        0.0, 0.0, 0.0, vz=float("nan")
    )
    assert "non-finite" in monitor.validate_command(
        0.0, 0.0, 0.0, wx=float("inf")
    )
    assert "non-finite" in monitor.validate_command(
        0.0, 0.0, 0.0, wy=float("-inf")
    )
    text = (ROOT / "scripts/check_cmd_vel.py").read_text(encoding="utf-8")
    assert "message_count == 0" in text


def test_validator_checks_expanded_packages_and_forbids_real_sdk():
    validator = load_script("validate_launch_contracts.py")
    assert "ego_planner" in validator.FORBIDDEN_PACKAGES
    assert "go2_sdk_bridge" in validator.FORBIDDEN_PACKAGES
    text = (
        ROOT / "scripts/validate_launch_contracts.py"
    ).read_text(encoding="utf-8")
    assert "roslaunch.config.load_config_default" in text


def test_legacy_navigation_launch_cannot_bypass_watchdog():
    text = (
        ROOT / "src/go2_navigation/launch/navigation.launch"
    ).read_text(encoding="utf-8")
    assert "scout" not in text.lower()
    assert 'to="/cmd_vel"' not in text
    assert "go2_navigation.launch" in text
