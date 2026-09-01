from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

REQUIRED_PACKAGES = {
    "FAST_LIO",
    "fast_lio_localization",
    "go2_cloud_adapter",
    "go2_cmd_watchdog",
    "go2_localization_guard",
    "go2_map_tools",
    "go2_navigation",
    "go2_pointcloud_mapper",
    "go2_pose_adapter",
    "go2_sdk_bridge",
    "go2_system_bringup",
    "go2_tf_manager",
    "livox_ros_driver2",
}


def test_required_root_entries_exist():
    for path in ("src", "README.md", "docs", "scripts"):
        assert (ROOT / path).exists(), path


def test_runtime_artifacts_are_ignored():
    text = (ROOT / ".gitignore").read_text(encoding="utf-8")
    for rule in (
        "/build/",
        "/devel/",
        "/install/",
        "/maps/",
        "/bags/",
        "/datasets/",
        "*.bag",
        "*.pcd",
    ):
        assert rule in text, rule


def test_no_nested_git_directories_in_active_source():
    src = ROOT / "src"
    if src.exists():
        assert not list(src.rglob(".git"))


def test_no_large_tracked_candidates():
    src = ROOT / "src"
    if not src.exists():
        return
    oversized = [
        path
        for path in src.rglob("*")
        if path.is_file() and path.stat().st_size > 95 * 1024 * 1024
    ]
    assert not oversized, oversized


def test_active_packages_exist():
    src = ROOT / "src"
    present = {path.parent.name for path in src.rglob("package.xml")} if src.exists() else set()
    assert REQUIRED_PACKAGES <= present, sorted(REQUIRED_PACKAGES - present)


def test_livox_driver_infers_ros1_from_ros_environment():
    cmake = (ROOT / "src/livox_ros_driver2/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    assert "$ENV{ROS_VERSION}" in cmake
    assert 'set(ROS_EDITION "ROS1"' in cmake


def test_cloud_adapter_cmake_source_exists():
    assert (
        ROOT / "src/go2_cloud_adapter/src/go2_cloud_adapter.cpp"
    ).is_file()
