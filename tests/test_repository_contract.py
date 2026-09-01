from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


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
