from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_ndt_is_the_only_map_to_odom_broadcaster():
    broadcasters = []
    for path in (ROOT / "src").rglob("*"):
        if not path.is_file():
            continue
        text = path.read_text(errors="ignore")
        if "sendTransform" in text and "mapFrame" in text and "odomFrame" in text:
            broadcasters.append(path)
    assert broadcasters
    assert all("fast_lio_localization" in str(path) for path in broadcasters)


def test_localization_uses_go2_public_inputs():
    text = (
        ROOT / "src/go2_system_bringup/launch/go2_relocalization.launch"
    ).read_text(encoding="utf-8")
    for value in (
        "/cloud_registered_base",
        "/odom_nav",
        "/localization",
        "/localization/ok",
        'name="map_frame" value="map"',
        'name="odom_frame" value="odom"',
        'name="base_frame" value="base_link"',
        'name="tf_postdate_sec" value="0.50"',
    ):
        assert value in text


def test_active_localization_launch_has_no_legacy_tf_bridge():
    for name in ("go2_relocalization.launch", "go2_localization.launch"):
        text = (
            ROOT / "src/go2_system_bringup/launch" / name
        ).read_text(encoding="utf-8")
        assert "go2_relocalization_bridge" not in text
        assert "51_navigation_tf.launch" not in text


def test_ndt_exposes_localization_pose_and_quality_metrics():
    text = (
        ROOT / "src/fast_lio_localization/src/fast_lio_localization.cpp"
    ).read_text(encoding="utf-8")
    for value in (
        "localizationTopic",
        "getFitnessScore",
        "getFinalNumIteration",
        "translation_jump",
        "rotation_jump",
    ):
        assert value in text


def test_ndt_owns_and_publishes_provisional_map_to_odom_before_alignment():
    text = (
        ROOT / "src/fast_lio_localization/src/fast_lio_localization.cpp"
    ).read_text(encoding="utf-8")
    assert "ros::Timer _tfPublishTimer" in text
    assert "&Localizer::publishTFTimer" in text
    assert "Publishing provisional identity map -> odom" in text

    publish_tf = text.split("void publishTF()", 1)[1]
    publish_tf = publish_tf.split("\n    }", 1)[0]
    assert "_haveValidAlignment" not in publish_tf
    assert "finiteTransform(_odomMap)" in publish_tf


def test_localization_guard_consumes_ndt_pose():
    text = (
        ROOT / "src/go2_localization_guard/config/guard.yaml"
    ).read_text(encoding="utf-8")
    assert "global_pose_topic: /localization" in text
