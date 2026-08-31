#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Imu, Image


class StampChecker:
    def __init__(self):
        self.last = {}

        self.lidar_topic = rospy.get_param(
            "~lidar_topic",
            "/livox/lidar")

        self.imu_topic = rospy.get_param(
            "~imu_topic",
            "/livox/imu")

        self.color_topic = rospy.get_param(
            "~color_topic",
            "/camera/color/image_raw")

        self.depth_topic = rospy.get_param(
            "~depth_topic",
            "/camera/depth/image_rect_raw")

        # Livox CustomMsg cannot be imported generically if package absent.
        # Use AnyMsg only to inspect connection age; actual header timestamp
        # is checked separately by rostopic in baseline script.
        rospy.Subscriber(
            self.imu_topic,
            Imu,
            self.imu_cb,
            queue_size=20)

        rospy.Subscriber(
            self.color_topic,
            Image,
            lambda m: self.image_cb("color", m),
            queue_size=5)

        rospy.Subscriber(
            self.depth_topic,
            Image,
            lambda m: self.image_cb("depth", m),
            queue_size=5)

        self.timer = rospy.Timer(
            rospy.Duration(1.0),
            self.report)

    def update(self, name, stamp):
        t = stamp.to_sec()

        prev = self.last.get(name)

        if prev is not None and t < prev:
            rospy.logerr(
                "%s timestamp went backward: %.9f -> %.9f",
                name,
                prev,
                t)

        self.last[name] = t

    def imu_cb(self, msg):
        self.update(
            "imu",
            msg.header.stamp)

    def image_cb(self, name, msg):
        self.update(
            name,
            msg.header.stamp)

    def report(self, _):
        now = rospy.Time.now().to_sec()

        for name, stamp in self.last.items():
            age_ms = (now - stamp) * 1000.0

            rospy.loginfo(
                "%s age = %.1f ms",
                name,
                age_ms)


if __name__ == "__main__":
    rospy.init_node(
        "go2_time_sync_checker")

    StampChecker()

    rospy.spin()
