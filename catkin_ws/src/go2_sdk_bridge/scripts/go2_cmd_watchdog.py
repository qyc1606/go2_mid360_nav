#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Go2CmdWatchdog:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/cmd_vel_nav"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/cmd_vel_safe"
        )

        self.localization_topic = rospy.get_param(
            "~localization_ok_topic",
            "/localization/ok"
        )

        self.timeout = rospy.get_param(
            "~timeout_sec",
            0.25
        )

        self.max_vx = rospy.get_param(
            "~max_vx",
            0.30
        )

        self.max_vy = rospy.get_param(
            "~max_vy",
            0.0
        )

        self.max_wz = rospy.get_param(
            "~max_wz",
            0.15
        )

        self.localization_ok = False
        self.have_cmd = False

        self.last_cmd = Twist()
        self.last_rx = rospy.get_time()

        self.pub = rospy.Publisher(
            self.output_topic,
            Twist,
            queue_size=10
        )

        rospy.Subscriber(
            self.input_topic,
            Twist,
            self.cmd_callback,
            queue_size=10
        )

        rospy.Subscriber(
            self.localization_topic,
            Bool,
            self.localization_callback,
            queue_size=10
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.02),
            self.timer_callback
        )

        rospy.logwarn(
            "GO2 cmd watchdog started: %s -> %s",
            self.input_topic,
            self.output_topic
        )


    @staticmethod
    def clamp(x, limit):
        if limit <= 0.0:
            return 0.0

        return max(
            -limit,
            min(limit, x)
        )


    @staticmethod
    def valid(cmd):
        return all(
            math.isfinite(v)
            for v in [
                cmd.linear.x,
                cmd.linear.y,
                cmd.angular.z
            ]
        )


    def localization_callback(self, msg):
        self.localization_ok = msg.data


    def cmd_callback(self, msg):
        if not self.valid(msg):
            rospy.logerr_throttle(
                1.0,
                "Rejected invalid cmd_vel_nav"
            )

            return

        self.last_cmd = msg
        self.last_rx = rospy.get_time()
        self.have_cmd = True


    def publish_zero(self):
        self.pub.publish(Twist())


    def timer_callback(self, _event):
        if not self.localization_ok:
            self.publish_zero()
            return

        if not self.have_cmd:
            self.publish_zero()
            return

        age = rospy.get_time() - self.last_rx

        if age > self.timeout:
            self.publish_zero()

            rospy.logwarn_throttle(
                1.0,
                "cmd_vel_nav timeout %.3f s",
                age
            )

            return

        out = Twist()

        out.linear.x = self.clamp(
            self.last_cmd.linear.x,
            self.max_vx
        )

        # Disabled during first real closed-loop stage.
        out.linear.y = self.clamp(
            self.last_cmd.linear.y,
            self.max_vy
        )

        out.angular.z = self.clamp(
            self.last_cmd.angular.z,
            self.max_wz
        )

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node(
        "go2_cmd_watchdog"
    )

    Go2CmdWatchdog()

    rospy.spin()
