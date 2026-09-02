#!/usr/bin/env python3
"""Read-only safety monitor for navigation velocity commands."""

import argparse
import math
import time


def validate_command(vx, vy, wz, max_vx=0.20, max_wz=0.30):
    values = (vx, vy, wz)
    if not all(math.isfinite(value) for value in values):
        return "non-finite command"
    if abs(vy) > 1e-6:
        return f"lateral velocity is disabled: vy={vy}"
    if abs(vx) > max_vx + 1e-9:
        return f"max_vx exceeded: vx={vx}, limit={max_vx}"
    if abs(wz) > max_wz + 1e-9:
        return f"max_wz exceeded: wz={wz}, limit={max_wz}"
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/cmd_vel_nav")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--max-vx", type=float, default=0.20)
    parser.add_argument("--max-wz", type=float, default=0.30)
    args = parser.parse_args()

    import rospy
    from geometry_msgs.msg import Twist

    errors = []
    message_count = 0

    def callback(message):
        nonlocal message_count
        message_count += 1
        error = validate_command(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            args.max_vx,
            args.max_wz,
        )
        if error:
            errors.append(error)
            rospy.logerr(error)

    rospy.init_node("go2_cmd_vel_monitor", anonymous=True)
    rospy.Subscriber(args.topic, Twist, callback, queue_size=20)
    deadline = time.monotonic() + args.duration
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline and not errors:
        rate.sleep()

    if errors:
        print(f"FAIL {errors[0]}")
        return 1
    print(f"PASS checked {message_count} command(s) on {args.topic}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
