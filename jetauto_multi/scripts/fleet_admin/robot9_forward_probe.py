#!/usr/bin/env python
from __future__ import print_function

import argparse
import time

import rospy
from geometry_msgs.msg import Twist


TOPIC = "/robot_9/jetauto_controller/cmd_vel"


def publish_stop(publisher):
    message = Twist()
    for _ in range(20):
        try:
            publisher.publish(message)
        except Exception:
            pass
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description="Robot9 forward/stop probe")
    parser.add_argument("action", choices=("forward", "stop"))
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args(rospy.myargv()[1:])

    if not 0.0 < args.speed <= 0.10:
        parser.error("--speed must be in (0, 0.10] m/s")
    if not 0.0 < args.duration <= 10.0:
        parser.error("--duration must be in (0, 10] seconds")

    rospy.init_node("robot9_forward_probe", anonymous=True)
    publisher = rospy.Publisher(TOPIC, Twist, queue_size=1)

    deadline = time.time() + 3.0
    while publisher.get_num_connections() == 0 and time.time() < deadline:
        rospy.sleep(0.05)
    if publisher.get_num_connections() == 0:
        raise RuntimeError("no subscriber on %s; Robot9 bottom layer is not ready" % TOPIC)

    try:
        publish_stop(publisher)
        if args.action == "stop":
            print("Robot9 stop command sent")
            return

        command = Twist()
        command.linear.x = args.speed
        end_time = time.time() + args.duration
        rate = rospy.Rate(20)
        print("Robot9 forward: %.3f m/s for %.1f s" % (args.speed, args.duration))
        while not rospy.is_shutdown() and time.time() < end_time:
            publisher.publish(command)
            rate.sleep()
    finally:
        publish_stop(publisher)
        print("Robot9 final stop command sent")


if __name__ == "__main__":
    main()
