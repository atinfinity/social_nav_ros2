#!/usr/bin/env python3
"""デモ用の合成の人(3 人)を TrackedPeople として 20 Hz で流す。実センサ・実追跡は使っていない。

map フレーム。ロボットは原点に静止している前提。
  id 1: y=1.2 を x=-2..2 で往復(0.25 m/s)
  id 2: 原点まわり半径 1.6 m の円(0.25 m/s)
  id 3: (-2, -2) に立ち止まる
usage: demo_people_publisher.py [duration_s(既定 26)]
"""
import math
import sys

import rclpy
from rclpy.node import Node
from social_nav_msgs.msg import TrackedPeople, TrackedPerson

RATE = 20.0
# SocialLayer は max_speed_for_layer(0.3 m/s)を超える人を描かない(速い人は戦術層の担当)。
# 山の追従を見せるため、動く人は 0.25 m/s にする
LINE_Y, LINE_HALF, LINE_SPEED = 1.2, 2.0, 0.25
CIRCLE_R, CIRCLE_SPEED = 1.6, 0.25


def line_state(t):
    """三角波で往復。(x, y, vx, vy)"""
    span = 2.0 * LINE_HALF
    s = (LINE_SPEED * t) % (2.0 * span)
    if s < span:
        return -LINE_HALF + s, LINE_Y, LINE_SPEED, 0.0
    return LINE_HALF - (s - span), LINE_Y, -LINE_SPEED, 0.0


def circle_state(t):
    w = CIRCLE_SPEED / CIRCLE_R
    a = w * t
    return (CIRCLE_R * math.cos(a), CIRCLE_R * math.sin(a),
            -CIRCLE_SPEED * math.sin(a), CIRCLE_SPEED * math.cos(a))


def states(t):
    return [(1,) + line_state(t), (2,) + circle_state(t), (3, -2.0, -2.0, 0.0, 0.0)]


class Demo(Node):
    def __init__(self, duration):
        super().__init__('demo_people_publisher')
        self.pub = self.create_publisher(TrackedPeople, '/social_nav/tracked_people', 10)
        self.t0 = self.get_clock().now()
        self.duration = duration
        self.done = False
        self.create_timer(1.0 / RATE, self.tick)

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        if t > self.duration:
            self.done = True
            return
        msg = TrackedPeople()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        for pid, x, y, vx, vy in states(t):
            p = TrackedPerson()
            p.id = pid
            p.position.x, p.position.y = x, y
            p.velocity.x, p.velocity.y = vx, vy
            p.position_covariance = [0.01, 0.0, 0.0, 0.01]
            p.radius = 0.3
            p.confidence = 1.0
            msg.people.append(p)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Demo(float(sys.argv[1]) if len(sys.argv) > 1 else 26.0)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
