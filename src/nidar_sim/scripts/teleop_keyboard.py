#!/usr/bin/env python3
"""
Keyboard teleoperation node for drone control and mission triggers.
"""

import sys
import termios
import tty
import select
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

HELP_MSG = """
NIDAR Teleop Controller
---------------------------
Movement:
        w
   a    s    d      (Forward / Back / Left / Right)

Altitude & Heading:
        i
   j    k    l      (Ascend / Descend / Yaw)

Missions:
   [1] : Fly to Room A
   [0] : Return to Launch Pad

SPACE : Hover
CTRL-C: Quit
---------------------------
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('nidar_teleop')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pos = [0.6, 1.0, 0.2]
        self.yaw = 0.0

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.pos = [p.x, p.y, p.z]
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def run_corridor_to_room_a(self):
        print("\n[Auto] Navigating to Room A...")
        waypoints = [
            (0.6, 1.0, 0.85, 0.0),
            (4.8, 1.0, 0.85, 0.0),
            (4.8, 2.5, 0.85, 1.57),
            (3.5, 3.5, 0.85, 3.14)
        ]
        for wp in waypoints:
            self.fly_to(wp[0], wp[1], wp[2], wp[3])
        print("[Auto] Reached Room A. Scanning room...")
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 6.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            cmd = Twist()
            cmd.angular.z = 0.35
            self.pub_cmd.publish(cmd)
            time.sleep(0.05)
        self.pub_cmd.publish(Twist())
        print("[Auto] Sweep complete. Hovering.")

    def run_return_to_pad(self):
        print("\n[Auto] Returning to Launch Pad...")
        waypoints = [
            (4.8, 2.5, 0.85, -1.57),
            (4.8, 1.0, 0.85, 3.14),
            (0.6, 1.0, 0.85, 3.14)
        ]
        for wp in waypoints:
            self.fly_to(wp[0], wp[1], wp[2], wp[3])
        self.pub_cmd.publish(Twist())
        print("[Auto] Back at Launch Pad. Hovering.")

    def fly_to(self, tx, ty, tz, tyaw, tol=0.3, max_sec=12.0):
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < max_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            dx = tx - self.pos[0]
            dy = ty - self.pos[1]
            dz = tz - self.pos[2]
            dist = math.hypot(dx, dy)

            yaw_err = tyaw - self.yaw
            while yaw_err > math.pi: yaw_err -= 2 * math.pi
            while yaw_err < -math.pi: yaw_err += 2 * math.pi

            if dist < tol and abs(dz) < 0.2:
                break

            cmd = Twist()
            cmd.linear.z = float(max(-0.25, min(0.35, dz * 1.5)))

            if abs(yaw_err) > 0.6:
                cmd.angular.z = float(max(-0.5, min(0.5, yaw_err * 1.5)))
                cmd.linear.x = 0.05
            else:
                cmd.angular.z = float(max(-0.35, min(0.35, yaw_err * 1.2)))
                fwd = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
                strf = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
                cmd.linear.x = float(max(-0.2, min(0.45, fwd * 1.2)))
                cmd.linear.y = float(max(-0.2, min(0.2, strf * 0.8)))

            self.pub_cmd.publish(cmd)
            time.sleep(0.05)


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()

    speed_xy = 0.6
    speed_z = 0.4
    speed_yaw = 0.8

    print(HELP_MSG)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = getKey(settings)
            if not key:
                continue

            twist = Twist()
            if key == 'w':
                twist.linear.x = speed_xy
            elif key == 's':
                twist.linear.x = -speed_xy
            elif key == 'a':
                twist.linear.y = speed_xy
            elif key == 'd':
                twist.linear.y = -speed_xy
            elif key == 'i':
                twist.linear.z = speed_z
            elif key == 'k':
                twist.linear.z = -speed_z
            elif key == 'j':
                twist.angular.z = speed_yaw
            elif key == 'l':
                twist.angular.z = -speed_yaw
            elif key == '1':
                node.run_corridor_to_room_a()
                print(HELP_MSG)
                continue
            elif key == '0':
                node.run_return_to_pad()
                print(HELP_MSG)
                continue
            elif key == ' ':
                twist = Twist()
            elif key == '\x03':
                break

            t_end = time.time() + 0.2
            while time.time() < t_end:
                node.pub_cmd.publish(twist)
                time.sleep(0.04)

    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        node.pub_cmd.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
