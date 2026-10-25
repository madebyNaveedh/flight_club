#!/usr/bin/env python3
"""
Autonomous maze exploration and survivor search node.
Uses 2D LiDAR for corridor centering, room sweeps, and returns to exit.
"""

import math
import json
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

STATE_IDLE = "IDLE"
STATE_TAKEOFF = "TAKEOFF"
STATE_CRUISING = "CRUISING"
STATE_ROOM_SCAN = "ROOM_SCAN"
STATE_RETURN_EXIT = "RETURN_EXIT"
STATE_LANDING = "LANDING"
STATE_COMPLETED = "COMPLETED"


class AutonomousExplorerNode(Node):
    def __init__(self):
        super().__init__('autonomous_explorer_node')

        self.target_alt = 0.8
        self.cruise_speed = 0.35
        self.min_wall_dist = 0.45
        self.exit_coord = (0.6, 1.0)
        
        self.state = STATE_TAKEOFF
        self.drone_pos = [0.6, 1.0, 0.2]
        self.drone_yaw = 0.0
        self.confirmed_survivors = 0
        self.total_target_survivors = 6

        self.ranges = {
            'front': 10.0,
            'front_left': 10.0,
            'front_right': 10.0,
            'left': 10.0,
            'right': 10.0
        }

        self.room_scan_yaw_accum = 0.0
        self.last_scan_yaw = 0.0
        self.last_room_scan_time = 0.0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_mission_state = self.create_publisher(String, '/nidar/mission_state', 10)

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_survivors = self.create_subscription(String, '/nidar/survivors_data', self.survivors_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.drone_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.drone_pos = [p.x, p.y, p.z]

    def survivors_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            survivors = data.get("survivors", [])
            self.confirmed_survivors = sum(1 for s in survivors if s.get("confirmed", False))
            if self.confirmed_survivors >= self.total_target_survivors and self.state == STATE_CRUISING:
                self.state = STATE_RETURN_EXIT
        except Exception:
            pass

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max, neginf=msg.range_min)

        n = len(ranges)
        idx_0 = n // 2
        deg = int(n / 360.0)

        self.ranges['front'] = float(np.min(ranges[idx_0 - 20*deg : idx_0 + 20*deg]))
        self.ranges['front_left'] = float(np.min(ranges[idx_0 + 20*deg : idx_0 + 65*deg]))
        self.ranges['front_right'] = float(np.min(ranges[idx_0 - 65*deg : idx_0 - 20*deg]))
        self.ranges['left'] = float(np.min(ranges[idx_0 + 65*deg : idx_0 + 115*deg]))
        self.ranges['right'] = float(np.min(ranges[idx_0 - 115*deg : idx_0 - 65*deg]))

    def control_loop(self):
        cmd = Twist()
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if self.state == STATE_TAKEOFF:
            if self.drone_pos[2] < (self.target_alt - 0.05):
                cmd.linear.z = 0.35
            else:
                cmd.linear.z = 0.0
                self.state = STATE_CRUISING

        elif self.state == STATE_CRUISING:
            alt_error = self.target_alt - self.drone_pos[2]
            cmd.linear.z = float(np.clip(alt_error * 1.5, -0.3, 0.3))

            is_room = (self.ranges['left'] > 1.8 and self.ranges['right'] > 1.8) or \
                      (self.ranges['left'] > 2.2) or (self.ranges['right'] > 2.2)
            
            if is_room and (now_sec - self.last_room_scan_time > 20.0):
                self.state = STATE_ROOM_SCAN
                self.room_scan_yaw_accum = 0.0
                self.last_scan_yaw = self.drone_yaw
                self.last_room_scan_time = now_sec
                return

            front = self.ranges['front']
            fl = self.ranges['front_left']
            fr = self.ranges['front_right']
            left = self.ranges['left']
            right = self.ranges['right']

            if front < 0.75 or fl < 0.50 or fr < 0.50:
                cmd.linear.x = 0.05
                cmd.angular.z = 0.55 if left > right else -0.55
            else:
                cmd.linear.x = self.cruise_speed
                centering_error = left - right
                if left < self.min_wall_dist:
                    cmd.angular.z = -0.35
                elif right < self.min_wall_dist:
                    cmd.angular.z = 0.35
                else:
                    cmd.angular.z = float(np.clip(centering_error * 0.25, -0.4, 0.4))

        elif self.state == STATE_ROOM_SCAN:
            alt_error = self.target_alt - self.drone_pos[2]
            cmd.linear.z = float(np.clip(alt_error * 1.5, -0.2, 0.2))
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.45

            yaw_diff = self.drone_yaw - self.last_scan_yaw
            while yaw_diff > math.pi: yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi: yaw_diff += 2 * math.pi
            self.room_scan_yaw_accum += abs(yaw_diff)
            self.last_scan_yaw = self.drone_yaw

            if self.room_scan_yaw_accum >= (2 * math.pi * 0.95):
                self.state = STATE_CRUISING

        elif self.state == STATE_RETURN_EXIT:
            dx = self.exit_coord[0] - self.drone_pos[0]
            dy = self.exit_coord[1] - self.drone_pos[1]
            dist_to_exit = math.hypot(dx, dy)

            if dist_to_exit < 0.5:
                self.state = STATE_LANDING
            else:
                target_heading = math.atan2(dy, dx)
                heading_err = target_heading - self.drone_yaw
                while heading_err > math.pi: heading_err -= 2 * math.pi
                while heading_err < -math.pi: heading_err += 2 * math.pi

                cmd.linear.z = float(np.clip((self.target_alt - self.drone_pos[2]) * 1.5, -0.2, 0.2))

                if self.ranges['front'] < 0.65:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5 if self.ranges['left'] > self.ranges['right'] else -0.5
                else:
                    cmd.linear.x = min(self.cruise_speed, dist_to_exit * 0.4)
                    cmd.angular.z = float(np.clip(heading_err * 0.8, -0.5, 0.5))

        elif self.state == STATE_LANDING:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            if self.drone_pos[2] > 0.12:
                cmd.linear.z = -0.20
            else:
                cmd.linear.z = 0.0
                self.state = STATE_COMPLETED

        elif self.state == STATE_COMPLETED:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)

        msg = String()
        msg.data = json.dumps({
            "state": self.state,
            "altitude": round(self.drone_pos[2], 2),
            "confirmed_survivors": self.confirmed_survivors,
            "front_clearance": round(self.ranges['front'], 2)
        })
        self.pub_mission_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.pub_cmd.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
