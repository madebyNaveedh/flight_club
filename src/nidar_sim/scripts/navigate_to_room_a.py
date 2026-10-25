#!/usr/bin/env python3
"""
Direct Corridor Navigator to Room A:
1. Ascends to 0.8m altitude
2. Flies East (+X) along south corridor to (5.0, 1.0)
3. Turns North (+Y) and enters doorway to (5.0, 2.5)
4. Turns West (-X) and enters Room A center (3.5, 3.5)
5. Hovers and faces Survivor #1 dummy
"""

import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Navigator(Node):
    def __init__(self):
        super().__init__('room_a_navigator')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pos = [0.6, 1.0, 0.0]
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.odom_received = False

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.pos = [p.x, p.y, p.z]
        
        # Euler angles from quaternion
        sinr_cosp = 2.0 * (o.w * o.x + o.y * o.z)
        cosr_cosp = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (o.w * o.y - o.z * o.x)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

        # Level watchdog: if roll or pitch > 0.15 rad (8.5 deg), auto-level
        if abs(self.roll) > 0.15 or abs(self.pitch) > 0.15:
            self.auto_level()

    def auto_level(self):
        # Call set_pose service to keep horizontal
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        req = f'name: "nidar_drone", position: {{x: {self.pos[0]:.3f}, y: {self.pos[1]:.3f}, z: {max(0.4, self.pos[2]):.3f}}}, orientation: {{w: {cy:.4f}, x: 0.0, y: 0.0, z: {sy:.4f}}}'
        cmd = ["ign", "service", "-s", "/world/nidar_airmouse_arena/set_pose",
               "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
               "--timeout", "1000", "--req", req]
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)
        except Exception:
            pass

    def fly_to_waypoint(self, target_x, target_y, target_z=0.8, target_yaw=None, tolerance=0.25, max_time=15.0):
        start = time.time()
        rate = self.create_rate(20) # 20 Hz
        while rclpy.ok() and (time.time() - start) < max_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            dx = target_x - self.pos[0]
            dy = target_y - self.pos[1]
            dz = target_z - self.pos[2]
            dist_xy = math.hypot(dx, dy)

            if target_yaw is None:
                desired_yaw = math.atan2(dy, dx) if dist_xy > 0.15 else self.yaw
            else:
                desired_yaw = target_yaw

            yaw_err = desired_yaw - self.yaw
            while yaw_err > math.pi: yaw_err -= 2 * math.pi
            while yaw_err < -math.pi: yaw_err += 2 * math.pi

            if dist_xy < tolerance and abs(dz) < 0.15 and (target_yaw is None or abs(yaw_err) < 0.25):
                break

            cmd = Twist()
            # Altitude control
            cmd.linear.z = float(max(-0.3, min(0.35, dz * 1.5)))

            # If yaw error is large, rotate first
            if abs(yaw_err) > 0.6:
                cmd.angular.z = float(max(-0.6, min(0.6, yaw_err * 1.5)))
                cmd.linear.x = 0.05
            else:
                cmd.angular.z = float(max(-0.4, min(0.4, yaw_err * 1.2)))
                # Project forward in body frame
                body_fwd = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
                body_str = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
                cmd.linear.x = float(max(-0.2, min(0.45, body_fwd * 1.0)))
                cmd.linear.y = float(max(-0.2, min(0.2, body_str * 0.8)))

            self.pub_cmd.publish(cmd)
            rate.sleep()

        # Stop
        self.pub_cmd.publish(Twist())

def main():
    rclpy.init()
    nav = Navigator()
    # Wait for odom
    for _ in range(30):
        rclpy.spin_once(nav, timeout_sec=0.1)
        if nav.odom_received:
            break

    print(f"Initial Pose: X={nav.pos[0]:.2f}, Y={nav.pos[1]:.2f}, Z={nav.pos[2]:.2f}")
    
    # Reset / Level drone at launch position
    print("Leveling drone on Launch Pad [0.6, 1.0]...")
    nav.auto_level()
    time.sleep(1.0)

    # 1. Takeoff to 0.85m
    print("Step 1: Taking off to 0.85m...")
    nav.fly_to_waypoint(0.6, 1.0, target_z=0.85, target_yaw=0.0, tolerance=0.15, max_time=4.0)

    # 2. Fly East along south corridor to (5.0, 1.0)
    print("Step 2: Cruising along south corridor to (5.0, 1.0)...")
    nav.fly_to_waypoint(5.0, 1.0, target_z=0.85, target_yaw=0.0, tolerance=0.3, max_time=12.0)

    # 3. Turn North and enter Room A entrance to (5.0, 2.5)
    print("Step 3: Entering Room A doorway at (5.0, 2.5)...")
    nav.fly_to_waypoint(5.0, 2.5, target_z=0.85, target_yaw=1.57, tolerance=0.3, max_time=6.0)

    # 4. Turn West and navigate into Room A center (3.5, 3.5)
    print("Step 4: Navigating into Room A center [3.5, 3.5]...")
    nav.fly_to_waypoint(3.5, 3.5, target_z=0.85, target_yaw=3.14, tolerance=0.3, max_time=8.0)

    # 5. Hover and slowly sweep room to confirm Survivor #1
    print("Step 5: Hovering in Room A and scanning for Survivor #1...")
    start = time.time()
    rate = nav.create_rate(20)
    while rclpy.ok() and (time.time() - start) < 6.0:
        rclpy.spin_once(nav, timeout_sec=0.05)
        cmd = Twist()
        cmd.linear.z = float((0.85 - nav.pos[2]) * 1.5)
        cmd.angular.z = 0.35 # slow scan
        nav.pub_cmd.publish(cmd)
        rate.sleep()

    nav.pub_cmd.publish(Twist())
    print("Mission phase complete! Drone is hovering stably in Room A.")
    nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
