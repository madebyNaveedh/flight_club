#!/usr/bin/env python3
"""
Scoring and validation node for survivor localization and mapping accuracy.
Evaluates detections against ground truth reference grid.
"""

import os
import json
import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry


class NidarScorerNode(Node):
    def __init__(self):
        super().__init__('nidar_scorer_node')

        self.declare_parameter('reference_json', '')
        ref_path = self.get_parameter('reference_json').get_parameter_value().string_value
        if not ref_path:
            ref_path = os.path.expanduser('~/nidar_airmouse_ws/src/nidar_sim/worlds/reference_grid.json')

        self.ground_truth = None
        if os.path.exists(ref_path):
            with open(ref_path, 'r') as f:
                self.ground_truth = json.load(f)
            self.get_logger().info(f"Loaded reference grid from {ref_path}")
        else:
            self.get_logger().warn(f"Reference file not found at {ref_path}")

        self.drone_pose = [0.6, 1.0]
        self.survivors_tagged = []
        self.confirmed_survivors_count = 0
        self.grid_explored_cells = set()
        self.total_corridor_cells = 45
        self.has_exited = False

        self.sub_survivors = self.create_subscription(String, '/nidar/survivors_data', self.survivors_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pub_score = self.create_publisher(String, '/nidar/mission_score', 10)

        self.timer = self.create_timer(1.0, self.score_evaluation_loop)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_pose = [p.x, p.y]
        gx, gy = int(math.floor(p.x)), int(math.floor(p.y))
        if 0 <= gx < 15 and 0 <= gy < 15:
            self.grid_explored_cells.add((gx, gy))

        if p.x < 0.2 and abs(p.y - 1.0) < 0.6 and len(self.grid_explored_cells) > 10:
            self.has_exited = True

    def survivors_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.survivors_tagged = data.get("survivors", [])
            self.confirmed_survivors_count = sum(1 for s in self.survivors_tagged if s.get("confirmed", False))
        except Exception:
            pass

    def map_callback(self, msg: OccupancyGrid):
        data = np.array(msg.data)
        explored = np.count_nonzero(data >= 0)
        self.total_corridor_cells = max(self.total_corridor_cells, explored)

    def score_evaluation_loop(self):
        survivor_score = 0
        correct_tags = 0
        gt_survivors = self.ground_truth.get("survivors", []) if self.ground_truth else []

        for s in self.survivors_tagged:
            if not s.get("confirmed", False):
                continue
            gx, gy = s.get("grid_coordinate", [-1, -1])
            is_match = False
            for gt in gt_survivors:
                gt_gx, gt_gy = gt["grid"]
                if abs(gx - gt_gx) <= 1 and abs(gy - gt_gy) <= 1:
                    is_match = True
                    break
            if is_match:
                correct_tags += 1
                survivor_score += 40

        survivor_score = min(240, survivor_score)

        exploration_ratio = min(1.0, len(self.grid_explored_cells) / max(30.0, float(self.total_corridor_cells)))
        mapping_score = int(exploration_ratio * 220)

        exit_score = 50 if self.has_exited else 0
        gcs_score = 50
        bonus_score = 40 if (correct_tags >= 6 and self.has_exited) else 15

        total_score = survivor_score + mapping_score + exit_score + gcs_score + bonus_score
        total_score = min(600, total_score)

        report = {
            "total_score": total_score,
            "max_score": 600,
            "breakdown": {
                "survivor_localisation_pts": survivor_score,
                "grid_mapping_pts": mapping_score,
                "autonomous_exit_pts": exit_score,
                "gcs_telemetry_pts": gcs_score,
                "efficiency_bonus_pts": bonus_score
            },
            "stats": {
                "confirmed_survivors": self.confirmed_survivors_count,
                "correct_grid_matches": correct_tags,
                "explored_cells": len(self.grid_explored_cells),
                "has_exited": self.has_exited
            }
        }

        msg = String()
        msg.data = json.dumps(report)
        self.pub_score.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NidarScorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
