#!/usr/bin/env python3
"""
Survivor perception node.
Combines YOLOv8 with color segmentation for rescue dummy detection
and projects 2D image coordinates to 1m x 1m grid map cells.
"""

import math
import json
import time
import datetime
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

import torch
from ultralytics import YOLO


def cv2_to_imgmsg(cv_img, encoding='bgr8'):
    msg = Image()
    msg.height, msg.width = cv_img.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = msg.width * 3
    msg.data = cv_img.tobytes()
    return msg


def imgmsg_to_cv2(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
    if msg.encoding == 'rgb8':
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return arr


class SurvivorCandidate:
    def __init__(self, survivor_id, wx, wy, gx, gy, confidence):
        self.id = survivor_id
        self.wx = wx
        self.wy = wy
        self.gx = gx
        self.gy = gy
        self.detection_count = 1
        self.avg_conf = confidence
        self.confirmed = False

    def update(self, wx, wy, gx, gy, conf):
        self.detection_count += 1
        self.wx = 0.8 * self.wx + 0.2 * wx
        self.wy = 0.8 * self.wy + 0.2 * wy
        self.gx = int(round(self.wx))
        self.gy = int(round(self.wy))
        self.avg_conf = (self.avg_conf * (self.detection_count - 1) + conf) / self.detection_count
        if self.detection_count >= 3:
            self.confirmed = True


class SurvivorDetectorNode(Node):
    def __init__(self):
        super().__init__('survivor_detector_node')
        
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('max_survivors', 6)
        self.declare_parameter('arena_size', 15.0)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.max_survivors = self.get_parameter('max_survivors').get_parameter_value().integer_value
        self.arena_size = self.get_parameter('arena_size').get_parameter_value().double_value

        self.latest_depth = None
        self.drone_pose = {'x': 0.6, 'y': 1.0, 'z': 0.8, 'yaw': 0.0}
        
        self.fx = 380.0
        self.fy = 380.0
        self.cx = 320.0
        self.cy = 240.0

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Loading YOLOv8 on {self.device}")
        self.model = YOLO('yolov8n.pt')

        self.survivors = []
        self.next_survivor_id = 1
        self.frame_count = 0
        self.fps_timer = time.time()
        self.fps = 30.0

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_img = self.create_subscription(Image, self.image_topic, self.image_callback, qos_sensor)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_sensor)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_sensor)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)

        self.pub_annotated = self.create_publisher(Image, '/nidar/annotated_feed', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/nidar/survivor_markers', 10)
        self.pub_json = self.create_publisher(String, '/nidar/survivors_data', 10)

    def info_callback(self, msg: CameraInfo):
        if msg.k[0] > 0:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.drone_pose['x'] = p.x
        self.drone_pose['y'] = p.y
        self.drone_pose['z'] = p.z
        self.drone_pose['yaw'] = yaw

    def depth_callback(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            self.latest_depth = arr
        except Exception:
            pass

    def detect_color_dummy(self, cv_img):
        # Color segmentation fallback for mannequins / rescue dummies
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array([0, 85, 60]), np.array([12, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([168, 85, 60]), np.array([180, 255, 255]))
        mask_red = m1 | m2
        mask_blue = cv2.inRange(hsv, np.array([98, 85, 60]), np.array([135, 255, 255]))
        combined = mask_red | mask_blue

        kernel = np.ones((5, 5), np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        cnts, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 350:
                x, y, w, h = cv2.boundingRect(c)
                # Ignore HUD banner at top
                if y < 38 and (y + h) < 45:
                    continue
                boxes.append([x, y, x + w, y + h, 0.88])
        return boxes

    def image_callback(self, msg: Image):
        try:
            cv_img = imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")
            return

        self.frame_count += 1
        now = time.time()
        if (now - self.fps_timer) >= 1.0:
            self.fps = self.frame_count / (now - self.fps_timer)
            self.frame_count = 0
            self.fps_timer = now

        # 1. Primary YOLOv8 inference
        results = self.model.predict(
            source=cv_img,
            conf=self.conf_threshold,
            device=self.device,
            verbose=False,
            classes=[0] # person
        )

        detected_raw = []
        if len(results) > 0 and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                detected_raw.append([int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]), conf])

        # 2. Secondary rescue dummy detector fallback
        if not detected_raw:
            detected_raw = self.detect_color_dummy(cv_img)

        detected_this_frame = []
        for item in detected_raw:
            x1, y1, x2, y2, conf = item
            u_center = int((x1 + x2) / 2.0)
            v_center = int((y1 + y2) / 2.0)

            dist = None
            if self.latest_depth is not None and 0 <= v_center < self.latest_depth.shape[0] and 0 <= u_center < self.latest_depth.shape[1]:
                d_val = float(self.latest_depth[v_center, u_center])
                if not math.isnan(d_val) and d_val > 0.1:
                    dist = d_val

            if dist is None or dist <= 0.1:
                altitude = max(0.4, self.drone_pose['z'])
                dist = altitude / max(0.3, math.cos(0.45))

            xc = ((u_center - self.cx) / self.fx) * dist
            yc = ((v_center - self.cy) / self.fy) * dist
            zc = dist

            xb = zc
            yb = -xc

            yaw = self.drone_pose['yaw']
            world_x = self.drone_pose['x'] + (xb * math.cos(yaw) - yb * math.sin(yaw))
            world_y = self.drone_pose['y'] + (xb * math.sin(yaw) + yb * math.cos(yaw))

            world_x = max(0.1, min(self.arena_size - 0.1, world_x))
            world_y = max(0.1, min(self.arena_size - 0.1, world_y))

            grid_x = int(math.floor(world_x))
            grid_y = int(math.floor(world_y))

            survivor_match = self.match_or_add_survivor(world_x, world_y, grid_x, grid_y, conf)
            detected_this_frame.append({
                'id': survivor_match.id,
                'box': [x1, y1, x2, y2],
                'grid': [grid_x, grid_y],
                'conf': conf,
                'confirmed': survivor_match.confirmed
            })

        # Draw HUD & bounding boxes
        annotated_img = cv_img.copy()
        h, w = annotated_img.shape[:2]

        # Top telemetry bar
        cv2.rectangle(annotated_img, (0, 0), (w, 32), (18, 22, 30), -1)
        dot_color = (0, 255, 65) if int(now * 2) % 2 == 0 else (0, 180, 50)
        cv2.circle(annotated_img, (16, 16), 5, dot_color, -1)
        time_str = datetime.datetime.now().strftime("%H:%M:%S")
        cv2.putText(annotated_img, f"LIVE {time_str} | {self.fps:.0f} FPS", (28, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 65), 1, cv2.LINE_AA)

        hdg = int((math.degrees(self.drone_pose['yaw']) + 360) % 360)
        telem_str = f"ALT: {self.drone_pose['z']:.2f}m | HDG: {hdg:03d} DEG | GPS-DENIED"
        cv2.putText(annotated_img, telem_str, (w // 2 - 120, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 229, 255), 1, cv2.LINE_AA)

        confirmed_count = sum(1 for s in self.survivors if s.confirmed)
        surv_str = f"SURVIVORS: {confirmed_count}/{self.max_survivors}"
        cv2.putText(annotated_img, surv_str, (w - 145, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 179, 0), 1, cv2.LINE_AA)

        # Reticle
        cx, cy = w // 2, h // 2
        reticle_color = (0, 229, 255)
        cv2.line(annotated_img, (cx - 16, cy), (cx - 4, cy), reticle_color, 1)
        cv2.line(annotated_img, (cx + 4, cy), (cx + 16, cy), reticle_color, 1)
        cv2.line(annotated_img, (cx, cy - 16), (cx, cy - 4), reticle_color, 1)
        cv2.line(annotated_img, (cx, cy + 4), (cx, cy + 16), reticle_color, 1)

        # Bounding boxes
        for det in detected_this_frame:
            b = det['box']
            color = (0, 255, 65) if det['confirmed'] else (0, 165, 255)
            cv2.rectangle(annotated_img, (b[0], b[1]), (b[2], b[3]), color, 2)
            label = f"SURVIVOR #{det['id']} [{det['grid'][0]},{det['grid'][1]}] {det['conf']:.0%}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            cv2.rectangle(annotated_img, (b[0], max(34, b[1] - 20)), (b[0] + tw + 6, max(34, b[1])), color, -1)
            cv2.putText(annotated_img, label, (b[0] + 3, max(34, b[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1, cv2.LINE_AA)

        try:
            self.pub_annotated.publish(cv2_to_imgmsg(annotated_img, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")

        self.publish_markers()
        self.publish_json()

    def match_or_add_survivor(self, wx, wy, gx, gy, conf):
        for s in self.survivors:
            if math.hypot(s.wx - wx, s.wy - wy) < 1.2:
                s.update(wx, wy, gx, gy, conf)
                return s

        if len(self.survivors) < self.max_survivors:
            new_s = SurvivorCandidate(self.next_survivor_id, wx, wy, gx, gy, conf)
            self.survivors.append(new_s)
            self.next_survivor_id += 1
            self.get_logger().info(f"Target Acquired: Survivor #{new_s.id} at ({wx:.2f}, {wy:.2f}) -> Grid [{gx}, {gy}]")
            return new_s
        else:
            closest = min(self.survivors, key=lambda s: math.hypot(s.wx - wx, s.wy - wy))
            closest.update(wx, wy, gx, gy, conf)
            return closest

    def publish_markers(self):
        m_array = MarkerArray()
        for s in self.survivors:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "survivors"
            m.id = s.id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(s.wx)
            m.pose.position.y = float(s.wy)
            m.pose.position.z = 0.5
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 1.0
            m.color.a = 0.9
            m.color.r = 0.0 if s.confirmed else 1.0
            m.color.g = 1.0 if s.confirmed else 0.6
            m.color.b = 0.0
            m_array.markers.append(m)
        self.pub_markers.publish(m_array)

    def publish_json(self):
        data = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "drone_pose": self.drone_pose,
            "total_detected": len(self.survivors),
            "survivors": [
                {
                    "id": s.id,
                    "grid_coordinate": [s.gx, s.gy],
                    "world_position": [round(s.wx, 2), round(s.wy, 2)],
                    "confidence": round(s.avg_conf, 3),
                    "detections": s.detection_count,
                    "confirmed": s.confirmed
                }
                for s in self.survivors
            ]
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pub_json.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SurvivorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
