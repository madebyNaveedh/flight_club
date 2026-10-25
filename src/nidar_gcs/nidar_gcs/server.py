#!/usr/bin/env python3
"""
Ground Control Station web server and ROS telemetry bridge.
"""

import os
import math
import json
import time
import threading
from socketserver import ThreadingMixIn
from http.server import HTTPServer, BaseHTTPRequestHandler
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String

GCS_STATE = {
    "drone_pose": {"x": 0.6, "y": 1.0, "z": 0.8, "yaw": 0.0},
    "survivors": [],
    "score_report": None,
    "mission_state": None,
    "latest_jpeg": None,
    "last_frame_time": 0.0
}
LOCK = threading.Lock()


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class GCSRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        clean_path = self.path.split('?')[0]

        if clean_path in ('/', '/index.html'):
            candidate_paths = [
                os.path.join(os.path.dirname(__file__), 'web', 'index.html'),
                '/home/mohammed/nidar_airmouse_ws/src/nidar_gcs/nidar_gcs/web/index.html',
                '/home/mohammed/nidar_airmouse_ws/install/nidar_gcs/share/nidar_gcs/web/index.html'
            ]
            html_path = next((p for p in candidate_paths if os.path.exists(p)), None)
            if html_path:
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                with open(html_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()

        elif clean_path == '/api/state':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            with LOCK:
                data = {
                    "drone_pose": GCS_STATE["drone_pose"],
                    "survivors": GCS_STATE["survivors"],
                    "score_report": GCS_STATE["score_report"],
                    "mission_state": GCS_STATE.get("mission_state")
                }
            self.wfile.write(json.dumps(data).encode('utf-8'))

        elif clean_path in ('/camera_frame', '/camera_frame.jpg'):
            with LOCK:
                frame = GCS_STATE["latest_jpeg"]
            if frame is not None:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame)))
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.end_headers()
                self.wfile.write(frame)
            else:
                self.send_response(503)
                self.end_headers()

        elif clean_path == '/video_feed':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with LOCK:
                        frame = GCS_STATE["latest_jpeg"]
                    if frame is not None:
                        header = (
                            b'--FRAME\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Content-Length: ' + str(len(frame)).encode('utf-8') + b'\r\n\r\n'
                        )
                        self.wfile.write(header + frame + b'\r\n')
                    else:
                        dummy = np.zeros((360, 480, 3), dtype=np.uint8)
                        cv2.putText(dummy, "WAITING FOR FEED...", (70, 180),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 1)
                        _, buf = cv2.imencode('.jpg', dummy)
                        f_bytes = buf.tobytes()
                        header = (
                            b'--FRAME\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Content-Length: ' + str(len(f_bytes)).encode('utf-8') + b'\r\n\r\n'
                        )
                        self.wfile.write(header + f_bytes + b'\r\n')
                    time.sleep(0.04) # ~25 FPS
            except Exception:
                pass
        else:
            self.send_response(404)
            self.end_headers()


class GCSRosNode(Node):
    def __init__(self):
        super().__init__('nidar_gcs_server_node')

        self.sub_feed = self.create_subscription(Image, '/nidar/annotated_feed', self.feed_callback, 10)
        self.sub_raw = self.create_subscription(Image, '/camera/image_raw', self.raw_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_survivors = self.create_subscription(String, '/nidar/survivors_data', self.survivors_callback, 10)
        self.sub_score = self.create_subscription(String, '/nidar/mission_score', self.score_callback, 10)
        self.sub_mission_state = self.create_subscription(String, '/nidar/mission_state', self.state_callback, 10)

    def state_callback(self, msg: String):
        try:
            with LOCK:
                GCS_STATE["mission_state"] = json.loads(msg.data)
        except Exception:
            pass

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with LOCK:
            GCS_STATE["drone_pose"] = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw}

    def process_and_store_image(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            _, buffer = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with LOCK:
                GCS_STATE["latest_jpeg"] = buffer.tobytes()
                GCS_STATE["last_frame_time"] = time.time()
        except Exception:
            pass

    def feed_callback(self, msg: Image):
        self.process_and_store_image(msg)

    def raw_callback(self, msg: Image):
        with LOCK:
            elapsed = time.time() - GCS_STATE["last_frame_time"]
        if elapsed > 0.5:
            self.process_and_store_image(msg)

    def survivors_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with LOCK:
                GCS_STATE["survivors"] = data.get("survivors", [])
        except Exception:
            pass

    def score_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with LOCK:
                GCS_STATE["score_report"] = data
        except Exception:
            pass


def run_http_server(port=8080):
    for _ in range(5):
        try:
            server = ThreadingHTTPServer(('0.0.0.0', port), GCSRequestHandler)
            print(f"GCS Server running at http://localhost:{port}")
            server.serve_forever()
            break
        except OSError:
            time.sleep(1.0)


def main(args=None):
    http_thread = threading.Thread(target=run_http_server, args=(8080,), daemon=True)
    http_thread.start()

    rclpy.init(args=args)
    node = GCSRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
