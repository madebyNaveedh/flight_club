# nidar-airmouse

Simulation, perception, and Ground Control Station (GCS) stack for the NIDAR 2026-27 GPS-Denied Indoor Search & Survivor Localisation Challenge.

## Stack Overview

- **`nidar_sim`**: 15m x 15m modular maze environment in Gazebo Fortress, drone model with LiDAR & RGB camera, bridge configurations, and navigation controllers.
- **`nidar_perception`**: Real-time survivor detection (YOLOv8 + rescue dummy color segmentation) with 3D-to-2D grid map raycasting and live scoring against ground-truth.
- **`nidar_gcs`**: Web-based Ground Control Station (`http://localhost:8080`) providing live MJPEG camera HUD, 15x15 occupancy map, telemetry, and tagged survivor coordinates.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Ignition Gazebo Fortress (`ros_gz`, `ros_gz_image`)
- Python 3.10+ with PyTorch & Ultralytics

```bash
pip install ultralytics torch torchvision opencv-python
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-image
```

## Quickstart

1. **Build workspace:**
   ```bash
   cd ~/nidar_airmouse_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```

2. **Launch simulation & GCS:**
   ```bash
   ./launch_all.sh
   ```
   Open `http://localhost:8080` in your browser for the GCS dashboard.

3. **Fly the drone (Keyboard Teleop):**
   ```bash
   source install/setup.bash
   ros2 run nidar_sim teleop_keyboard
   ```
   - `W / A / S / D`: Forward / Strafe Left / Back / Strafe Right
   - `I / K`: Ascend / Descend
   - `J / L`: Yaw Left / Yaw Right
   - `1`: Auto-navigate corridor into Room A (Survivor #1)
   - `0`: Auto-return to Launch Pad
   - `SPACE`: Hover / Hold Position

## Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera feed from Gazebo |
| `/nidar/annotated_feed` | `sensor_msgs/Image` | YOLOv8 annotated stream with HUD |
| `/scan` | `sensor_msgs/LaserScan` | 2D 360° LiDAR data |
| `/odom` | `nav_msgs/Odometry` | Drone odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Flight velocity commands |
| `/nidar/survivors_data` | `std_msgs/String` | JSON list of confirmed survivor coordinates |
| `/nidar/mission_score` | `std_msgs/String` | Live competition scoring breakdown |
