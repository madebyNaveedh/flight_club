#!/usr/bin/env python3
"""
NIDAR AirMouse Maze & World Generator
Generates an SDF world file conforming strictly to the NIDAR 2026-27 Rulebook:
- Arena Size: 15m x 15m modular grid
- Wall Height: 8 ft (2.44 m)
- Corridors: Clear width of 1.0 m
- Rooms: Standard 2.0 m x 2.0 m
- Roof: Covered with top net mesh at 8 ft height
- Entry/Exit: Designated 1m doorway with 2ft x 2ft launch zone
- Survivors: 6 survivor dummy models in rooms
- Output: SDF World + Ground-truth reference_grid.json for automated scoring
"""

import os
import json
import random

def generate_nidar_world(output_sdf_path, output_json_path):
    os.makedirs(os.path.dirname(output_sdf_path), exist_ok=True)
    os.makedirs(os.path.dirname(output_json_path), exist_ok=True)
    
    ARENA_SIZE = 15.0  # 15m x 15m
    WALL_HEIGHT = 2.44 # 8 feet
    WALL_THICKNESS = 0.08
    
    # 15x15 discrete grid: 0 = corridor/empty, 1 = wall, 2 = survivor, 3 = launch
    grid = [[0 for _ in range(15)] for _ in range(15)]
    
    # Outer boundaries: (0, y), (14, y), (x, 0), (x, 14)
    # Entry/Exit at x=0, y=1 (open doorway)
    survivor_positions = [
        {"id": 1, "grid": [3, 3], "x": 3.5, "y": 3.5, "room": "Room A"},
        {"id": 2, "grid": [11, 2], "x": 11.5, "y": 2.5, "room": "Room B"},
        {"id": 3, "grid": [7, 7], "x": 7.5, "y": 7.5, "room": "Central Chamber"},
        {"id": 4, "grid": [2, 11], "x": 2.5, "y": 11.5, "room": "Room C"},
        {"id": 5, "grid": [12, 12], "x": 12.5, "y": 12.5, "room": "Room D"},
        {"id": 6, "grid": [7, 12], "x": 7.5, "y": 12.5, "room": "Room E"}
    ]
    
    # Walls configuration (x, y, length_x, length_y)
    # Designed with 1m corridors and 2m x 2m standard rooms
    walls = [
        # Outer boundary walls
        {"name": "outer_south", "x": 7.5, "y": 0.0, "lx": 15.0, "ly": WALL_THICKNESS},
        {"name": "outer_north", "x": 7.5, "y": 15.0, "lx": 15.0, "ly": WALL_THICKNESS},
        {"name": "outer_east",  "x": 15.0, "y": 7.5, "lx": WALL_THICKNESS, "ly": 15.0},
        # West outer wall with 1m doorway at y=1.0 for Entry/Exit
        {"name": "outer_west_1", "x": 0.0, "y": 0.4, "lx": WALL_THICKNESS, "ly": 0.8},
        {"name": "outer_west_2", "x": 0.0, "y": 8.4, "lx": WALL_THICKNESS, "ly": 13.2},
        
        # Room A (2m x 2m at grid x:2..4, y:2..4)
        {"name": "room_a_w1", "x": 3.5, "y": 2.0, "lx": 3.0, "ly": WALL_THICKNESS},
        {"name": "room_a_w2", "x": 2.0, "y": 3.5, "lx": WALL_THICKNESS, "ly": 3.0},
        {"name": "room_a_w3", "x": 5.0, "y": 4.0, "lx": WALL_THICKNESS, "ly": 2.0},
        
        # Room B (2m x 2m at grid x:10..12, y:2..4)
        {"name": "room_b_w1", "x": 11.5, "y": 2.0, "lx": 3.0, "ly": WALL_THICKNESS},
        {"name": "room_b_w2", "x": 10.0, "y": 3.0, "lx": WALL_THICKNESS, "ly": 2.0},
        {"name": "room_b_w3", "x": 13.0, "y": 3.5, "lx": WALL_THICKNESS, "ly": 3.0},

        # Central Maze Corridors & Chamber (grid x:6..9, y:6..9)
        {"name": "center_w1", "x": 6.0, "y": 7.0, "lx": WALL_THICKNESS, "ly": 2.0},
        {"name": "center_w2", "x": 9.0, "y": 8.0, "lx": WALL_THICKNESS, "ly": 2.0},
        {"name": "center_w3", "x": 7.0, "y": 6.0, "lx": 2.0, "ly": WALL_THICKNESS},
        {"name": "center_w4", "x": 8.0, "y": 9.0, "lx": 2.0, "ly": WALL_THICKNESS},
        
        # Internal corridors dividing sectors
        {"name": "div_s1", "x": 4.0, "y": 7.5, "lx": 4.0, "ly": WALL_THICKNESS},
        {"name": "div_s2", "x": 11.0, "y": 7.5, "lx": 4.0, "ly": WALL_THICKNESS},
        {"name": "div_v1", "x": 7.5, "y": 3.5, "lx": WALL_THICKNESS, "ly": 4.0},
        
        # Room C (2m x 2m at grid x:2..4, y:10..12)
        {"name": "room_c_w1", "x": 2.0, "y": 11.5, "lx": WALL_THICKNESS, "ly": 3.0},
        {"name": "room_c_w2", "x": 3.5, "y": 13.0, "lx": 3.0, "ly": WALL_THICKNESS},
        {"name": "room_c_w3", "x": 5.0, "y": 11.0, "lx": WALL_THICKNESS, "ly": 2.0},

        # Room D (2m x 2m at grid x:11..13, y:11..13)
        {"name": "room_d_w1", "x": 13.0, "y": 11.5, "lx": WALL_THICKNESS, "ly": 3.0},
        {"name": "room_d_w2", "x": 11.5, "y": 13.0, "lx": 3.0, "ly": WALL_THICKNESS},
        {"name": "room_d_w3", "x": 10.0, "y": 12.0, "lx": WALL_THICKNESS, "ly": 2.0},

        # Room E (2m x 2m at grid x:6..8, y:11..13)
        {"name": "room_e_w1", "x": 6.0, "y": 12.5, "lx": WALL_THICKNESS, "ly": 3.0},
        {"name": "room_e_w2", "x": 8.0, "y": 12.5, "lx": WALL_THICKNESS, "ly": 3.0},
        {"name": "room_e_w3", "x": 7.0, "y": 14.0, "lx": 2.0, "ly": WALL_THICKNESS},
    ]

    # Generate SDF XML
    sdf_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="nidar_airmouse_arena">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="ignition-gazebo-sensors-system"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Ambient lighting simulating indoor dim conditions -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>7.5 7.5 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane (15m x 15m Floor with Grid Pattern) -->
    <model name="arena_floor">
      <static>true</static>
      <pose>7.5 7.5 0 0 0 0</pose>
      <link name="floor_link">
        <collision name="collision">
          <geometry>
            <box><size>15.0 15.0 0.05</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>15.0 15.0 0.05</size></box>
          </geometry>
          <material>
            <ambient>0.25 0.25 0.28 1</ambient>
            <diffuse>0.35 0.35 0.38 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 2ft x 2ft Launch Pad (0.61m x 0.61m) at (0.5, 1.0) -->
    <model name="launch_pad">
      <static>true</static>
      <pose>0.6 1.0 0.03 0 0 0</pose>
      <link name="pad_link">
        <visual name="pad_visual">
          <geometry>
            <box><size>0.61 0.61 0.01</size></box>
          </geometry>
          <material>
            <ambient>0.9 0.7 0.0 1</ambient>
            <diffuse>1.0 0.8 0.0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Top Net Ceiling at 8 ft (2.44m) -->
    <model name="top_net_ceiling">
      <static>true</static>
      <pose>7.5 7.5 2.44 0 0 0</pose>
      <link name="net_link">
        <collision name="net_collision">
          <geometry>
            <box><size>15.0 15.0 0.02</size></box>
          </geometry>
        </collision>
        <visual name="net_visual">
          <geometry>
            <box><size>15.0 15.0 0.02</size></box>
          </geometry>
          <material>
            <ambient>0.1 0.1 0.1 0.4</ambient>
            <diffuse>0.1 0.1 0.1 0.05</diffuse><transparency>0.95</transparency>
          </material>
        </visual>
      </link>
    </model>
"""
    # Append walls
    for w in walls:
        sdf_content += f"""
    <!-- Wall: {w['name']} -->
    <model name="{w['name']}">
      <static>true</static>
      <pose>{w['x']} {w['y']} {WALL_HEIGHT/2.0} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{w['lx']} {w['ly']} {WALL_HEIGHT}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{w['lx']} {w['ly']} {WALL_HEIGHT}</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.65 1</ambient>
            <diffuse>0.7 0.7 0.75 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    # Append Survivors (Prone & Seated mannequin dummies)
    for s in survivor_positions:
        sdf_content += f"""
    <!-- Survivor {s['id']}: {s['room']} -->
    <model name="survivor_{s['id']}">
      <static>true</static>
      <pose>{s['x']} {s['y']} 0.15 0 0 0</pose>
      <link name="body">
        <visual name="torso">
          <pose>0 0 0.1 0 0 0</pose>
          <geometry>
            <box><size>0.8 0.4 0.2</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.9 0.1 0.1 1</diffuse>
          </material>
        </visual>
        <visual name="head">
          <pose>0.5 0 0.15 0 0 0</pose>
          <geometry>
            <sphere><radius>0.14</radius></sphere>
          </geometry>
          <material>
            <ambient>0.9 0.75 0.6 1</ambient>
            <diffuse>0.95 0.8 0.65 1</diffuse>
          </material>
        </visual>
        <visual name="limbs">
          <pose>-0.5 0 0.08 0 0 0</pose>
          <geometry>
            <box><size>0.6 0.35 0.15</size></box>
          </geometry>
          <material>
            <ambient>0.1 0.2 0.8 1</ambient>
            <diffuse>0.1 0.3 0.9 1</diffuse>
          </material>
        </visual>
        <collision name="col">
          <geometry>
            <box><size>1.2 0.5 0.3</size></box>
          </geometry>
        </collision>
      </link>
    </model>
"""

    sdf_content += """
  </world>
</sdf>
"""

    with open(output_sdf_path, 'w') as f:
        f.write(sdf_content)

    # Ground-truth JSON data structure
    ground_truth = {
        "competition": "NIDAR 2026-27 AirMouse Challenge",
        "arena": {
            "size_meters": [ARENA_SIZE, ARENA_SIZE],
            "grid_cells": 15,
            "cell_size_meters": 1.0,
            "corridor_min_width_meters": 1.0,
            "room_dimensions_meters": [2.0, 2.0],
            "ceiling_height_meters": WALL_HEIGHT,
            "launch_pad_coords": [0.6, 1.0],
            "entry_exit_coords": [0.0, 1.0]
        },
        "survivors": survivor_positions,
        "walls": walls
    }

    with open(output_json_path, 'w') as f:
        json.dump(ground_truth, f, indent=2)

    print(f"Generated NIDAR Maze SDF: {output_sdf_path}")
    print(f"Generated Ground-Truth JSON: {output_json_path}")

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sdf_path = os.path.join(base_dir, "worlds", "nidar_airmouse_arena.sdf")
    json_path = os.path.join(base_dir, "worlds", "reference_grid.json")
    generate_nidar_world(sdf_path, json_path)
