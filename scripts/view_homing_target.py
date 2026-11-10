#!/usr/bin/env python3
"""Read-only RViz target-posture viewer. Sends no robot commands."""

import argparse
import importlib.util
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def load_robot_module(root):
    path = root / "optimizer" / "robot_model.py"
    spec = importlib.util.spec_from_file_location(
        "homing_target_robot_model", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def point(values):
    return Point(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
    )


class HomingTargetViewer(Node):
    def __init__(self, positions, frame):
        super().__init__("homing_target_viewer")
        self.positions = positions
        self.frame = frame
        self.publisher = self.create_publisher(
            MarkerArray, "/knee_task/homing_target", 1
        )
        self.timer = self.create_timer(0.5, self.publish_target)
        self.publish_target()

    def marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.ns = "homing_target"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.65
        marker.color.b = 0.12
        marker.color.a = 0.85
        return marker

    def publish_target(self):
        skeleton = self.marker(0, Marker.LINE_STRIP)
        skeleton.scale.x = 0.009
        skeleton.points = [point(p) for p in self.positions]

        joints = self.marker(1, Marker.SPHERE_LIST)
        joints.scale.x = 0.016
        joints.scale.y = 0.016
        joints.scale.z = 0.016
        joints.points = [point(p) for p in self.positions[1:-1]]

        label = self.marker(2, Marker.TEXT_VIEW_FACING)
        label.pose.position = point(
            self.positions[-1] + np.array([0.0, 0.0, 0.07])
        )
        label.scale.z = 0.025
        label.text = "Target start posture"

        self.publisher.publish(
            MarkerArray(markers=[skeleton, joints, label])
        )

    def clear_target(self):
        markers = []
        for marker_id in (0, 1, 2):
            marker = self.marker(marker_id, Marker.LINE_STRIP)
            marker.action = Marker.DELETE
            markers.append(marker)
        self.publisher.publish(MarkerArray(markers=markers))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--path-dir", required=True)
    args, ros_args = parser.parse_known_args()

    root = Path(__file__).resolve().parents[1]
    module = load_robot_module(root)
    robot = module.RobotModel(
        root / "optimizer" / "models" / "med14_runtime.urdf"
    )

    path = Path(args.path_dir).expanduser().resolve()
    with np.load(
        path / "whole_trajectory_path.npz", allow_pickle=False
    ) as data:
        q = np.asarray(data["q"], dtype=float)
        if (
            q.ndim != 2 or q.shape[1] != 7 or len(q) == 0
            or not np.isfinite(q).all()
        ):
            raise ValueError("Expected a finite saved path with shape (N, 7)")
        q_start = q[0].copy()

    poses = robot.frame_poses(q_start)
    positions = np.array([
        poses[name][:3, 3] for name in robot.chain_frames
    ])

    rclpy.init(args=ros_args)
    node = HomingTargetViewer(positions, module.BASE_FRAME)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.clear_target()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()