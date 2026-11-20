#!/usr/bin/env python3
"""Run a selected optimizer knee-task plan through JIC and CIC.

The runner loads a hash-verified exported plan bundle, initializes the robot
to the plan start configuration with JIC, activates CIC with secondary control
disabled, verifies a static hold, and replays the absolute end-effector
reference using simulation time.

The C++ controller is not modified by this script.
"""

import argparse
import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
import hashlib


import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation

from copy import deepcopy
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Point
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RCLParameter, ParameterType
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController, ListControllers
from lbr_impedance_control.action import JointTrajectoryExecution

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
OPTIMIZER_ROOT = REPOSITORY_ROOT / "optimizer"

for import_path in (REPOSITORY_ROOT, OPTIMIZER_ROOT):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from integration.initialization import initialize, joint_sample
from task_envelope import TaskEnvelope
from view_homing_target import load_robot_module

ROBOT_NAME = "lbr"
NAMESPACE = f"/{ROBOT_NAME}"
FRAME = "lbr_link_0"
JOINT_NAMES = [f"lbr_A{index}" for index in range(1, 8)]

JIC = "joint_impedance_controller"
CIC = "cartesian_impedance_controller"

JIC_ACTION = f"{NAMESPACE}/{JIC}/joint_trajectory"
JIC_PARAMETERS = f"{NAMESPACE}/{JIC}/set_parameters"
CIC_PARAMETERS = f"{NAMESPACE}/{CIC}/set_parameters"

CIC_REFERENCE_POSE = f"{NAMESPACE}/{CIC}/reference_cartesian_pose"
CIC_REFERENCE_TWIST = f"{NAMESPACE}/{CIC}/reference_cartesian_twist"
CIC_ACTUAL_POSE = f"{NAMESPACE}/{CIC}/cartesian_pose"
CIC_COMMANDED_TORQUE = f"{NAMESPACE}/{CIC}/commanded_torque"
CIC_SECONDARY_TORQUE = f"{NAMESPACE}/{CIC}/nullspace_torque"

JOINT_STATE_TOPIC = f"{NAMESPACE}/joint_states"
CONTROLLER_MANAGER = f"{NAMESPACE}/controller_manager"

JIC_STIFFNESS = 300.0
JIC_DAMPING_RATIO = 0.7
JIC_SEPARATE_JOINTS = False
JIC_REFERENCE_SPEED = 0.05

CIC_TRANSLATIONAL_STIFFNESS = 3000.0
CIC_ROTATIONAL_STIFFNESS = 150.0
CIC_DAMPING_RATIO = 1.0

TIBIA_LENGTH_M = 0.31
TIBIA_DIAMETER_M = 0.06
TIBIA_STRIP_LENGTH_M = 0.24
TIBIA_STRIP_DIAMETER_M = 0.008
TIBIA_STRIP_OFFSET_M = 0.045

NULLSPACE_ENABLED = False
NULLSPACE_JOINT_LIMIT_WEIGHT = 0.0
NULLSPACE_TORQUE_MIN_WEIGHT = 0.0
NULLSPACE_SINGULARITY_WEIGHT = 0.0
NULLSPACE_DAMPING = 0.0
NULLSPACE_MAX_TORQUE = 10.0

RATE_HZ = 50.0
STATIC_HOLD_S = 2.0
FINAL_HOLD_S = 2.0

START_POSITION_TOLERANCE_M = 0.02
START_ROTATION_TOLERANCE_RAD = np.deg2rad(5.0)

ABORT_POSITION_ERROR_M = 0.08
ABORT_DURATION_S = 2.0

REFERENCE_MAX_AGE_S = 0.20


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, content):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(content, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def append_event(path, event):
    with Path(path).open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, allow_nan=False) + "\n")


def require_confirmation(prompt, run_dir, event_name):
    print("\n" + "=" * 72)
    print(prompt)
    print("Press ENTER to confirm, or type anything else to abort.")
    print("=" * 72)

    response = input("Confirmation: ").strip().lower()

    if response not in ("", "y", "yes"):
        raise RuntimeError(f"Operator aborted before: {event_name}")

    append_event(
        run_dir / "events.jsonl",
        {
            "event": event_name,
            "confirmed_by": "operator_enter",
            "response": response or "enter",
        },
    )


def parameter(name, value, parameter_type=None):
    message = RCLParameter()
    message.name = name

    if parameter_type == "double_array":
        message.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        message.value.double_array_value = [float(item) for item in value]
    elif parameter_type == "string_array":
        message.value.type = ParameterType.PARAMETER_STRING_ARRAY
        message.value.string_array_value = [str(item) for item in value]
    elif isinstance(value, bool):
        message.value.type = ParameterType.PARAMETER_BOOL
        message.value.bool_value = value
    elif isinstance(value, str):
        message.value.type = ParameterType.PARAMETER_STRING
        message.value.string_value = value
    else:
        message.value.type = ParameterType.PARAMETER_DOUBLE
        message.value.double_value = float(value)

    return message


class SelectedPlanRunner(Node):
    def __init__(self):
        super().__init__("run_selected_knee_baseline")

        self.joint_state = None
        self.actual_pose = None
        self.actual_pose_received_time = None
        self.commanded_torque = np.full(7, np.nan)
        self.secondary_torque = np.full(7, np.nan)

        self.jic_action = ActionClient(
            self,
            JointTrajectoryExecution,
            JIC_ACTION,
        )
        self.jic_parameters = self.create_client(
            SetParameters,
            JIC_PARAMETERS,
        )
        self.cic_parameters = self.create_client(
            SetParameters,
            CIC_PARAMETERS,
        )
        self.controller_list = self.create_client(
            ListControllers,
            f"{CONTROLLER_MANAGER}/list_controllers",
        )
        self.controller_switch = self.create_client(
            SwitchController,
            f"{CONTROLLER_MANAGER}/switch_controller",
        )

        self.reference_pose_publisher = self.create_publisher(
            PoseStamped,
            CIC_REFERENCE_POSE,
            1,
        )
        self.reference_twist_publisher = self.create_publisher(
            TwistStamped,
            CIC_REFERENCE_TWIST,
            1,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "knee_task_visualization",
            10,
        )
        self.homing_marker_publisher = self.create_publisher(
            MarkerArray,
            "knee_task/homing_target",
            1,
        )

        self.homing_timer = None
        self.homing_positions = None
        self.actual_points = []
        self.last_reference_position = None
        self.last_reference_rotation = None
        self.attachment_transform = None
        self.placement_transform = None
        self.reference_positions = None
        self.robot = None

        self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.store_joint_state,
            10,
        )
        self.create_subscription(
            PoseStamped,
            CIC_ACTUAL_POSE,
            self.store_actual_pose,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            CIC_COMMANDED_TORQUE,
            self.store_commanded_torque,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            CIC_SECONDARY_TORQUE,
            self.store_secondary_torque,
            10,
        )

        # Compatibility interface expected by integration.initialization.initialize()
        self.jic_params = self.jic_parameters
        self.switch = self.switch_controllers

    def store_joint_state(self, message):
        self.joint_state = message

    @property
    def js(self):
        return self.joint_state

    def store_actual_pose(self, message):
        self.actual_pose = message
        self.actual_pose_received_time = (
            self.get_clock().now().nanoseconds * 1e-9
        )

    def store_commanded_torque(self, message):
        self.commanded_torque = np.asarray(message.data, dtype=float)

    def store_secondary_torque(self, message):
        self.secondary_torque = np.asarray(message.data, dtype=float)

    def make_parameter(self, name, value):
        return parameter(name, value)

    def call_service(self, client, request, timeout=8.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")

        future = client.call_async(request)
        deadline = time.monotonic() + timeout

        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(f"Service timed out: {client.srv_name}")
            rclpy.spin_once(self, timeout_sec=min(0.02, remaining))

        if not future.done():
            raise RuntimeError(f"Service interrupted: {client.srv_name}")

        response = future.result()
        if response is None:
            raise RuntimeError(f"No service response: {client.srv_name}")

        return response

    def controller_states(self):
        request = ListControllers.Request()
        response = self.call_service(self.controller_list, request)
        return {
            controller.name: controller.state
            for controller in response.controller
        }

    def clear_task_visualization(self):
        """Clear markers from a previously completed or interrupted task run.

        This is called when a new executor run starts, before the new homing
        visualization is shown. It intentionally does not clear the homing
        visualization.
        """
        marker = Marker()
        marker.header.frame_id = FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL

        self.marker_publisher.publish(MarkerArray(markers=[marker]))

    def switch_controllers(self, activate=None, deactivate=None):
        states = self.controller_states()

        activate_controllers = []
        if activate is not None and states.get(activate) != "active":
            activate_controllers.append(activate)

        deactivate_controllers = []
        if states.get(deactivate) == "active":
            deactivate_controllers.append(deactivate)

        if not activate_controllers and not deactivate_controllers:
            return True

        request = SwitchController.Request()
        request.activate_controllers = activate_controllers
        request.deactivate_controllers = deactivate_controllers
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5

        response = self.call_service(self.controller_switch, request)
        return response is not None and response.ok

    def set_jic_profile(self):
        request = SetParameters.Request()
        request.parameters = [
            parameter("stiffness", JIC_STIFFNESS),
            parameter("damping_ratio", JIC_DAMPING_RATIO),
            parameter("separate_joints", JIC_SEPARATE_JOINTS),
        ]

        response = self.call_service(self.jic_parameters, request)

        for result in response.results:
            if not result.successful:
                raise RuntimeError("JIC profile parameter was rejected.")

    def set_cic_profile(self):
        request = SetParameters.Request()
        request.parameters = [
            parameter(
                "translational_stiffness",
                CIC_TRANSLATIONAL_STIFFNESS,
            ),
            parameter(
                "rotational_stiffness",
                CIC_ROTATIONAL_STIFFNESS,
            ),
            parameter(
                "translational_damping_ratio",
                CIC_DAMPING_RATIO,
            ),
            parameter(
                "rotational_damping_ratio",
                CIC_DAMPING_RATIO,
            ),
            parameter("ns_enabled", NULLSPACE_ENABLED),
            parameter(
                "ns_weight_joint_limit",
                NULLSPACE_JOINT_LIMIT_WEIGHT,
            ),
            parameter(
                "ns_weight_torque_min",
                NULLSPACE_TORQUE_MIN_WEIGHT,
            ),
            parameter(
                "ns_weight_singularity",
                NULLSPACE_SINGULARITY_WEIGHT,
            ),
            parameter("ns_damping", NULLSPACE_DAMPING),
            parameter("ns_max_torque", NULLSPACE_MAX_TORQUE),
            parameter("ns_headroom_scaling", False),
        ]

        response = self.call_service(self.cic_parameters, request)

        for result in response.results:
            if not result.successful:
                raise RuntimeError("CIC profile parameter was rejected.")

    def wait_for_fresh_joint_state(self, timeout=5.0):
        previous = self.joint_state
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

            if self.joint_state is None or self.joint_state is previous:
                continue

            return joint_sample(self.joint_state)

        raise RuntimeError("No fresh joint state received.")

    def measured_pose(self, timeout=5.0):
        previous = self.actual_pose
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

            if self.actual_pose is None or self.actual_pose is previous:
                continue

            position = self.actual_pose.pose.position
            orientation = self.actual_pose.pose.orientation

            return (
                np.array(
                    [
                        position.x,
                        position.y,
                        position.z,
                    ]
                ),
                Rotation.from_quat(
                    [
                        orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w,
                    ]
                ),
                self.actual_pose_received_time,
            )

        raise RuntimeError("No measured CIC pose received.")

    def publish_reference(self, position, rotation, twist=None):
        message = PoseStamped()
        message.header.frame_id = FRAME
        message.header.stamp = self.get_clock().now().to_msg()

        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])

        quaternion = rotation.as_quat()
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])

        self.reference_pose_publisher.publish(message)

        if twist is not None:
            twist_message = TwistStamped()
            twist_message.header.frame_id = FRAME
            twist_message.header.stamp = self.get_clock().now().to_msg()

            twist_message.twist.linear.x = float(twist[0])
            twist_message.twist.linear.y = float(twist[1])
            twist_message.twist.linear.z = float(twist[2])
            twist_message.twist.angular.x = float(twist[3])
            twist_message.twist.angular.y = float(twist[4])
            twist_message.twist.angular.z = float(twist[5])

            self.reference_twist_publisher.publish(twist_message)

    def publish_zero_twist(self):
        self.publish_reference_twist(np.zeros(3), np.zeros(3))

    def publish_reference_twist(self, linear, angular):
        message = TwistStamped()
        message.header.frame_id = FRAME
        message.header.stamp = self.get_clock().now().to_msg()

        message.twist.linear.x = float(linear[0])
        message.twist.linear.y = float(linear[1])
        message.twist.linear.z = float(linear[2])
        message.twist.angular.x = float(angular[0])
        message.twist.angular.y = float(angular[1])
        message.twist.angular.z = float(angular[2])

        self.reference_twist_publisher.publish(message)


    def prepare_homing_visualization(self, q_start):
        if self.robot is None:
            raise RuntimeError("Robot model is not attached to the runner.")

        poses = self.robot.frame_poses(np.asarray(q_start, dtype=float))
        self.homing_positions = np.array([
            poses[name][:3, 3]
            for name in self.robot.chain_frames
        ])

    def homing_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
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

    @staticmethod
    def visual_point(values):
        point = Point()
        point.x, point.y, point.z = map(float, values)
        return point

    def publish_homing_visualization(self):
        if self.homing_positions is None:
            return

        positions = self.homing_positions
        markers = []

        skeleton = self.homing_marker(0, Marker.LINE_STRIP)
        skeleton.scale.x = 0.009
        skeleton.points = [self.visual_point(p) for p in positions]
        markers.append(skeleton)

        joints = self.homing_marker(1, Marker.SPHERE_LIST)
        joints.scale.x = 0.016
        joints.scale.y = 0.016
        joints.scale.z = 0.016
        joints.points = [self.visual_point(p) for p in positions[1:-1]]
        markers.append(joints)

        label = self.homing_marker(2, Marker.TEXT_VIEW_FACING)
        label.pose.position = self.visual_point(
            positions[-1] + np.array([0.0, 0.0, 0.14])
        )
        label.scale.z = 0.075
        label.text = "TARGET\nSTART\nPOSTURE"
        label.color.a = 1.0
        markers.append(label)

        self.homing_marker_publisher.publish(MarkerArray(markers=markers))

    def show_homing_visualization(self):
        if self.homing_positions is None:
            raise RuntimeError("Homing visualization was not prepared.")

        self.publish_homing_visualization()
        if self.homing_timer is None:
            self.homing_timer = self.create_timer(
                0.5,
                self.publish_homing_visualization,
            )

    def hide_homing_visualization(self):
        if self.homing_timer is not None:
            self.homing_timer.cancel()
            self.destroy_timer(self.homing_timer)
            self.homing_timer = None

        markers = []
        for marker_id in (0, 1, 2):
            marker = self.homing_marker(marker_id, Marker.LINE_STRIP)
            marker.action = Marker.DELETE
            markers.append(marker)

        self.homing_marker_publisher.publish(MarkerArray(markers=markers))

    def set_task_visualization_context(self, attachment, reference):
        self.attachment_transform = np.asarray(attachment, dtype=float)
        self.reference_positions = np.asarray(
            reference[:, :3, 3],
            dtype=float,
        )
        self.actual_points = []

        self.last_reference_position = self.reference_positions[0].copy()
        self.last_reference_rotation = Rotation.from_matrix(
            reference[0, :3, :3]
        )

        placement = None
        for key in ("X_B_F", "XBF", "X_BF", "placement_transform"):
            try:
                if key in reference:
                    placement = reference[key]
                    break
            except TypeError:
                pass

        if placement is not None:
            self.placement_transform = np.asarray(placement, dtype=float)

    def compose_transform(self, position, rotation, transform):
        composed = np.eye(4)
        composed[:3, :3] = rotation.as_matrix()
        composed[:3, 3] = position
        return composed @ transform

    def send_tf(self, child_frame, position, rotation, stamp):
        transform = TransformStamped()
        transform.header.frame_id = FRAME
        transform.header.stamp = stamp
        transform.child_frame_id = child_frame

        transform.transform.translation.x, \
            transform.transform.translation.y, \
            transform.transform.translation.z = map(float, position)

        quaternion = rotation.as_quat()
        transform.transform.rotation.x, \
            transform.transform.rotation.y, \
            transform.transform.rotation.z, \
            transform.transform.rotation.w = map(float, quaternion)

        self.tf_broadcaster.sendTransform(transform)

    def publish_task_visualization(
        self,
        actual_position,
        actual_rotation,
        publish_markers=True,
    ):
        if (
            self.attachment_transform is None
            or self.last_reference_position is None
            or self.last_reference_rotation is None
        ):
            return

        now = self.get_clock().now().to_msg()
        tibia = self.compose_transform(
            actual_position,
            actual_rotation,
            self.attachment_transform,
        )
        tibia_position = tibia[:3, 3]
        tibia_rotation = Rotation.from_matrix(tibia[:3, :3])

        self.send_tf(
            "ref_ee",
            self.last_reference_position,
            self.last_reference_rotation,
            now,
        )
        self.send_tf(
            "actual_ee",
            actual_position,
            actual_rotation,
            now,
        )
        self.send_tf(
            "tibia",
            tibia_position,
            tibia_rotation,
            now,
        )

        if self.placement_transform is not None:
            self.send_tf(
                "placement",
                self.placement_transform[:3, 3],
                Rotation.from_matrix(self.placement_transform[:3, :3]),
                now,
            )

        if not publish_markers:
            return

        markers = []

        if self.reference_positions is not None:
            reference_marker = Marker()
            reference_marker.header.frame_id = FRAME
            reference_marker.header.stamp = now
            reference_marker.ns = "reference_trajectory"
            reference_marker.id = 0
            reference_marker.type = Marker.LINE_STRIP
            reference_marker.action = Marker.ADD
            reference_marker.pose.orientation.w = 1.0
            reference_marker.scale.x = 0.004
            reference_marker.color.b = 1.0
            reference_marker.color.a = 0.8
            reference_marker.points = [
                self.visual_point(position)
                for position in self.reference_positions[::20]
            ]
            markers.append(reference_marker)

        trail = Marker()
        trail.header.frame_id = FRAME
        trail.header.stamp = now
        trail.ns = "actual_trajectory"
        trail.id = 1
        trail.type = Marker.POINTS
        trail.action = Marker.ADD
        trail.pose.orientation.w = 1.0
        trail.scale.x = 0.008
        trail.scale.y = 0.008
        trail.color.r = 1.0
        trail.color.a = 0.9
        trail.points = [
            self.visual_point(position)
            for position in self.actual_points
        ]
        markers.append(trail)

        sphere = Marker()
        sphere.header.frame_id = FRAME
        sphere.header.stamp = now
        sphere.ns = "tibia_origin"
        sphere.id = 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = map(float, tibia_position)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.014
        sphere.scale.y = 0.014
        sphere.scale.z = 0.014
        sphere.color.g = 1.0
        sphere.color.a = 0.9
        markers.append(sphere)

        attachment_inverse = np.linalg.inv(self.attachment_transform)
        attachment_offset = attachment_inverse[:3, 3]
        attachment_distance = float(np.linalg.norm(attachment_offset))

        if not np.isfinite(attachment_distance) or attachment_distance <= 1e-9:
            raise ValueError("Invalid tibia-to-EE attachment offset.")

        # The tibia frame's +x axis is the cylinder long axis.
        tibia_axis = np.array([1.0, 0.0, 0.0])
        helper = np.array([0.0, 0.0, 1.0])

        if abs(float(np.dot(tibia_axis, helper))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])

        transverse = np.cross(helper, tibia_axis)
        transverse /= np.linalg.norm(transverse)
        second = np.cross(tibia_axis, transverse)

        cylinder_rotation = tibia_rotation * Rotation.from_matrix(
            np.column_stack((transverse, second, tibia_axis))
        )

        body_center = tibia_position + tibia_rotation.apply(
            0.5 * TIBIA_LENGTH_M * tibia_axis
        )

        body = Marker()
        body.header.frame_id = FRAME
        body.header.stamp = now
        body.ns = "tibia_body"
        body.id = 10
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x, body.pose.position.y, body.pose.position.z = map(float, body_center)

        body_quaternion = cylinder_rotation.as_quat()
        body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z, body.pose.orientation.w = map(float, body_quaternion)

        body.scale.x = TIBIA_DIAMETER_M
        body.scale.y = TIBIA_DIAMETER_M
        body.scale.z = TIBIA_LENGTH_M
        body.color.r = 0.65
        body.color.g = 0.85
        body.color.b = 0.70
        body.color.a = 0.95
        markers.append(body)

        for strip_id, side in ((11, 1.0), (12, -1.0)):
            strip_center = tibia_position + tibia_rotation.apply(
                0.5 * TIBIA_STRIP_LENGTH_M * tibia_axis
                + side * TIBIA_STRIP_OFFSET_M * transverse
            )

            strip = Marker()
            strip.header.frame_id = FRAME
            strip.header.stamp = now
            strip.ns = "tibia_rotation_indicator"
            strip.id = strip_id
            strip.type = Marker.CYLINDER
            strip.action = Marker.ADD
            strip.pose.position.x, strip.pose.position.y, strip.pose.position.z = map(float, strip_center)

            strip_quaternion = cylinder_rotation.as_quat()
            strip.pose.orientation.x, strip.pose.orientation.y, strip.pose.orientation.z, strip.pose.orientation.w = map(float, strip_quaternion)

            strip.scale.x = TIBIA_STRIP_DIAMETER_M
            strip.scale.y = TIBIA_STRIP_DIAMETER_M
            strip.scale.z = TIBIA_STRIP_LENGTH_M
            strip.color.r = 0.15
            strip.color.g = 0.15
            strip.color.b = 0.15
            strip.color.a = 1.0
            markers.append(strip)

        self.marker_publisher.publish(MarkerArray(markers=markers))


def load_plan(plan_dir):
    plan_dir = Path(plan_dir).resolve()
    plan_path = plan_dir / "plan.json"
    reference_path = plan_dir / "reference.npz"

    if not plan_path.is_file():
        raise FileNotFoundError(f"Plan not found: {plan_path}")
    if not reference_path.is_file():
        raise FileNotFoundError(f"Reference not found: {reference_path}")

    plan = load_json(plan_path)

    expected_hash = plan.get("generated", {}).get("reference_npz")
    actual_hash = sha256(reference_path)

    if expected_hash is None:
        raise ValueError("Plan does not record a reference.npz hash.")

    if actual_hash != expected_hash:
        raise ValueError("reference.npz hash does not match plan.json.")

    with np.load(reference_path, allow_pickle=False) as archive:
        reference = {
            key: np.array(archive[key], dtype=float)
            for key in archive.files
        }

    return plan_dir, plan, reference


def create_run_directory(repository_root, plan, duration_s):
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    run_root = repository_root / "runs" / "selected_plan"
    run_root.mkdir(parents=True, exist_ok=True)

    run_dir = run_root / f"{timestamp}_{duration_s:.0f}s_baseline"
    run_dir.mkdir()

    manifest = {
        "schema": "knee_selected_plan_run_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "plan_dir": str(Path(plan["campaign_dir"]).parent),
        "design_id": plan["design_id"],
        "rank": plan["rank"],
        "task_id": plan["task_id"],
        "duration_s": duration_s,
        "rate_hz": RATE_HZ,
        "q_start_rad": plan["q_start_rad"],
        "secondary_control": "off",
        "jic_profile": {
            "stiffness_nm_per_rad": JIC_STIFFNESS,
            "damping_ratio": JIC_DAMPING_RATIO,
            "separate_joints": JIC_SEPARATE_JOINTS,
            "reference_speed_rad_s": JIC_REFERENCE_SPEED,
        },
        "cic_profile": {
            "translational_stiffness": CIC_TRANSLATIONAL_STIFFNESS,
            "rotational_stiffness": CIC_ROTATIONAL_STIFFNESS,
            "damping_ratio": CIC_DAMPING_RATIO,
            "nullspace_enabled": NULLSPACE_ENABLED,
        },
        "status": "created",
    }

    write_json(run_dir / "manifest.json", manifest)
    append_event(
        run_dir / "events.jsonl",
        {
            "event": "run_created",
            "design_id": plan["design_id"],
            "duration_s": duration_s,
        },
    )

    return run_dir


def main():
    parser = argparse.ArgumentParser(
        description="Run a selected knee plan through JIC and CIC."
    )
    parser.add_argument(
        "--plan-dir",
        required=True,
        help="Exported plan bundle directory.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Task duration to replay in seconds. Required unless --initialize-only is used.",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=50,
        help="Reference publication rate in Hz.",
    )
    parser.add_argument(
        "--initialize-only",
        action="store_true",
        help="Initialize with JIC and exit before CIC activation.",
    )
    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("Reference rate must be positive.")
    if args.duration is not None and args.duration <= 0:
        raise ValueError("Duration must be positive.")

    plan_dir, plan, reference = load_plan(args.plan_dir)

    duration_s = (
        float(args.duration)
        if args.duration is not None
        else 0.0
    )
    task_end_s = float(plan["task_duration_s"])
    if not args.initialize_only and args.duration is None:
        parser.error(
            "--duration is required unless --initialize-only is used."
        )

    if duration_s > task_end_s:
        raise ValueError(
            f"Requested duration {duration_s} exceeds task duration {task_end_s}."
        )

    rate = float(args.rate)
    sample_period = 1.0 / rate

    time_ee = reference["time_ee_s"]
    ee_reference = reference["X_BE_task"]
    time_q = reference["time_q_s"]
    q_reference = reference["q_ref_rad"]
    attachment = reference["A_X_ET"]
    q_start = np.asarray(plan["q_start_rad"], dtype=float)

    robot_module = load_robot_module(REPOSITORY_ROOT)
    robot = robot_module.RobotModel(
        OPTIMIZER_ROOT / "models" / "med14_runtime.urdf"
    )

    required_start_position = reference["X_BE_start_required"][:3, 3]

    ee_frames = ("lbr_link_7", "lbr_link_ee")
    frame_positions = {}

    for frame_name in ee_frames:
        try:
            frame_pose = robot.frame_poses(
                q_start,
                frames=[frame_name],
            )[frame_name]
        except TypeError:
            frame_pose = robot.frame_poses(q_start)[frame_name]

        frame_position = np.asarray(frame_pose[:3, 3], dtype=float)
        frame_positions[frame_name] = frame_position

        offset = frame_position - required_start_position
        print(
            f"EE-frame diagnostic: {frame_name} position error = "
            f"{np.linalg.norm(offset):.6f} m"
        )
        print(f"  offset [m]: {offset.tolist()}")

    closest_frame = min(
        ee_frames,
        key=lambda name: np.linalg.norm(
            frame_positions[name] - required_start_position
        ),
    )
    print(f"EE-frame diagnostic: closest frame = {closest_frame}")

    envelope = TaskEnvelope(plan_dir / "task")

    rclpy.init()
    node = SelectedPlanRunner()
    node.robot = robot

    run_dir = create_run_directory(
        REPOSITORY_ROOT,
        plan,
        duration_s,
    )

    try:
        states = node.controller_states()

        # If a previous CIC task is still active, deactivate it before
        # initializing the next plan with JIC.
        if states.get(CIC) == "active":
            if not node.switch_controllers(deactivate=CIC):
                raise RuntimeError(
                    "Could not deactivate CIC before JIC initialization."
                )

        # Activate JIC automatically if it is not already active.
        if states.get(JIC) != "active":
            if not node.switch_controllers(activate=JIC):
                raise RuntimeError("Could not activate JIC automatically.")

        # Recheck the final controller state before initialization.
        states = node.controller_states()
        if states.get(JIC) != "active":
            raise RuntimeError("JIC is not active after automatic activation.")
        if states.get(CIC) != "inactive":
            raise RuntimeError("CIC is not inactive after deactivation.")

        _, measured_q, measured_qd = node.wait_for_fresh_joint_state()

        print("\nSelected plan:")
        print(f"  design: {plan['design_id']}")
        print(f"  path: {plan['selected_path']}")
        print(f"  task duration: {task_end_s:.1f} s")
        print(f"  requested duration: {duration_s:.1f} s")
        print(f"  q_start [rad]: {q_start.tolist()}")
        print(f"  current q [rad]: {measured_q.tolist()}")

        node.clear_task_visualization()
        node.prepare_homing_visualization(q_start)
        node.show_homing_visualization()

        require_confirmation(
            "Authorize JIC motion to the selected optimizer start posture?\n"
            f"Target q_start [rad]: {q_start.tolist()}\n"
            f"JIC: stiffness={JIC_STIFFNESS} Nm/rad, damping ratio={JIC_DAMPING_RATIO}, "
            f"separate joints={'on' if JIC_SEPARATE_JOINTS else 'off'}, "
            f"reference speed={JIC_REFERENCE_SPEED} rad/s",
            run_dir,
            "jic_motion_authorized",
        )

        node.set_jic_profile()

        initialization_policy = {
            "position_tolerance_rad": 0.06,
            "velocity_tolerance_rad_s": 0.05,
            "settle_duration_sim_s": 0.5,
            "maximum_sample_gap_sim_s": 0.25,
            "settling_timeout_wall_s": 10.0,
            "action_timeout_margin": 2.5,
            "action_timeout_floor_s": 5.0,
        }

        initialized = initialize(
            node=node,
            q_start=q_start,
            directory=run_dir,
            action_type=JointTrajectoryExecution,
            stiffness=JIC_STIFFNESS,
            velocity=JIC_REFERENCE_SPEED,
            policy=initialization_policy,
        )

        if not initialized:
            raise RuntimeError("JIC initialization failed.")

        _, measured_q, measured_qd = node.wait_for_fresh_joint_state()

        position_error = float(np.max(np.abs(measured_q - q_start)))
        velocity = float(np.max(np.abs(measured_qd)))

        append_event(
            run_dir / "events.jsonl",
            {
                "event": "jic_initialization_passed",
                "position_error_rad": position_error,
                "velocity_rad_s": velocity,
            },
        )

        if args.initialize_only:
            print("Initialization-only complete; CIC was not activated.")
            return

        require_confirmation(
            "JIC initialization passed.\n"
            "Authorize CIC activation and selected-task replay?\n"
            f"Plan: {plan_dir}\n"
            f"Duration: {duration_s:.1f} s\n"
            "Secondary control: off\n"
            "The task is anchored to the exported absolute reference; "
            "the current pose will not be used as the task origin.",
            run_dir,
            "cic_task_authorized",
        )

        node.switch_controllers(CIC, JIC)
        node.set_cic_profile()

        start_pose = ee_reference[0]
        start_position = start_pose[:3, 3]
        start_rotation = Rotation.from_matrix(start_pose[:3, :3])

        node.set_task_visualization_context(
            attachment=attachment,
            reference=ee_reference,
        )

        node.publish_reference(
            start_position,
            start_rotation,
            twist=np.zeros(6),
        )

        static_hold_start = node.get_clock().now()

        while rclpy.ok():
            elapsed = (
                node.get_clock().now() - static_hold_start
            ).nanoseconds * 1e-9

            if elapsed >= STATIC_HOLD_S:
                break

            measured_position, measured_rotation, _ = node.measured_pose()

            position_error = np.linalg.norm(
                measured_position - start_position
            )
            rotation_error = (
                measured_rotation * start_rotation.inv()
            ).magnitude()

            if (
                position_error > START_POSITION_TOLERANCE_M
                or rotation_error > START_ROTATION_TOLERANCE_RAD
            ):
                raise RuntimeError(
                    "Static hold failed before task replay."
                )

            time.sleep(sample_period)

        append_event(
            run_dir / "events.jsonl",
            {
                "event": "static_hold_passed",
                "duration_s": STATIC_HOLD_S,
            },
        )

        node.hide_homing_visualization()

        replay_start = node.get_clock().now()
        abort_start = None

        csv_path = run_dir / "knee_task_baseline_log.csv"

        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "simulation_time_s",
                "task_time_s",
                "phase_id",
                "phase_name",
                "reference_ee_position_x",
                "reference_ee_position_y",
                "reference_ee_position_z",
                "reference_ee_quaternion_x",
                "reference_ee_quaternion_y",
                "reference_ee_quaternion_z",
                "reference_ee_quaternion_w",
                "reference_twist_linear_x",
                "reference_twist_linear_y",
                "reference_twist_linear_z",
                "reference_twist_angular_x",
                "reference_twist_angular_y",
                "reference_twist_angular_z",
                "actual_ee_position_x",
                "actual_ee_position_y",
                "actual_ee_position_z",
                "actual_ee_quaternion_x",
                "actual_ee_quaternion_y",
                "actual_ee_quaternion_z",
                "actual_ee_quaternion_w",
                "actual_tibia_position_x",
                "actual_tibia_position_y",
                "actual_tibia_position_z",
                "actual_tibia_quaternion_x",
                "actual_tibia_quaternion_y",
                "actual_tibia_quaternion_z",
                "actual_tibia_quaternion_w",
                *JOINT_NAMES,
                *[f"{name}_velocity" for name in JOINT_NAMES],
                *[f"{name}_reference" for name in JOINT_NAMES],
                *[f"{name}_room_rad" for name in JOINT_NAMES],
                "limiting_joint",
                *[f"commanded_torque_{index}" for index in range(1, 8)],
                *[f"secondary_torque_{index}" for index in range(1, 8)],
                "reference_age_s",
                "fault_flags",
            ])

            while rclpy.ok():
                elapsed = (
                    node.get_clock().now() - replay_start
                ).nanoseconds * 1e-9

                task_time = min(elapsed, duration_s)
                index = int(round(task_time * rate))
                index = min(index, len(time_ee) - 1)

                reference_pose = ee_reference[index]
                reference_position = reference_pose[:3, 3]
                reference_rotation = Rotation.from_matrix(
                    reference_pose[:3, :3]
                )

                if index == 0:
                    twist = np.zeros(6)
                else:
                    previous = max(0, index - 1)
                    following = min(len(time_ee) - 1, index + 1)

                    delta_time = (
                        time_ee[following] - time_ee[previous]
                    )

                    if delta_time <= 0:
                        twist = np.zeros(6)
                    else:
                        delta_position = (
                            ee_reference[following][:3, 3]
                            - ee_reference[previous][:3, 3]
                        )

                        delta_rotation = (
                            Rotation.from_matrix(
                                ee_reference[following][:3, :3]
                            )
                            * Rotation.from_matrix(
                                ee_reference[previous][:3, :3]
                            ).inv()
                        ).as_rotvec()

                        twist = np.concatenate(
                            [
                                delta_position / delta_time,
                                delta_rotation / delta_time,
                            ]
                        )

                node.publish_reference(
                    reference_position,
                    reference_rotation,
                    twist=twist,
                )

                rclpy.spin_once(node, timeout_sec=0.0)

                measured_position, measured_rotation, measured_stamp = (
                    node.measured_pose()
                )

                node.last_reference_position = reference_position.copy()
                node.last_reference_rotation = reference_rotation
                node.actual_points.append(measured_position.copy())
                node.publish_task_visualization(
                    measured_position,
                    measured_rotation,
                    publish_markers=(index % 10 == 0),
                )

                _, measured_q, measured_qd = node.wait_for_fresh_joint_state()

                actual_ee = robot.fk_ee(measured_q)
                actual_tibia = actual_ee @ attachment

                reference_q = np.array(
                    [
                        np.interp(
                            task_time,
                            time_q,
                            q_reference[:, joint],
                        )
                        for joint in range(7)
                    ]
                )

                joint_room = np.minimum(
                    measured_q - robot.position_lower,
                    robot.position_upper - measured_q,
                )

                limiting_joint = int(np.argmin(joint_room))

                position_error = np.linalg.norm(
                    measured_position - reference_position
                )
                rotation_error = (
                    measured_rotation * reference_rotation.inv()
                ).magnitude()

                reference_age = (
                    node.get_clock().now().nanoseconds * 1e-9
                    - measured_stamp
                )

                faults = []

                if position_error > ABORT_POSITION_ERROR_M:
                    faults.append("position_error")

                if reference_age > REFERENCE_MAX_AGE_S:
                    faults.append("stale_reference")

                if not np.isfinite(node.commanded_torque).all():
                    faults.append("invalid_commanded_torque")

                if not np.isfinite(node.secondary_torque).all():
                    faults.append("invalid_secondary_torque")

                phase = envelope.phases_at(task_time)[0]

                writer.writerow([
                    node.get_clock().now().nanoseconds * 1e-9,
                    task_time,
                    phase["id"],
                    phase["kind"],
                    *reference_position,
                    *reference_rotation.as_quat(),
                    *twist,
                    *measured_position,
                    *measured_rotation.as_quat(),
                    *actual_tibia[:3, 3],
                    *Rotation.from_matrix(
                        actual_tibia[:3, :3]
                    ).as_quat(),
                    *measured_q,
                    *measured_qd,
                    *reference_q,
                    *joint_room,
                    limiting_joint,
                    *node.commanded_torque,
                    *node.secondary_torque,
                    reference_age,
                    ";".join(faults),
                ])

                if position_error > ABORT_POSITION_ERROR_M:
                    if abort_start is None:
                        abort_start = node.get_clock().now()

                    abort_elapsed = (
                        node.get_clock().now() - abort_start
                    ).nanoseconds * 1e-9

                    if abort_elapsed >= ABORT_DURATION_S:
                        raise RuntimeError(
                            "Position error exceeded abort threshold."
                        )
                else:
                    abort_start = None

                if task_time >= duration_s:
                    break

                time.sleep(sample_period)

        node.publish_zero_twist()

        final_hold_start = node.get_clock().now()

        while rclpy.ok():
            elapsed = (
                node.get_clock().now() - final_hold_start
            ).nanoseconds * 1e-9

            if elapsed >= FINAL_HOLD_S:
                break

            time.sleep(sample_period)

        append_event(
            run_dir / "events.jsonl",
            {
                "event": "baseline_replay_completed",
                "duration_s": duration_s,
            },
        )

        write_json(
            run_dir / "baseline_report.json",
            {
                "schema": "knee_selected_plan_baseline_report_v1",
                "status": "completed",
                "duration_s": duration_s,
                "design_id": plan["design_id"],
                "secondary_control": "off",
                "csv": str(csv_path),
            },
        )

        print(f"Baseline replay completed: {run_dir}")

    except Exception as error:
        append_event(
            run_dir / "events.jsonl",
            {
                "event": "run_failed",
                "error": str(error),
            },
        )

        write_json(
            run_dir / "baseline_report.json",
            {
                "schema": "knee_selected_plan_baseline_report_v1",
                "status": "failed",
                "error": str(error),
            },
        )

        raise

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()