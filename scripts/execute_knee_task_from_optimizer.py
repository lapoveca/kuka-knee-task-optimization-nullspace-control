#!/usr/bin/env python3
"""
execute_knee_task_from_optimizer.py

Thin adapter: optimizer output -> CIC+NS execution.

Three-stage pipeline:
  Stage 1: JIC homing to the optimizer's required start configuration q[0]
  Stage 2: CIC (with null-space) executes the task-envelope EE reference
  Stage 3: Logging + comparison against the optimizer's prediction

The optimizer and CIC remain independently usable:
  - Optimizer: produces path NPZ, analyzable without the controller
  - CIC: accepts any streamed EE reference (e.g., the 60 s flexion test)
  - This adapter: translates optimizer output into CIC's streaming interface

Usage:
  python3 execute_knee_task_from_optimizer.py \
      --path-dir <path_XX> \
      --envelope-dir <envelope_dir> \
      --ns-config jl

  python3 execute_knee_task_from_optimizer.py \
      --path-dir <path_XX> \
      --envelope-dir <envelope_dir> \
      --precompute-only

  python3 execute_knee_task_from_optimizer.py \
      --path-dir <path_XX> \
      --envelope-dir <envelope_dir> \
      --skip-homing   # if robot is already at q[0]
"""
import argparse
import csv
import json
import os
import sys
import time
import numpy as np
from copy import deepcopy
from scipy.spatial.transform import Rotation as R, Slerp

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RCLParameter, ParameterType
from controller_manager_msgs.srv import SwitchController, ListControllers
from lbr_impedance_control.action import JointTrajectoryExecution
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


from task_envelope import TaskEnvelope
from pathlib import Path
from view_homing_target import load_robot_module

# ---------------------------------------------------------------- config
JIC = "joint_impedance_controller"
CIC = "cartesian_impedance_controller"
NS = "/lbr/cartesian_impedance_controller"
FRAME = "lbr_link_0"
JOINTS = [f"lbr_A{i}" for i in range(1, 8)]

JIC_ACTION = "/lbr/joint_impedance_controller/joint_trajectory"
JIC_PARAMS = "/lbr/joint_impedance_controller/set_parameters"
CIC_PARAMS = f"{NS}/set_parameters"
CIC_REF_TOPIC = f"{NS}/reference_cartesian_pose"
CIC_TWIST_TOPIC = f"{NS}/reference_cartesian_twist"
CIC_POSE_TOPIC = f"{NS}/cartesian_pose"
CIC_TAU_TOPIC = f"{NS}/commanded_torque"

JIC_STIFFNESS = 300.0
JIC_VELOCITY = 0.05

CIC_KT = 500.0
CIC_KR = 50.0
CIC_ZETA = 1.0

RATE_HZ = 50.0
HOLD_S = 2.0

ABORT_ERR_M = 0.08
ABORT_AFTER_S = 2.0

LOG_CSV = "knee_task_optimizer_log.csv"


# ---------------------------------------------------------------- NS presets
# Copied exactly from the working execute_knee_flex_cic_tibia.py
NS_PRESETS = {
    "baseline": dict(enabled=False, jl=0.0, tm=0.0, sing=0.0, directional=None),
    "ns_only": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0, directional=None),
    "jl": dict(enabled=True, jl=100.0, tm=0.0, sing=0.0, directional=None),
    "tm": dict(enabled=True, jl=0.0, tm=500.0, sing=0.0, directional=None),
    "sing": dict(enabled=True, jl=0.0, tm=0.0, sing=10.0, directional=None),
    "jl_tm": dict(enabled=True, jl=100.0, tm=500.0, sing=0.0, directional=None),
    "jl_sing": dict(enabled=True, jl=100.0, tm=0.0, sing=1.0, directional=None),
    "tm_sing": dict(enabled=True, jl=0.0, tm=500.0, sing=1.0, directional=None),
    "jl_tm_sing": dict(enabled=True, jl=100.0, tm=500.0, sing=1.0, directional=None),
    "vel_x": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                  directional=("linear_velocity", [1.0, 0.0, 0.0], 1000000.0)),
    "vel_y": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                  directional=("linear_velocity", [0.0, 1.0, 0.0], 10.0)),
    "vel_z": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                  directional=("linear_velocity", [0.0, 0.0, 1.0], 1000000.0)),
    "ang_vel_x": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                      directional=("angular_velocity", [1.0, 0.0, 0.0], 100.0)),
    "ang_vel_y": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                      directional=("angular_velocity", [0.0, 1.0, 0.0], 1000000.0)),
    "ang_vel_z": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                      directional=("angular_velocity", [0.0, 0.0, 1.0], 100.0)),
    "force_x": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                    directional=("force", [1.0, 0.0, 0.0], 1000000.0)),
    "force_y": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                    directional=("force", [0.0, 1.0, 0.0], 1000000.0)),
    "force_z": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                    directional=("force", [0.0, 0.0, 1.0], 1000000.0)),
    "moment_x": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                     directional=("moment", [1.0, 0.0, 0.0], 1000000.0)),
    "moment_y": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                     directional=("moment", [0.0, 1.0, 0.0], 1000.0)),
    "moment_z": dict(enabled=True, jl=0.0, tm=0.0, sing=0.0,
                     directional=("moment", [0.0, 0.0, 1.0], 1000000.0)),
    "jl_tm_fx": dict(enabled=True, jl=100.0, tm=500.0, sing=0.0,
                     directional=("force", [1.0, 0.0, 0.0], 1.0)),
    "sing_vel_z": dict(enabled=True, jl=0.0, tm=0.0, sing=10.0,
                       directional=("linear_velocity", [0.0, 0.0, 1.0], 1.0)),
}

NS_DAMPING = 1.0
NS_MAX_TORQUE = 10.0
NS_SOFT_RATIO = 0.70
NS_HEADROOM_SCALING = False
NS_SINGULARITY_FD_STEP = 1e-3
NS_SINGULARITY_EPSILON = 1e-8
NS_SINGULARITY_LENGTH_SCALE = 0.35
NS_DIRECTIONAL_FD_STEP = 1e-3


# ---------------------------------------------------------------- helpers
def param(name, value, ptype=None):
    p = RCLParameter()
    p.name = name
    if ptype == "double_array":
        p.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        p.value.double_array_value = [float(v) for v in value]
    elif ptype == "string_array":
        p.value.type = ParameterType.PARAMETER_STRING_ARRAY
        p.value.string_array_value = [str(v) for v in value]
    elif isinstance(value, bool):
        p.value.type = ParameterType.PARAMETER_BOOL
        p.value.bool_value = value
    elif isinstance(value, str):
        p.value.type = ParameterType.PARAMETER_STRING
        p.value.string_value = value
    elif isinstance(value, (list, tuple)):
        if all(isinstance(item, str) for item in value):
            p.value.type = ParameterType.PARAMETER_STRING_ARRAY
            p.value.string_array_value = list(value)
        else:
            p.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            p.value.double_array_value = [float(item) for item in value]
    else:
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(value)
    return p


def ns_params(cfg_name):
    """Exact copy from working execute_knee_flex_cic_tibia.py."""
    if cfg_name not in NS_PRESETS:
        raise ValueError(f"Unknown NS config '{cfg_name}'. Available: {list(NS_PRESETS)}")
    cfg = NS_PRESETS[cfg_name]
    directional = cfg.get("directional")
    if directional is None:
        directional_types = []
        directional_weights = []
        directional_directions = []
    else:
        objective_type, direction, weight = directional
        directional_types = [objective_type]
        directional_weights = [float(weight)]
        directional_directions = [float(v) for v in direction]
    if len(directional_directions) != 3 * len(directional_types):
        raise ValueError(
            f"Invalid directional preset '{cfg_name}': expected three direction values per objective.")
    return [
        param("ns_weight_joint_limit", cfg["jl"]),
        param("ns_weight_torque_min", cfg["tm"]),
        param("ns_weight_singularity", cfg["sing"]),
        param("ns_damping", NS_DAMPING),
        param("ns_headroom_scaling", NS_HEADROOM_SCALING),
        param("ns_torque_soft_ratio", NS_SOFT_RATIO),
        param("ns_max_torque", NS_MAX_TORQUE),
        param("ns_singularity_fd_step", NS_SINGULARITY_FD_STEP),
        param("ns_singularity_epsilon", NS_SINGULARITY_EPSILON),
        param("ns_singularity_length_scale", NS_SINGULARITY_LENGTH_SCALE),
        param("ns_directional_fd_step", NS_DIRECTIONAL_FD_STEP),
        param("ns_directional_types", directional_types, ptype="string_array"),
        param("ns_directional_weights", directional_weights, ptype="double_array"),
        param("ns_directional_directions_ee", directional_directions, ptype="double_array"),
        param("ns_enabled", cfg["enabled"]),
    ]


# ---------------------------------------------------------------- reference
def build_ee_reference(env, X_BF, A, rate):
    """Generate dense EE reference using the envelope's own validated composition."""
    t_dense = np.arange(env.time_s[0], env.time_s[-1] + 1.0 / rate, 1.0 / rate)
    # Clamp final sample to exactly the envelope end (avoid overshoot)
    if t_dense[-1] > env.time_s[-1]:
        t_dense[-1] = env.time_s[-1]
    n = len(t_dense)
    ee_pos = np.zeros((n, 3))
    ee_quat = np.zeros((n, 4))
    for i, t in enumerate(t_dense):
        X_BE = env.required_ee(float(t), X_BF, A)
        ee_pos[i] = X_BE[:3, 3]
        ee_quat[i] = R.from_matrix(X_BE[:3, :3]).as_quat()
    return t_dense, ee_pos, ee_quat


def interpolate_angles(env, t_dense):
    """Interpolate anatomical angles to the dense grid (for CSV logging)."""
    n = len(t_dense)
    angles = np.zeros((n, 3))
    for j in range(3):
        angles[:, j] = np.interp(t_dense, env.time_s, env.angles_rad[:, j])
    return angles


# ---------------------------------------------------------------- node
class TaskExecutor(Node):
    def __init__(self):
        super().__init__("execute_knee_task_optimizer")
        self.jic_action = ActionClient(self, JointTrajectoryExecution, JIC_ACTION)
        self.jic_params = self.create_client(SetParameters, JIC_PARAMS)
        self.cic_params = self.create_client(SetParameters, CIC_PARAMS)
        self.cic_set_ref = self.create_client(Trigger, f"{NS}/set_reference_to_current")
        self.ref_pub = self.create_publisher(PoseStamped, CIC_REF_TOPIC, 1)
        self.twist_pub = self.create_publisher(TwistStamped, CIC_TWIST_TOPIC, 1)
        self.list_ctrl = self.create_client(
            ListControllers, "/lbr/controller_manager/list_controllers")
        self.switch_ctrl = self.create_client(
            SwitchController, "/lbr/controller_manager/switch_controller")
        self.js = None
        self.pose = None
        self.tau = np.full(7, np.nan)
        self.create_subscription(
            JointState, "/lbr/joint_states", lambda m: setattr(self, "js", m), 10)
        self.create_subscription(
            PoseStamped, CIC_POSE_TOPIC, lambda m: setattr(self, "pose", m), 10)
        self.create_subscription(
            Float64MultiArray, CIC_TAU_TOPIC, lambda m: setattr(self, "tau", np.array(m.data)), 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "/knee_task_visualization", 10)
        self.ref_traj_published = False
        self.homing_marker_pub = self.create_publisher(MarkerArray, "/knee_task/homing_target", 1)
        self._homing_timer = None
        self._homing_positions = None

    def call(self, client, req):
        client.wait_for_service()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def ee_pose(self):
        self.pose = None
        t0 = time.monotonic()
        while self.pose is None and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pose is None:
            raise RuntimeError("No CIC cartesian_pose received.")
        p, q = self.pose.pose.position, self.pose.pose.orientation
        return (np.array([p.x, p.y, p.z]),
                R.from_quat([q.x, q.y, q.z, q.w]))

    def switch(self, activate, deactivate):
        res = self.call(self.list_ctrl, ListControllers.Request())
        st = {c.name: c.state for c in res.controller}
        act = [] if st.get(activate) == "active" else [activate]
        deact = [deactivate] if st.get(deactivate) == "active" else []
        if not act and not deact:
            return True
        req = SwitchController.Request()
        req.activate_controllers = act
        req.deactivate_controllers = deact
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = 5
        r = self.call(self.switch_ctrl, req)
        return r is not None and r.ok

    def set_params(self, client, plist):
        req = SetParameters.Request()
        req.parameters = plist
        res = self.call(client, req)
        for p, r in zip(plist, res.results):
            if not r.successful:
                self.get_logger().warning(f"Param {p.name} rejected: {r.reason}")

    def lock_ref(self):
        r = self.call(self.cic_set_ref, Trigger.Request())
        if r is None or not r.success:
            raise RuntimeError("set_reference_to_current failed")

    def publish_ref(self, pos, rot):
        q = rot.as_quat()
        m = PoseStamped()
        m.header.frame_id = FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
        m.pose.orientation.x, m.pose.orientation.y = float(q[0]), float(q[1])
        m.pose.orientation.z, m.pose.orientation.w = float(q[2]), float(q[3])
        self.ref_pub.publish(m)

    def publish_twist(self, linear_velocity, angular_velocity):
        msg = TwistStamped()
        msg.header.frame_id = FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_velocity[0])
        msg.twist.linear.y = float(linear_velocity[1])
        msg.twist.linear.z = float(linear_velocity[2])
        msg.twist.angular.x = float(angular_velocity[0])
        msg.twist.angular.y = float(angular_velocity[1])
        msg.twist.angular.z = float(angular_velocity[2])
        self.twist_pub.publish(msg)

    # ---------------------------------------------------- Stage 1: JIC homing
    def home_to_q0(self, q0):
        self.get_logger().info("Stage 1: JIC homing to optimizer's q[0]")
        if not self.switch(JIC, CIC):
            raise RuntimeError("Could not activate JIC.")
        time.sleep(0.3)

        p = param("stiffness", JIC_STIFFNESS)
        req = SetParameters.Request()
        req.parameters = [p]
        self.call(self.jic_params, req)

        self.jic_action.wait_for_server()
        goal = JointTrajectoryExecution.Goal()
        goal.joint_positions_goal = Float64MultiArray(data=q0.tolist())
        goal.joint_velocities_goal = Float64MultiArray(data=[JIC_VELOCITY] * 7)

        for attempt in (1, 2, 3):  # 3 attempts now
            fut = self.jic_action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().error("JIC goal rejected.")
                continue
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut)
            if bool(res_fut.result().result.success):
                self.get_logger().info(f"JIC homing (try {attempt}): SUCCESS")
                # Settle check: wait for joint velocity to be near zero
                self.get_logger().info("Settling (waiting for velocity to decay)...")
                time.sleep(2.0)  # let transients decay
                # Read joint state and log
                t0 = time.monotonic()
                while self.js is None and time.monotonic() - t0 < 5.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                if self.js is not None:
                    vel = np.array(self.js.velocity)
                    pos = np.array(self.js.position)
                    max_vel = np.max(np.abs(vel))
                    pos_err = np.max(np.abs(pos - q0))
                    self.get_logger().info(
                        f"Post-JIC state: max |vel| = {max_vel:.4f} rad/s, "
                        f"max |pos - q0| = {pos_err:.4f} rad "
                        f"({np.degrees(pos_err):.2f} deg)")
                    if max_vel > 0.01:
                        self.get_logger().warning(
                            f"Joint velocity still {max_vel:.4f} rad/s — "
                            f"waiting 2 more seconds...")
                        time.sleep(2.0)
                        # Re-read
                        t0 = time.monotonic()
                        while self.js is None and time.monotonic() - t0 < 5.0:
                            rclpy.spin_once(self, timeout_sec=0.05)
                        if self.js is not None:
                            vel = np.array(self.js.velocity)
                            max_vel = np.max(np.abs(vel))
                            self.get_logger().info(
                                f"Re-check: max |vel| = {max_vel:.4f} rad/s")
                return True
            self.get_logger().warning(f"JIC homing (try {attempt}): FAILED")
            time.sleep(1.0)
        return False

    def prepare_homing_visualization(self, q_start):
        """Compute the target skeleton before issuing any motion."""
        root = Path(__file__).resolve().parents[1]
        module = load_robot_module(root)
        robot = module.RobotModel(
            root / "optimizer" / "models" / "med14_runtime.urdf"
        )

        if module.BASE_FRAME != FRAME:
            raise ValueError(
                f"Skeleton base frame {module.BASE_FRAME!r} "
                f"does not match executor frame {FRAME!r}"
            )

        poses = robot.frame_poses(np.asarray(q_start, dtype=float))
        self._homing_positions = np.array([
            poses[name][:3, 3]
            for name in robot.chain_frames
        ])

    def _homing_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = FRAME
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
    def _visual_point(values):
        return Point(
            x=float(values[0]),
            y=float(values[1]),
            z=float(values[2]),
        )

    def publish_homing_visualization(self):
        if self._homing_positions is None:
            return

        positions = self._homing_positions

        skeleton = self._homing_marker(0, Marker.LINE_STRIP)
        skeleton.scale.x = 0.009
        skeleton.points = [
            self._visual_point(p) for p in positions
        ]

        joints = self._homing_marker(1, Marker.SPHERE_LIST)
        joints.scale.x = 0.016
        joints.scale.y = 0.016
        joints.scale.z = 0.016
        joints.points = [
            self._visual_point(p) for p in positions[1:-1]
        ]

        label = self._homing_marker(2, Marker.TEXT_VIEW_FACING)
        label.pose.position = self._visual_point(
            positions[-1] + np.array([0.0, 0.0, 0.07])
        )
        label.scale.z = 0.025
        label.text = "Target start posture"

        self.homing_marker_pub.publish(
            MarkerArray(markers=[skeleton, joints, label])
        )

    def show_homing_visualization(self):
        if self._homing_positions is None:
            raise RuntimeError("Homing visualization was not prepared")

        self.publish_homing_visualization()

        if self._homing_timer is None:
            self._homing_timer = self.create_timer(
                0.5, self.publish_homing_visualization
            )

    def hide_homing_visualization(self):
        if self._homing_timer is not None:
            self._homing_timer.cancel()
            self.destroy_timer(self._homing_timer)
            self._homing_timer = None

        markers = []
        for marker_id in (0, 1, 2):
            marker = self._homing_marker(
                marker_id, Marker.LINE_STRIP
            )
            marker.action = Marker.DELETE
            markers.append(marker)

        self.homing_marker_pub.publish(
            MarkerArray(markers=markers)
        )

    # ---------------------------------------------------- Stage 2: CIC execution
    def execute_task(self, ns_cfg_name, t_dense, pos_dense, quat_dense,
                     angles_dense, report, X_BF, A):
        self.get_logger().info("Stage 2: CIC execution of task-envelope reference")
        if not self.switch(CIC, JIC):
            raise RuntimeError("Could not activate CIC.")
        time.sleep(0.3)

        # CIC stiffness / damping
        plist = [
            param("translational_stiffness", CIC_KT),
            param("rotational_stiffness", CIC_KR),
            param("translational_damping_ratio", CIC_ZETA),
            param("rotational_damping_ratio", CIC_ZETA),
        ]
        # Null-space parameters (modular, all sent every run)
        plist += ns_params(ns_cfg_name)
        self.set_params(self.cic_params, plist)

        # Lock reference to current pose, then validate start
        self.lock_ref()
        time.sleep(0.5)
        cur_pos, cur_rot = self.ee_pose()
        ref0_pos = pos_dense[0]
        ref0_rot = R.from_quat(quat_dense[0])
        pos_err0 = np.linalg.norm(cur_pos - ref0_pos)
        rot_err0 = (cur_rot * ref0_rot.inv()).magnitude()
        self.get_logger().info(
            f"Start-pose check: {pos_err0*1000:.2f} mm / "
            f"{np.degrees(rot_err0):.2f} deg")
        if pos_err0 > 0.02:
            raise RuntimeError(
                f"Start EE position {pos_err0*1000:.1f} mm from reference start "
                f"(tolerance 20 mm). Check JIC homing.")
        if rot_err0 > np.radians(5.0):
            raise RuntimeError(
                f"Start EE orientation {np.degrees(rot_err0):.1f} deg from "
                f"reference start (tolerance 5 deg). Check JIC homing.")

        self.hide_homing_visualization()

        self._last_ref_pos = pos_dense[0]
        self._last_ref_rot = R.from_quat(quat_dense[0])
        self._X_BF = X_BF  # store for TF publishing
        self._A = A  # store for TF publishing
        self._ref_pos_dense = pos_dense
        self._actual_points = []

        # Scheduled streaming loop
        n = len(t_dense)
        total = n + int(HOLD_S * RATE_HZ)
        period = 1.0 / RATE_HZ
        next_tick = time.monotonic()
        bad = 0
        errs = []
        status = "Done."
        log_csv = LOG_CSV

        with open(log_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "flexion_deg", "e_pos_m", "e_rot_rad",
                        "ref_x", "ref_y", "ref_z", "ee_x", "ee_y", "ee_z",
                        *[f"tau{i}" for i in range(1, 8)]])
            for k in range(1, total + 1):
                idx = min(k - 1, n - 1)
                pos = pos_dense[idx]
                rot = R.from_quat(quat_dense[idx])
                fl = angles_dense[idx, 0]

                # Zero twist during hold, computed during motion
                if k <= n and idx > 0:
                    h = 1
                    i_minus = max(0, idx - h)
                    i_plus = min(n - 1, idx + h)
                    dt = t_dense[i_plus] - t_dense[i_minus]
                    if dt > 1e-9:
                        dpos = (pos_dense[i_plus] - pos_dense[i_minus]) / dt
                        r_minus = R.from_quat(quat_dense[i_minus])
                        r_plus = R.from_quat(quat_dense[i_plus])
                        drot = (r_plus * r_minus.inv()).as_rotvec() / dt
                    else:
                        dpos = np.zeros(3)
                        drot = np.zeros(3)
                    linear_velocity = dpos
                    angular_velocity = drot
                else:
                    linear_velocity = np.zeros(3)
                    angular_velocity = np.zeros(3)

                # Publish reference
                self.publish_ref(pos, rot)
                self.publish_twist(linear_velocity, angular_velocity)

                # Update last ref for TF publishing
                self._last_ref_pos = pos
                self._last_ref_rot = rot

                # Read feedback ONCE per cycle
                rclpy.spin_once(self, timeout_sec=0.0)
                if self.pose is not None:
                    ee_p, ee_r = self.ee_pose()

                    # Accumulate actual EE position for the trail
                    self._actual_points.append(ee_p)

                    # Republish visualization every 10 cycles (0.2 s at 50 Hz)
                    if k % 10 == 0:
                        self.publish_visualization(ee_p, ee_r)


                    # Error check
                    e_pos = np.linalg.norm(ee_p - pos)
                    e_rot = (ee_r * rot.inv()).magnitude()
                    errs.append(e_pos)
                    if e_pos > ABORT_ERR_M:
                        bad += 1
                        if bad * period > ABORT_AFTER_S:
                            status = (f"Aborted: position error exceeded "
                                      f"{ABORT_ERR_M*1000:.0f} mm for "
                                      f"{ABORT_AFTER_S:.0f} s.")
                            break
                    else:
                        bad = 0
                    w.writerow([t_dense[idx], np.degrees(fl), e_pos, e_rot,
                                *pos, *ee_p, *self.tau])

                # Scheduled tick
                next_tick += period
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)

        # Summary vs optimizer prediction
        opt_err = report["max_ee_error_mm"] / 1000.0
        self.get_logger().info(
            f"{status} max err {max(errs)*1000:.1f} mm, "
            f"mean {np.mean(errs)*1000:.1f} mm, "
            f"final {errs[-1]*1000:.1f} mm. "
            f"Optimizer predicted max: {opt_err*1000:.1f} mm. "
            f"Log: {log_csv}")

    def publish_visualization(self, actual_pos, actual_rot):
        """Publish reference trajectory + actual trail + tibia marker.

        Called periodically during execution so RViz (subscribing at any
        time) always receives a current, complete visualization.
        """
        now = self.get_clock().now().to_msg()

        # --- TF frames  ---
        t = TransformStamped()
        t.header.frame_id = FRAME
        t.header.stamp = now
        t.child_frame_id = "ref_ee"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            map(float, self._last_ref_pos)
        q = self._last_ref_rot.as_quat()
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(t)

        t.child_frame_id = "actual_ee"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            map(float, actual_pos)
        q = actual_rot.as_quat()
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(t)

        tibia_pos, tibia_rot = self.compose_transform(actual_pos, actual_rot, self._A)
        t.child_frame_id = "tibia"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            map(float, tibia_pos)
        q = tibia_rot.as_quat()
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(t)

        t.child_frame_id = "placement"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            map(float, self._X_BF[:3, 3])
        q = R.from_matrix(self._X_BF[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(t)

        # --- Markers ---

        markers = []

        # 1. Reference trajectory (blue) — decimated for performance
        ref = Marker()
        ref.header.frame_id = FRAME
        ref.header.stamp = now
        ref.ns = "reference_trajectory"
        ref.id = 0
        ref.type = Marker.LINE_STRIP
        ref.action = Marker.ADD
        ref.pose.orientation.w = 1.0
        ref.scale.x = 0.004
        ref.color.b = 1.0
        ref.color.a = 0.8
        for p in self._ref_pos_dense[::20]:  # 13001 -> ~650 points
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
            ref.points.append(pt)
        markers.append(ref)

        # 2. Actual EE trail (red) — grows during execution
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
        for p in self._actual_points:
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
            trail.points.append(pt)
        markers.append(trail)

        # 3. Tibia origin (green sphere) — the knee/task reference point
        tibia_pos, _ = self.compose_transform(
            actual_pos, actual_rot, self._A)
        sphere = Marker()
        sphere.header.frame_id = FRAME
        sphere.header.stamp = now
        sphere.ns = "tibia_origin"
        sphere.id = 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(tibia_pos[0])
        sphere.pose.position.y = float(tibia_pos[1])
        sphere.pose.position.z = float(tibia_pos[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.014
        sphere.scale.y = 0.014
        sphere.scale.z = 0.014
        sphere.color.g = 1.0
        sphere.color.a = 0.9
        markers.append(sphere)


        # Schematic tibia: declared task origin -> distal EE attachment.
        tibia_pos, tibia_rot = self.compose_transform(
            actual_pos, actual_rot, self._A
        )

        X_TE = np.linalg.inv(self._A)
        distal_T = X_TE[:3, 3]
        length = float(np.linalg.norm(distal_T))
        if not np.isfinite(length) or length <= 1e-9:
            raise ValueError("Invalid tibia-to-EE attachment offset")

        axis_T = distal_T / length

        # Construct a cylinder orientation whose local z follows axis_T.
        helper = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(axis_T, helper))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])

        transverse_T = np.cross(helper, axis_T)
        transverse_T /= np.linalg.norm(transverse_T)
        second_T = np.cross(axis_T, transverse_T)

        cylinder_R_T = np.column_stack(
            [transverse_T, second_T, axis_T]
        )
        cylinder_rot_B = tibia_rot * R.from_matrix(cylinder_R_T)
        center_B = tibia_pos + tibia_rot.apply(0.5 * distal_T)

        body = Marker()
        body.header.frame_id = FRAME
        body.header.stamp = now
        body.ns = "tibia_body"
        body.id = 10
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = float(center_B[0])
        body.pose.position.y = float(center_B[1])
        body.pose.position.z = float(center_B[2])

        quat = cylinder_rot_B.as_quat()
        body.pose.orientation.x = float(quat[0])
        body.pose.orientation.y = float(quat[1])
        body.pose.orientation.z = float(quat[2])
        body.pose.orientation.w = float(quat[3])

        body.scale.x = 0.035
        body.scale.y = 0.035
        body.scale.z = length
        body.color.r = 0.65
        body.color.g = 0.85
        body.color.b = 0.70
        body.color.a = 0.95
        markers.append(body)
 
        # Longitudinal surface stripe for visible axial rotation
        # Small longitudinal rib, slightly embedded in the tibia body.
        rib = Marker()
        rib.header.frame_id = FRAME
        rib.header.stamp = now
        rib.ns = "tibia_rotation_indicator"
        rib.id = 11
        rib.type = Marker.CYLINDER
        rib.action = Marker.ADD

        body_radius = 0.5 * body.scale.x
        rib_radius = 0.004

        # Centre on the body's nominal surface:
        # half the rib is embedded, half protrudes.
        center_T = (
            0.5 * distal_T
            + body_radius * transverse_T
        )
        center_B = tibia_pos + tibia_rot.apply(center_T)

        rib.pose.position.x = float(center_B[0])
        rib.pose.position.y = float(center_B[1])
        rib.pose.position.z = float(center_B[2])

        quat = cylinder_rot_B.as_quat()
        rib.pose.orientation.x = float(quat[0])
        rib.pose.orientation.y = float(quat[1])
        rib.pose.orientation.z = float(quat[2])
        rib.pose.orientation.w = float(quat[3])

        rib.scale.x = 2.0 * rib_radius
        rib.scale.y = 2.0 * rib_radius
        rib.scale.z = 0.84 * length

        rib.color.r = 0.18
        rib.color.g = 0.23
        rib.color.b = 0.28
        rib.color.a = 1.0
        markers.append(rib)

        opposite_rib = deepcopy(rib)
        opposite_rib.id = 12

        opposite_center_T = (
            0.5 * distal_T
            - body_radius * transverse_T
        )
        opposite_center_B = tibia_pos + tibia_rot.apply(
            opposite_center_T
        )

        opposite_rib.pose.position.x = float(opposite_center_B[0])
        opposite_rib.pose.position.y = float(opposite_center_B[1])
        opposite_rib.pose.position.z = float(opposite_center_B[2])

        markers.append(opposite_rib)

        self.marker_pub.publish(MarkerArray(markers=markers))

    def publish_actual_trajectory_point(self, actual_pos, actual_rot):
        """Publish TF frames each cycle (uses self._A and self._X_BF)."""
        now = self.get_clock().now().to_msg()

        # TF: ref_ee (reference EE pose)
        t = TransformStamped()
        t.header.frame_id = FRAME
        t.header.stamp = now
        t.child_frame_id = "ref_ee"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            map(float, self._last_ref_pos)
        q = self._last_ref_rot.as_quat()
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(t)

        # TF: actual_ee (actual EE pose from CIC)
        t.child_frame_id = "actual_ee"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            map(float, actual_pos)
        q = actual_rot.as_quat()
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(t)

        # TF: tibia = actual_ee @ A (attachment transform)
        tibia_pos, tibia_rot = self.compose_transform(actual_pos, actual_rot, self._A)
        t.child_frame_id = "tibia"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            map(float, tibia_pos)
        q = tibia_rot.as_quat()
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(t)

        # TF: placement (X_BF, fixed)
        t.child_frame_id = "placement"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            map(float, self._X_BF[:3, 3])
        q = R.from_matrix(self._X_BF[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(t)

    def compose_transform(self, pos, rot, A):
        """Compose actual_ee @ A to get tibia pose."""
        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = pos
        T_tibia = T @ A
        tibia_pos = T_tibia[:3, 3]
        tibia_rot = R.from_matrix(T_tibia[:3, :3])
        return tibia_pos, tibia_rot




# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path-dir", required=True,
                    help="Optimizer's saved path folder (contains whole_trajectory_path.npz)")
    ap.add_argument("--envelope-dir", required=True,
                    help="Task envelope folder (contains task.json + task_arrays.npz)")
    ap.add_argument("--ns-config", default="baseline")
    ap.add_argument("--rate", type=int, default=50)
    ap.add_argument("--precompute-only", action="store_true")
    ap.add_argument("--skip-homing", action="store_true",
                    help="Skip JIC homing (assume robot already at q[0])")
    args = ap.parse_args()

    rclpy.init()
    node = TaskExecutor()
    try:
        # Load and validate envelope
        env = TaskEnvelope(args.envelope_dir)
        node.get_logger().info(
            f"Loaded envelope: {env.meta['task_id']} | "
            f"duration: {env.time_s[-1] - env.time_s[0]:.1f} s | "
            f"samples: {len(env.time_s)}")

        # Load path NPZ
        npz_path = os.path.join(args.path_dir, "whole_trajectory_path.npz")
        z = np.load(npz_path)
        q = z["q"]
        A = z["A"]
        X_BF = z["X_BF"]
        node.get_logger().info(
            f"Loaded path: {args.path_dir} | q shape: {q.shape}")

        # Check / build EE cache
        cache = os.path.join(args.path_dir, "ee_trajectory.npz")
        if os.path.exists(cache):
            node.get_logger().info(f"EE cache found: {cache}")
            zc = np.load(cache)
            t_dense = zc["time_s"]
            pos_dense = zc["ee_pos_base"]
            quat_dense = zc["ee_quat_base"]
        else:
            node.get_logger().info("Building EE reference (dense grid)...")
            t_dense, pos_dense, quat_dense = build_ee_reference(
                env, X_BF, A, args.rate)
            np.savez(cache, time_s=t_dense, ee_pos_base=pos_dense,
                     ee_quat_base=quat_dense)
            node.get_logger().info(f"EE cache saved: {cache}")

        if args.precompute_only:
            node.get_logger().info("Precompute-only: exiting.")
            return

        # Interpolate angles for CSV
        angles_dense = interpolate_angles(env, t_dense)

        # Load report for validation baseline
        report_path = os.path.join(args.path_dir, "report.json")
        report = json.load(open(report_path))

        # Stage 1: JIC homing (optional)
        node.prepare_homing_visualization(q[0])
        node.show_homing_visualization()
        if not args.skip_homing:
            if not node.home_to_q0(q[0]):
                node.get_logger().error("JIC homing failed. Aborting.")
                return
        else:
            node.get_logger().info("Skipping JIC homing (--skip-homing).")

        # Stage 2: CIC execution
        node.execute_task(args.ns_config, t_dense, pos_dense, quat_dense,
                          angles_dense, report, X_BF, A)
    finally:
        if rclpy.ok():
            node.hide_homing_visualization()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()