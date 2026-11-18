"""Reliable JIC initialization with fresh measured settling."""

import csv
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .run_recording import write_json


JOINT_NAMES = tuple(f"lbr_A{i}" for i in range(1, 8))


def wait_future(node, future, timeout, description):
    deadline = time.monotonic() + timeout

    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Timed out: {description}")
        rclpy.spin_once(node, timeout_sec=min(0.02, remaining))

    if not future.done():
        raise RuntimeError(f"ROS shutdown while waiting: {description}")

    result = future.result()
    if result is None:
        raise RuntimeError(f"No response: {description}")
    return result


def bounded_call(node, client, request, timeout=8.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError("Required service is unavailable")

    return wait_future(
        node,
        client.call_async(request),
        timeout,
        "service response",
    )


def joint_sample(message):
    if len(message.name) != len(set(message.name)):
        raise ValueError("Duplicate names in JointState")

    indices = {name: i for i, name in enumerate(message.name)}
    missing = [name for name in JOINT_NAMES if name not in indices]
    if missing:
        raise ValueError(f"JointState is missing required joints: {missing}")

    order = [indices[name] for name in JOINT_NAMES]

    if (
        len(message.position) != len(message.name)
        or len(message.velocity) != len(message.name)
    ):
        raise ValueError(
            "JointState position/velocity arrays are incomplete"
        )

    q = np.asarray([message.position[i] for i in order], dtype=float)
    qd = np.asarray([message.velocity[i] for i in order], dtype=float)

    if not np.isfinite(q).all() or not np.isfinite(qd).all():
        raise ValueError("Non-finite JointState")

    stamp = (
        float(message.header.stamp.sec)
        + 1e-9 * float(message.header.stamp.nanosec)
    )
    return stamp, q, qd


def expected_action_timeout(q_current, q_target, velocity, policy):
    q_current = np.asarray(q_current, dtype=float)
    q_target = np.asarray(q_target, dtype=float)

    if q_current.shape != (7,) or q_target.shape != (7,):
        raise ValueError("Expected seven joint positions")

    if not np.isfinite(q_current).all() or not np.isfinite(q_target).all():
        raise ValueError("Non-finite joint positions")

    if velocity <= 0:
        raise ValueError("JIC velocity must be positive")

    displacement = float(
        np.max(np.abs(q_target - q_current))
    )
    trajectory_time = displacement / velocity

    return (
        policy["action_timeout_margin"] * trajectory_time
        + policy["action_timeout_floor_s"]
    )


def wait_for_fresh_joint_state(node, timeout=5.0):
    previous_message = node.js
    deadline = time.monotonic() + timeout

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

        message = node.js
        if message is None or message is previous_message:
            continue

        return joint_sample(message)

    raise TimeoutError("No fresh JointState message received")


def initialize(
    node,
    q_start,
    directory,
    action_type,
    stiffness,
    velocity,
    policy,
):
    q_start = np.asarray(q_start, dtype=float)

    if q_start.shape != (7,) or not np.isfinite(q_start).all():
        raise ValueError("Invalid q_start")

    outcome = {
        "schema": "knee_initialization_result_v1",
        "status": "failed",
        "action_status": None,
        "action_success": None,
        "measured_settled": False,
        "reason": None,
        "requested_stiffness": float(stiffness),
        "requested_joint_speed_rad_s": float(velocity),
        "initial_position_error_rad": None,
        "expected_action_timeout_s": None,
    }

    goal_handle = None
    result_future = None

    try:
        if not node.switch(
            "joint_impedance_controller",
            "cartesian_impedance_controller",
        ):
            raise RuntimeError("Could not activate JIC")

        parameter_request = SetParameters.Request()
        parameter_request.parameters = [
            node.make_parameter("stiffness", stiffness)
        ]

        parameter_response = bounded_call(
            node,
            node.jic_params,
            parameter_request,
        )

        if (
            len(parameter_response.results) != 1
            or not parameter_response.results[0].successful
        ):
            raise RuntimeError("Required JIC stiffness parameter rejected")

        if not node.jic_action.wait_for_server(timeout_sec=8.0):
            raise TimeoutError("JIC action server unavailable")

        _, q_current, _ = wait_for_fresh_joint_state(node)
        outcome["initial_position_error_rad"] = float(
            np.max(np.abs(q_current - q_start))
        )

        action_timeout = expected_action_timeout(
            q_current,
            q_start,
            velocity,
            policy,
        )
        outcome["expected_action_timeout_s"] = action_timeout

        goal = action_type.Goal()
        goal.joint_positions_goal = Float64MultiArray(
            data=q_start.tolist()
        )
        goal.joint_velocities_goal = Float64MultiArray(
            data=[float(velocity)] * 7
        )

        goal_handle = wait_future(
            node,
            node.jic_action.send_goal_async(goal),
            8.0,
            "JIC goal acknowledgement",
        )

        if not goal_handle.accepted:
            raise RuntimeError("JIC goal rejected")

        result_future = goal_handle.get_result_async()

        action_response = wait_future(
            node,
            result_future,
            action_timeout,
            "JIC action result",
        )

        outcome["action_status"] = int(action_response.status)
        outcome["action_success"] = bool(
            action_response.result.success
        )

        previous_message = node.js
        previous_stamp = None
        good_since = None
        latest_position_error = None
        latest_velocity = None

        settling_deadline = (
            time.monotonic()
            + policy["settling_timeout_wall_s"]
        )

        with (directory / "initialization.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "sample_sim_time_s",
                "max_position_error_rad",
                "max_velocity_rad_s",
                "within_thresholds",
                *[f"q_{index}" for index in range(1, 8)],
                *[f"qd_{index}" for index in range(1, 8)],
            ])

            while rclpy.ok() and time.monotonic() < settling_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)

                message = node.js
                if message is None or message is previous_message:
                    continue

                previous_message = message
                stamp, q, qd = joint_sample(message)

                if previous_stamp is not None:
                    sample_delta = stamp - previous_stamp

                    if sample_delta < 0:
                        raise RuntimeError(
                            "Joint-state simulation clock moved backward"
                        )

                    if sample_delta == 0:
                        continue

                    if (
                        sample_delta
                        > policy["maximum_sample_gap_sim_s"]
                    ):
                        good_since = None

                previous_stamp = stamp

                latest_position_error = float(
                    np.max(np.abs(q - q_start))
                )
                latest_velocity = float(np.max(np.abs(qd)))

                within_thresholds = (
                    latest_position_error
                    <= policy["position_tolerance_rad"]
                    and latest_velocity
                    <= policy["velocity_tolerance_rad_s"]
                )

                writer.writerow([
                    stamp,
                    latest_position_error,
                    latest_velocity,
                    int(within_thresholds),
                    *q,
                    *qd,
                ])

                if within_thresholds:
                    if good_since is None:
                        good_since = stamp

                    if (
                        stamp - good_since
                        >= policy["settle_duration_sim_s"]
                    ):
                        outcome["measured_settled"] = True
                        break
                else:
                    good_since = None

        outcome["last_max_position_error_rad"] = latest_position_error
        outcome["last_max_velocity_rad_s"] = latest_velocity

        if not outcome["measured_settled"]:
            raise RuntimeError(
                "Measured settling did not pass within the "
                "wall-clock timeout"
            )

        action_passed = (
            outcome["action_status"] == GoalStatus.STATUS_SUCCEEDED
            and outcome["action_success"] is True
        )

        if not action_passed:
            raise RuntimeError(
                "Measured settling passed, but the JIC action did not "
                "report success; CIC was not activated"
            )

        outcome["status"] = "passed"
        node.get_logger().info(
            "Initialization PASSED: action success and measured settling"
        )
        return True

    except Exception as exc:
        outcome["reason"] = f"{type(exc).__name__}: {exc}"

        if (
            goal_handle is not None
            and goal_handle.accepted
            and result_future is not None
            and not result_future.done()
            and rclpy.ok()
        ):
            try:
                cancellation = wait_future(
                    node,
                    goal_handle.cancel_goal_async(),
                    5.0,
                    "JIC cancellation acknowledgement",
                )
                outcome["cancel_acknowledged"] = bool(
                    cancellation.goals_canceling
                )
            except Exception as cancellation_error:
                outcome["cancel_error"] = str(cancellation_error)

        node.get_logger().error(
            f"Initialization FAILED: {type(exc).__name__}: {exc}"
        )
        return False

    finally:
        write_json(
            directory / "initialization_result.json",
            outcome,
        )