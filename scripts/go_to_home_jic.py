#!/usr/bin/env python3
"""Move to a fixed joint posture with JIC. Leaves JIC active.
Usage: python3 home_jic.py [--yes]
"""
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RCLParameter, ParameterType
from controller_manager_msgs.srv import SwitchController, ListControllers
from lbr_impedance_control.action import JointTrajectoryExecution

# HOME_JOINT_POSITIONS = [0.0, 0.3, 0.0, -1.6, 0.0, -0.5, 0.0]
# HOME_JOINT_POSITIONS = [0.0, 0.576, 0.0, -1.117, 0.0, -1.257, 0.0]
HOME_JOINT_POSITIONS = [0.0, 0.75, 0.0, -1.4, 0.0, -1.257, 0.0]
VELOCITY_RAD_S = 0.20
STIFFNESS_NM_RAD = 300.0
JIC = "joint_impedance_controller"
CIC = "cartesian_impedance_controller"
JOINTS = [f"lbr_A{i}" for i in range(1, 8)]


class Homer(Node):
    def __init__(self):
        super().__init__("home_jic")
        self.action = ActionClient(
            self, JointTrajectoryExecution,
            "/lbr/joint_impedance_controller/joint_trajectory")
        self.params = self.create_client(
            SetParameters, "/lbr/joint_impedance_controller/set_parameters")
        self.list_ctrl = self.create_client(
            ListControllers, "/lbr/controller_manager/list_controllers")
        self.switch_ctrl = self.create_client(
            SwitchController, "/lbr/controller_manager/switch_controller")
        self.js = None
        self.create_subscription(
            JointState, "/lbr/joint_states",
            lambda m: setattr(self, "js", m), 10)

    def _call(self, client, req):
        client.wait_for_service()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def joints(self):
        t0 = time.monotonic()
        while self.js is None and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        m = dict(zip(self.js.name, self.js.position))
        return np.array([m[j] for j in JOINTS])

    def ensure_jic(self):
        res = self._call(self.list_ctrl, ListControllers.Request())
        st = {c.name: c.state for c in res.controller}
        self.get_logger().info(f"Controller states: JIC={st.get(JIC)}, CIC={st.get(CIC)}")
        if st.get(JIC) is None:
            self.get_logger().error("JIC is not loaded. Use the launch that spawns it.")
            return False
        act = [] if st.get(JIC) == "active" else [JIC]
        deact = [CIC] if st.get(CIC) == "active" else []
        if not act and not deact:
            return True
        req = SwitchController.Request()
        req.activate_controllers = act
        req.deactivate_controllers = deact
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = 5
        r = self._call(self.switch_ctrl, req)
        return r is not None and r.ok

    def run(self, auto_yes):
        if not self.ensure_jic():
            return

        time.sleep(1.0)
        target = np.array(HOME_JOINT_POSITIONS)
        delta = target - self.joints()
        for i, d in enumerate(delta, 1):
            self.get_logger().info(f"  A{i}: {np.degrees(d):+.1f} deg")
        if not auto_yes and input("Move to home posture? [y/N]: ").strip().lower() != "y":
            return

        p = RCLParameter()
        p.name = "stiffness"
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = STIFFNESS_NM_RAD
        req = SetParameters.Request()
        req.parameters = [p]
        self._call(self.params, req)

        self.action.wait_for_server()
        goal = JointTrajectoryExecution.Goal()
        goal.joint_positions_goal = Float64MultiArray(data=target.tolist())
        goal.joint_velocities_goal = Float64MultiArray(data=[VELOCITY_RAD_S] * 7)

        for attempt in (1, 2):
            fut = self.action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().error("Goal rejected.")
                continue
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut)
            ok = bool(res_fut.result().result.success)
            self.get_logger().info(f"Home result (try {attempt}): {'SUCCESS' if ok else 'FAILED'}")
            if ok:
                break
            time.sleep(1.0)




def main():
    rclpy.init()
    node = Homer()
    try:
        node.run("--yes" in sys.argv)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()