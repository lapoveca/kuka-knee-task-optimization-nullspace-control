#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lbr_impedance_control/action/joint_trajectory_execution.hpp"

namespace lbr_impedance_control {

class JointImpedanceController : public controller_interface::ControllerInterface {
public:
  static constexpr int N_JOINTS = 7;

  using JointTrajectoryExecution = lbr_impedance_control::action::JointTrajectoryExecution;
  using GoalHandleJTE = rclcpp_action::ServerGoalHandle<JointTrajectoryExecution>;

  // Passed via RealtimeBuffer from action server (non-RT) to update() (RT)
  struct TrajectoryGoal {
    std::array<double, N_JOINTS> q_start{};
    std::array<double, N_JOINTS> q_goal{};
    std::array<double, N_JOINTS> v_max{};
    double total_time{0.0};
    double elapsed_time{0.0};
    bool active{false};
  };

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void update_joint_names();
  void update_stiffness_damping();

  bool simulation_{false};


  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const JointTrajectoryExecution::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandleJTE>);
  void handle_accepted(std::shared_ptr<GoalHandleJTE> goal_handle);
  void execute_trajectory(std::shared_ptr<GoalHandleJTE> goal_handle);

  // Robot config
  std::string robot_name_{"lbr"};
  std::array<std::string, N_JOINTS> joint_names_;

  // Impedance parameters (updated by parameter callback, read in RT update)
  std::array<double, N_JOINTS> stiffness_{};
  std::array<double, N_JOINTS> damping_{};

  // Desired joint positions — modified by RT update() during trajectory execution
  std::array<double, N_JOINTS> q_d_{};

  // Hardware interface handles (valid between on_activate / on_deactivate)
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> effort_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>  position_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>  velocity_state_;

  // Parameter callback handle — must stay alive as long as the callback is active
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Action server
  rclcpp_action::Server<JointTrajectoryExecution>::SharedPtr action_server_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TrajectoryGoal>> rt_trajectory_;

  // Home pose — set by service, used as reference target
  std::array<double, N_JOINTS> q_home_{};

  // Atomic flags — written by service callbacks (non-RT), read in update() (RT)
  std::atomic<bool> set_reference_requested_{false};
  std::atomic<bool> set_home_requested_{false};
  std::atomic<bool> set_reference_to_home_requested_{false};

  // Pose services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_reference_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_reference_to_home_srv_;

  // publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commanded_torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_pub_;

  // Safety & trajectory quality
  std::array<double, N_JOINTS> effort_limits_{};
  double time_limit_{2.0};
  std::array<double, N_JOINTS> goal_tolerances_{};

};

} // namespace lbr_impedance_control
