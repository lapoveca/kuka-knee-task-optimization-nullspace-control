#include "lbr_impedance_control/joint_impedance_controller.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace lbr_impedance_control {

  // ---------------------------------------------------------------------------
  // Interface configuration — tells controller_manager which interfaces to claim
  // ---------------------------------------------------------------------------

  controller_interface::InterfaceConfiguration
  JointImpedanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &name : joint_names_) {
      if (!simulation_) {
          cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
      }
      cfg.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return cfg;
  }

  controller_interface::InterfaceConfiguration
  JointImpedanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &name : joint_names_) {
      cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
      cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return cfg;
  }

  // ---------------------------------------------------------------------------
  // Lifecycle
  // ---------------------------------------------------------------------------

  controller_interface::CallbackReturn JointImpedanceController::on_init() {
    try {
      get_node()->declare_parameter("robot_name", "lbr");
      get_node()->declare_parameter("stiffness",   100.0);
      get_node()->declare_parameter("damping_ratio", 0.7);
      get_node()->declare_parameter("separate_joints", false);
      get_node()->declare_parameter("simulation", false);
      get_node()->declare_parameter("time_limit", 2.0);
      get_node()->declare_parameter("fallback_controllers", std::vector<std::string>{});
      
      // settling time (useful for trajectory optimization, realtime MPC --> set to 0)
      get_node()->declare_parameter("settling_time_ms", 0);  // 0 = disabled

      const std::array<double, N_JOINTS> default_effort_limits = {320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0};
      for (int i = 0; i < N_JOINTS; ++i){
        get_node()->declare_parameter("joint_" + std::to_string(i) + ".effort_limit",
                                      default_effort_limits[i]);
      }
      
      for (int i = 0; i < N_JOINTS; ++i){
        get_node()->declare_parameter("joint_" + std::to_string(i) + ".goal_tolerance", 0.05);
      }

      for (int i = 0; i < N_JOINTS; ++i) {
        get_node()->declare_parameter("joint_" + std::to_string(i) + ".stiffness",    100.0);
        get_node()->declare_parameter("joint_" + std::to_string(i) + ".damping_ratio",  0.7);
        get_node()->declare_parameter("joint_" + std::to_string(i) + ".q_d",            0.0);
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  JointImpedanceController::on_configure(const rclcpp_lifecycle::State &) {
    robot_name_ = get_node()->get_parameter("robot_name").as_string();
    update_joint_names();
    update_stiffness_damping();
    simulation_ = get_node()->get_parameter("simulation").as_bool();

    // Load desired positions from parameters (can be overridden at runtime or by action)
    for (int i = 0; i < N_JOINTS; ++i) {
      q_d_[i] = get_node()->get_parameter("joint_" + std::to_string(i) + ".q_d").as_double();
    }

    // Get effort limits
    for (int i = 0; i < N_JOINTS; ++i){
      effort_limits_[i] = get_node()->get_parameter(
        "joint_" + std::to_string(i) + ".effort_limit").as_double();
    }
    // get time limits
    time_limit_      = get_node()->get_parameter("time_limit").as_double();
    
    // get goal tolerances
    for (int i = 0; i < N_JOINTS; ++i){
      goal_tolerances_[i] = get_node()->get_parameter(
        "joint_" + std::to_string(i) + ".goal_tolerance").as_double();
    }

    // Parameter callback — post-set so get_parameter() returns the new value
    param_cb_handle_ = get_node()->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
        update_stiffness_damping();
        for (const auto & p : params) {
            for (int i = 0; i < N_JOINTS; ++i) {
                if (p.get_name() == "joint_" + std::to_string(i) + ".q_d")
                    q_d_[i] = p.as_double();
                if (p.get_name() == "joint_" + std::to_string(i) + ".effort_limit")
                    effort_limits_[i] = p.as_double();
                if (p.get_name() == "joint_" + std::to_string(i) + ".goal_tolerance")
                    goal_tolerances_[i] = p.as_double();
            }
            if (p.get_name() == "time_limit")
                time_limit_ = p.as_double();
        }
    });

    // Action server
    action_server_ = rclcpp_action::create_server<JointTrajectoryExecution>(
        get_node(),
        "~/joint_trajectory",
        [this](const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const JointTrajectoryExecution::Goal> goal) {
          return handle_goal(uuid, goal);
        },
        [this](std::shared_ptr<GoalHandleJTE> handle) {
          return handle_cancel(handle);
        },
        [this](std::shared_ptr<GoalHandleJTE> handle) {
          handle_accepted(handle);
        });

    // Pose services
    set_reference_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/set_reference_to_current",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        set_reference_requested_.store(true);
        resp->success = true;
        resp->message = "Reference will be set to current pose on next update cycle.";
      });

    set_home_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/set_home_to_current_reference",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        set_home_requested_.store(true);
        resp->success = true;
        resp->message = "Home will be set to current reference on next update cycle.";
      });

    set_reference_to_home_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/set_reference_to_home",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        set_reference_to_home_requested_.store(true);
        resp->success = true;
        resp->message = "Reference will be set to home on next update cycle.";
      });

     // Publishers
    commanded_torque_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/commanded_torque", 10);
    trajectory_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/trajectory", 10);

    RCLCPP_INFO(get_node()->get_logger(), "JointImpedanceController configured.");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  JointImpedanceController::on_activate(const rclcpp_lifecycle::State &) {
    position_cmd_.clear();
    effort_cmd_.clear();
    position_state_.clear();
    velocity_state_.clear();

    for (auto &iface : command_interfaces_) {
      if (iface.get_interface_name() == hardware_interface::HW_IF_POSITION)
        position_cmd_.emplace_back(std::ref(iface));
      else if (iface.get_interface_name() == hardware_interface::HW_IF_EFFORT)
        effort_cmd_.emplace_back(std::ref(iface));
    }
    for (auto &iface : state_interfaces_) {
      if (iface.get_interface_name() == hardware_interface::HW_IF_POSITION)
        position_state_.emplace_back(std::ref(iface));
      else if (iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
        velocity_state_.emplace_back(std::ref(iface));
    }

    if ((!simulation_ && position_cmd_.size() != N_JOINTS) ||
      effort_cmd_.size()    != N_JOINTS ||
      position_state_.size() != N_JOINTS ||
      velocity_state_.size() != N_JOINTS) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Interface count mismatch — got pos_cmd:%zu eff_cmd:%zu pos_st:%zu vel_st:%zu, need %d each",
            position_cmd_.size(), effort_cmd_.size(),
            position_state_.size(), velocity_state_.size(), N_JOINTS);
        return controller_interface::CallbackReturn::ERROR;
      }

    // Always start from current measured positions — never jump
    for (int i = 0; i < N_JOINTS; ++i) {
      q_d_[i] = position_state_[i].get().get_optional().value_or(0.0);
    }

    q_home_ = q_d_;   // home starts as current pose

    RCLCPP_INFO(get_node()->get_logger(), "JointImpedanceController activated.");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State &) {
    position_cmd_.clear();
    effort_cmd_.clear();
    position_state_.clear();
    velocity_state_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // ---------------------------------------------------------------------------
  // Update — runs at 200 Hz in the RT thread
  // ---------------------------------------------------------------------------

  controller_interface::return_type
  JointImpedanceController::update(const rclcpp::Time &, const rclcpp::Duration &period) {
    // Handle service requests (flags set from non-RT service callbacks)
    if (set_reference_requested_.exchange(false)) {
      for (int i = 0; i < N_JOINTS; ++i)
        q_d_[i] = position_state_[i].get().get_optional().value_or(q_d_[i]);
      RCLCPP_INFO(get_node()->get_logger(), "Reference set to current pose.");
    }
    if (set_home_requested_.exchange(false)) {
      q_home_ = q_d_;
      RCLCPP_INFO(get_node()->get_logger(), "Home set to current reference.");
    }
    if (set_reference_to_home_requested_.exchange(false)) {
      q_d_ = q_home_;
      RCLCPP_INFO(get_node()->get_logger(), "Reference set to home.");
    }
    
    // Check for active trajectory from action server (non-RT write, RT read)
    auto traj_ptr = rt_trajectory_.readFromRT();
    if (traj_ptr && *traj_ptr && (*traj_ptr)->active) {
      auto &traj = **traj_ptr;
      traj.elapsed_time += period.seconds();
      double alpha = std::min(traj.elapsed_time / traj.total_time, 1.0);
      for (int i = 0; i < N_JOINTS; ++i) {
        q_d_[i] = traj.q_start[i] + alpha * (traj.q_goal[i] - traj.q_start[i]);
      }
      if (traj.elapsed_time >= traj.total_time) {
        for (int i = 0; i < N_JOINTS; ++i) q_d_[i] = traj.q_goal[i];
        traj.active = false;
      }
    }

    // Impedance law: tau = K*(q_d - q) - D*dq
    std::array<double, N_JOINTS> tau_arr{};
    for (int i = 0; i < N_JOINTS; ++i) {
      const double q      = position_state_[i].get().get_optional().value_or(0.0);
      const double dq     = velocity_state_[i].get().get_optional().value_or(0.0);
      tau_arr[i] = stiffness_[i] * (q_d_[i] - q) - damping_[i] * dq;
      
      // Cap at hardware limits — exceeding these triggers a Sunrise safety stop
      tau_arr[i] = std::clamp(tau_arr[i], -effort_limits_[i], effort_limits_[i]);

      if (!simulation_) {
        std::ignore = position_cmd_[i].get().set_value(q);   // hardware only — zeros Sunrise spring
      }
      
      std::ignore = effort_cmd_[i].get().set_value(tau_arr[i]);  // always — impedance torque overlay

      // TODO: add tau_gravity[i] to tau here via KDL::ChainDynParam when needed
    
    }

    // Publish topics
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(tau_arr.begin(), tau_arr.end());
    commanded_torque_pub_->publish(tau_msg);

    std_msgs::msg::Float64MultiArray traj_msg;
    traj_msg.data.assign(q_d_.begin(), q_d_.end());
    trajectory_pub_->publish(traj_msg);

    return controller_interface::return_type::OK;
  }

  // ---------------------------------------------------------------------------
  // Private helpers
  // ---------------------------------------------------------------------------

  void JointImpedanceController::update_joint_names() {
    for (int i = 0; i < N_JOINTS; ++i)
      joint_names_[i] = robot_name_ + "_A" + std::to_string(i + 1);
  }

  void JointImpedanceController::update_stiffness_damping() {
    const bool separate = get_node()->get_parameter("separate_joints").as_bool();
    for (int i = 0; i < N_JOINTS; ++i) {
      if (separate) {
        stiffness_[i] = get_node()->get_parameter("joint_" + std::to_string(i) + ".stiffness").as_double();
        const double r = get_node()->get_parameter("joint_" + std::to_string(i) + ".damping_ratio").as_double();
        damping_[i]   = 2.0 * r * std::sqrt(stiffness_[i]);
      } else {
        stiffness_[i] = get_node()->get_parameter("stiffness").as_double();
        const double r = get_node()->get_parameter("damping_ratio").as_double();
        damping_[i]   = 2.0 * r * std::sqrt(stiffness_[i]);
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Action server
  // ---------------------------------------------------------------------------

  rclcpp_action::GoalResponse JointImpedanceController::handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const JointTrajectoryExecution::Goal> goal) {
    if (static_cast<int>(goal->joint_positions_goal.data.size())  != N_JOINTS ||
        static_cast<int>(goal->joint_velocities_goal.data.size()) != N_JOINTS) {
      RCLCPP_WARN(get_node()->get_logger(), "Goal rejected: expected %d joints", N_JOINTS);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse JointImpedanceController::handle_cancel(
      std::shared_ptr<GoalHandleJTE>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void JointImpedanceController::handle_accepted(std::shared_ptr<GoalHandleJTE> goal_handle) {
    std::thread([this, goal_handle]() { execute_trajectory(goal_handle); }).detach();
  }

  void JointImpedanceController::execute_trajectory(std::shared_ptr<GoalHandleJTE> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback   = std::make_shared<JointTrajectoryExecution::Feedback>();
  auto result     = std::make_shared<JointTrajectoryExecution::Result>();

  auto traj = std::make_shared<TrajectoryGoal>();
  for (int i = 0; i < N_JOINTS; ++i) {
    traj->q_start[i] = q_d_[i];
    traj->q_goal[i]  = goal->joint_positions_goal.data[i];
    traj->v_max[i]   = std::abs(goal->joint_velocities_goal.data[i]);
  }

  for (int i = 0; i < N_JOINTS; ++i) {
    const double dist = std::abs(traj->q_goal[i] - traj->q_start[i]);
    if (traj->v_max[i] > 1e-6)
      traj->total_time = std::max(traj->total_time, dist / traj->v_max[i]);
  }

  if (traj->total_time < 1e-6) {
    result->success = true;
    goal_handle->succeed(result);
    return;
  }

  traj->elapsed_time = 0.0;
  traj->active       = true;
  rt_trajectory_.writeFromNonRT(traj);

  const auto wall_start = get_node()->now();
  rclcpp::Rate rate(50);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      rt_trajectory_.writeFromNonRT(nullptr);
      result->success = false;
      goal_handle->canceled(result);
      return;
    }

    // Time limit check — abort if taking too long
    const double wall_elapsed = (get_node()->now() - wall_start).seconds();
    if (wall_elapsed > traj->total_time * time_limit_) {
      rt_trajectory_.writeFromNonRT(nullptr);
      result->success = false;
      RCLCPP_WARN(get_node()->get_logger(),
          "Trajectory aborted: time limit (%.1fx) exceeded.", time_limit_);
      goal_handle->abort(result);
      return;
    }

    const auto current = rt_trajectory_.readFromNonRT();
    if (!current || !*current) break;

    const float progress = static_cast<float>(
        std::min((*current)->elapsed_time / traj->total_time, 1.0));
    feedback->time_progress     = progress;
    feedback->distance_progress = progress;
    goal_handle->publish_feedback(feedback);

    if (!(*current)->active) break;
    rate.sleep();
  }
  
  // Settling wait - should be 0 for MPC!
  const int settling_ms = get_node()->get_parameter("settling_time_ms").as_int();
  if (settling_ms > 0){
    rclcpp::sleep_for(std::chrono::milliseconds(settling_ms));
  }

  // Goal tolerance check — did the robot actually arrive?
  bool goal_reached = true;
  if (position_state_.size() == static_cast<size_t>(N_JOINTS)) {
    for (int i = 0; i < N_JOINTS; ++i) {
      const double q = position_state_[i].get().get_optional().value_or(0.0);
      if (std::abs(q - traj->q_goal[i]) > goal_tolerances_[i]) {
        goal_reached = false;
        RCLCPP_WARN(get_node()->get_logger(),
            "Goal tolerance not met on joint %d: error=%.4f rad, tolerance=%.4f rad",
            i, std::abs(q - traj->q_goal[i]), goal_tolerances_[i]);
        break;
      }
    }
  } else {
    goal_reached = false;
  }

  result->success = goal_reached;
  if (goal_reached)
    goal_handle->succeed(result);
  else
    goal_handle->abort(result);
}

} // namespace lbr_impedance_control

PLUGINLIB_EXPORT_CLASS(
  lbr_impedance_control::JointImpedanceController,
  controller_interface::ControllerInterface)
