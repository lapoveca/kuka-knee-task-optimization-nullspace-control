#include "lbr_impedance_control/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "urdf/model.h"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <vector>

namespace lbr_impedance_control {

using Ctrl = CartesianImpedanceController;

controller_interface::InterfaceConfiguration
Ctrl::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &name : joint_names_) {
    if (!simulation_)
      cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
Ctrl::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

controller_interface::CallbackReturn Ctrl::on_init() {
  try {
    get_node()->declare_parameter("robot_name", "lbr");
    get_node()->declare_parameter("base_link", "lbr_link_0");
    get_node()->declare_parameter("end_effector_link", "lbr_link_ee");
    get_node()->declare_parameter("simulation", false);
    get_node()->declare_parameter("fallback_controllers", std::vector<std::string>{});

    get_node()->declare_parameter("translational_stiffness", 300.0);
    get_node()->declare_parameter("translational_damping_ratio", 0.7);
    get_node()->declare_parameter("rotational_stiffness", 10.0);
    get_node()->declare_parameter("rotational_damping_ratio", 0.7);
    get_node()->declare_parameter("separate_axis", false);
    get_node()->declare_parameter("translational_stiffness_x", 300.0);
    get_node()->declare_parameter("translational_stiffness_y", 300.0);
    get_node()->declare_parameter("translational_stiffness_z", 300.0);
    get_node()->declare_parameter("rotational_stiffness_alpha", 10.0);
    get_node()->declare_parameter("rotational_stiffness_theta", 10.0);
    get_node()->declare_parameter("rotational_stiffness_phi", 10.0);

    // Assistance
    get_node()->declare_parameter("assistance", false);
    get_node()->declare_parameter("assistance_stiffness", 0.0);
    get_node()->declare_parameter("assistance_z_offset", 0.0);
    get_node()->declare_parameter("sticky_assistance", false);
    get_node()->declare_parameter("sticky_assistance_z_pose", 0.0);
    get_node()->declare_parameter("sticky_assistance_z_margin_latch", 0.0);
    get_node()->declare_parameter("sticky_assistance_z_margin_unlatch", 0.0);

    // Safety & trajectory quality
    const std::array<double, N_JOINTS> default_effort_limits = {
        320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0};
    for (int i = 0; i < N_JOINTS; ++i)
      get_node()->declare_parameter(
          "joint_" + std::to_string(i) + ".effort_limit", default_effort_limits[i]);

    get_node()->declare_parameter("goal_translation_tolerance", 0.01);
    get_node()->declare_parameter("goal_rotation_tolerance", 0.1);
    get_node()->declare_parameter("time_limit", 2.0);
    get_node()->declare_parameter("settling_time_ms", 0);

    get_node()->declare_parameter("measured_velocity_filter_tau", 0.02);

    // ---- Modular null-space controller (all off by default) ----
    // ns_enabled = false  -> tau_ns == 0 exactly (baseline CIC).
    // Each objective is switched on by giving it a weight > 0.
    get_node()->declare_parameter("ns_enabled", false);

    get_node()->declare_parameter("ns_weight_joint_limit", 0.0); // k_ns [Nm]

    get_node()->declare_parameter("ns_weight_torque_min", 0.0);  // k_tm [Nm]

    // Singularity/manipulability objective.
    // This is a non-negative objective weight; zero disables it.
    get_node()->declare_parameter("ns_weight_singularity", 0.0);
    get_node()->declare_parameter("ns_singularity_fd_step", 1e-3);
    get_node()->declare_parameter("ns_singularity_epsilon", 1e-8);
    get_node()->declare_parameter("ns_singularity_length_scale", 0.35);

   
    
    get_node()->declare_parameter("ns_max_torque", 10.0); // [Nm]

    get_node()->declare_parameter("ns_directional_types", std::vector<std::string>{});
    get_node()->declare_parameter("ns_directional_weights", std::vector<double>{});
    get_node()->declare_parameter("ns_directional_directions_ee", std::vector<double>{});

    get_node()->declare_parameter("ns_directional_fd_step", 1e-3);



    get_node()->declare_parameter("ns_damping", 1.0); // d_ns [Nms/rad]

    get_node()->declare_parameter("ns_headroom_scaling", false);
    get_node()->declare_parameter("ns_torque_soft_ratio", 0.70);

    // Candidate rated-payload axis speeds, rad/s.
    get_node()->declare_parameter("joint_velocity_limits_rad_s",
        std::vector<double>{1.483529864, 1.483529864, 1.745329252, 1.308996939,
            2.268928028, 2.356194490, 2.356194490});


  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Ctrl::on_configure(const rclcpp_lifecycle::State &) {
  robot_name_         = get_node()->get_parameter("robot_name").as_string();
  base_link_          = get_node()->get_parameter("base_link").as_string();
  end_effector_link_  = get_node()->get_parameter("end_effector_link").as_string();
  simulation_         = get_node()->get_parameter("simulation").as_bool();
  update_joint_names();

  for (int i = 0; i < N_JOINTS; ++i)
    effort_limits_[i] = get_node()->get_parameter(
        "joint_" + std::to_string(i) + ".effort_limit").as_double();
  goal_translation_tolerance_ =
      get_node()->get_parameter("goal_translation_tolerance").as_double();
  goal_rotation_tolerance_ =
      get_node()->get_parameter("goal_rotation_tolerance").as_double();
  time_limit_ = get_node()->get_parameter("time_limit").as_double();

  measured_velocity_filter_tau_ = get_node()->get_parameter("measured_velocity_filter_tau").as_double();

  
  const std::string robot_description = get_robot_description();
  if (robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  urdf::Model urdf_model;
  if (!urdf_model.initString(robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF.");
    return controller_interface::CallbackReturn::ERROR;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build KDL tree.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!tree.getChain(base_link_, end_effector_link_, eef_chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get KDL chain %s -> %s.",
                 base_link_.c_str(), end_effector_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  lbr_chain_ = eef_chain_;

  fk_solver_  = std::make_unique<KDL::ChainFkSolverPos_recursive>(eef_chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(eef_chain_);
  q_kdl_   = KDL::JntArray(N_JOINTS);
  kdl_jac_ = KDL::Jacobian(N_JOINTS);

  update_stiffness_damping();
  update_ns_optimizer_params();

  param_cb_handle_ = get_node()->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        update_stiffness_damping();
        update_ns_optimizer_params();
        for (const auto &p : params) {
          for (int i = 0; i < N_JOINTS; ++i) {
            if (p.get_name() == "joint_" + std::to_string(i) + ".effort_limit")
              effort_limits_[i] = p.as_double();
          }
          if (p.get_name() == "goal_translation_tolerance")
            goal_translation_tolerance_ = p.as_double();
          if (p.get_name() == "goal_rotation_tolerance")
            goal_rotation_tolerance_ = p.as_double();
          if (p.get_name() == "time_limit")
            time_limit_ = p.as_double();
          if (p.get_name() == "measured_velocity_filter_tau") 
            measured_velocity_filter_tau_ = p.as_double();
        }
      });

  const auto velocity_limits = get_node()->get_parameter("joint_velocity_limits_rad_s").as_double_array();

  if (velocity_limits.size() != N_JOINTS)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "joint_velocity_limits_rad_s must contain %d values.",
                 N_JOINTS);
    return controller_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < N_JOINTS; ++i)
  {
    if (!std::isfinite(velocity_limits[i]) || velocity_limits[i] <= 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint velocity limits must be finite and positive.");
      return controller_interface::CallbackReturn::ERROR;
    }
    qdot_limits_[i] = velocity_limits[i];
  }

  

  


  // ---- ROS interfaces ----

  ref_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/reference_cartesian_pose", 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        CartesianReference ref;
        ref.position    = Eigen::Vector3d(msg->pose.position.x,
                                          msg->pose.position.y,
                                          msg->pose.position.z);
        ref.orientation = Eigen::Quaterniond(msg->pose.orientation.w,
                                             msg->pose.orientation.x,
                                             msg->pose.orientation.y,
                                             msg->pose.orientation.z);

        auto previous = rt_ref_pose_.readFromRT();

        if (previous) {
          ref.twist =
              previous->twist;
        }

        rt_ref_pose_.writeFromNonRT(ref);
      });

  reference_twist_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
          "~/reference_cartesian_twist", 1, 
          [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
          {
            auto previous =
                rt_ref_pose_.readFromRT();

            if (!previous)
            {
              return;
            }

            CartesianReference ref =
                *previous;

            ref.twist << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;

            rt_ref_pose_.writeFromNonRT(ref);
          });

  action_server_ = rclcpp_action::create_server<CartesianTrajectoryExecution>(
      get_node(), "~/cartesian_trajectory",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const CartesianTrajectoryExecution::Goal> goal) {
        return handle_goal(uuid, goal);
      },
      [this](std::shared_ptr<GoalHandleCTE> handle) {
        return handle_cancel(handle);
      },
      [this](std::shared_ptr<GoalHandleCTE> handle) {
        handle_accepted(handle);
      });

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

  cartesian_pose_pub_  = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/cartesian_pose", 10);
  cartesian_twist_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
      "~/cartesian_twist", 10);
  trajectory_pub_      = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/trajectory", 10);
  commanded_torque_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/commanded_torque", 10);
  cartesian_wrench_pub_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/cartesian_wrench", 10);
  diagnostics_pub_ = get_node()->create_publisher<NullspaceDiagnostics>(
      "~/nullspace_diagnostics", 10);
  diagnostics_rt_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<NullspaceDiagnostics>>(
          diagnostics_pub_);
  desired_cartesian_twist_pub_ =
    get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        "~/desired_cartesian_twist", 10);

  filtered_cartesian_twist_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/filtered_cartesian_twist",
          10);

  nullspace_torque_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/nullspace_torque",
          10);
  ns_costs_pub_ =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/nullspace_costs", 10);

  ee_velocity_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
          "~/ee_velocity", 10);

  ee_force_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
          "~/ee_force_estimate", 10);

  velocity_ellipsoid_pub_ =
      get_node()->create_publisher<visualization_msgs::msg::Marker>(
          "~/velocity_capability_ellipsoid", 10);

  force_ellipsoid_pub_ =
      get_node()->create_publisher<visualization_msgs::msg::Marker>(
          "~/force_capability_ellipsoid", 10);




  RCLCPP_INFO(get_node()->get_logger(),
              "CartesianImpedanceController configured.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Ctrl::on_activate(const rclcpp_lifecycle::State &) {
  position_cmd_.clear(); effort_cmd_.clear();
  position_state_.clear(); velocity_state_.clear(); effort_state_.clear();

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
    else if (iface.get_interface_name() == hardware_interface::HW_IF_EFFORT)
      effort_state_.emplace_back(std::ref(iface));
  }

  if ((!simulation_ && position_cmd_.size() != N_JOINTS) ||
      effort_cmd_.size()    != N_JOINTS ||
      position_state_.size() != N_JOINTS ||
      velocity_state_.size() != N_JOINTS ||
      effort_state_.size()   != N_JOINTS) {
    RCLCPP_ERROR(get_node()->get_logger(), "Interface count mismatch.");
    return controller_interface::CallbackReturn::ERROR;
  }

  Eigen::Matrix<double, N_JOINTS, 1> q;
  for (int i = 0; i < N_JOINTS; ++i)
    q(i) = position_state_[i].get().get_optional().value_or(0.0);
  Eigen::Affine3d T = get_fk(q);
  ee_position_ref_    = T.translation();
  ee_orientation_ref_ = Eigen::Quaterniond(T.linear()).normalized();
  ee_position_home_    = ee_position_ref_;
  ee_orientation_home_ = ee_orientation_ref_;

  cart_vel_filtered_.setZero();

  CartesianReference initial_ref;
  initial_ref.position    = ee_position_ref_;
  initial_ref.orientation = ee_orientation_ref_;
  rt_ref_pose_.writeFromNonRT(initial_ref);

  RCLCPP_INFO(get_node()->get_logger(), "CartesianImpedanceController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
Ctrl::on_deactivate(const rclcpp_lifecycle::State &) {
  position_cmd_.clear(); effort_cmd_.clear();
  position_state_.clear(); velocity_state_.clear(); effort_state_.clear();
  cart_vel_ref_.setZero();
  cart_vel_filtered_.setZero();
  // previous_ref_valid_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
Ctrl::update(const rclcpp::Time &time, const rclcpp::Duration &period) {

  // --- Service requests ---
  if (set_reference_requested_.exchange(false))
  {
    Eigen::Matrix<double, N_JOINTS, 1> q;

    for (int i = 0; i < N_JOINTS; ++i)
    {
      q(i) =
          position_state_[i].get().get_optional().value_or(0.0);
    }

    Eigen::Affine3d T = get_fk(q);

    ee_position_ref_ = T.translation();
    ee_orientation_ref_ = Eigen::Quaterniond(T.linear()).normalized();

    cart_vel_ref_.setZero();
    cart_vel_filtered_.setZero();

    CartesianReference hold;
    hold.position = ee_position_ref_;

    hold.orientation = ee_orientation_ref_;

    hold.twist.setZero();

    rt_ref_pose_.writeFromNonRT(hold);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Reference set to current pose.");
  }
  if (set_home_requested_.exchange(false)) {
    ee_position_home_    = ee_position_ref_;
    ee_orientation_home_ = ee_orientation_ref_;
    RCLCPP_INFO(get_node()->get_logger(), "Home set to current reference.");
  }
  if (set_reference_to_home_requested_.exchange(false)) {
    ee_position_ref_    = ee_position_home_;
    ee_orientation_ref_ = ee_orientation_home_;
    RCLCPP_INFO(get_node()->get_logger(), "Reference set to home.");
  }



  // --- Streaming reference from Python ---
  auto traj_ptr =
      rt_trajectory_.readFromRT();

  const bool action_active = traj_ptr && *traj_ptr && (*traj_ptr)->active;

  if (!action_active) {
    auto ref_ptr =
        rt_ref_pose_.readFromRT();

    if (ref_ptr) {
      ee_position_ref_ = ref_ptr->position;
      ee_orientation_ref_ = ref_ptr->orientation.normalized();
      cart_vel_ref_ = ref_ptr->twist;
    }
  }

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    2000,
    "accepted ref: p=[%.4f %.4f %.4f], twist=[%.4f %.4f %.4f]",
    ee_position_ref_.x(),
    ee_position_ref_.y(),
    ee_position_ref_.z(),
    cart_vel_ref_(0),
    cart_vel_ref_(1),
    cart_vel_ref_(2));

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    2000,
    "internal trajectory: p=[%.4f %.4f %.4f]",
    ee_position_ref_.x(),
    ee_position_ref_.y(),
    ee_position_ref_.z());

  // --- Action trajectory ---
  if (action_active) {
    auto &traj = **traj_ptr;
    traj.elapsed_time += period.seconds();

    const double alpha_t = std::min(
        traj.v_ref_translation * traj.time_factor_translation * traj.elapsed_time /
            std::max((traj.position_goal - traj.position_start).norm(), 1e-6),
        1.0);
    const double alpha_r = std::min(
        traj.v_ref_rotation * traj.time_factor_rotation * traj.elapsed_time, 1.0);

    ee_position_ref_ = traj.position_start +
        alpha_t * (traj.position_goal - traj.position_start);
    ee_orientation_ref_ = traj.orientation_start.slerp(alpha_r, traj.orientation_goal);

    if (traj.elapsed_time >= traj.total_time) {
      ee_position_ref_    = traj.position_goal;
      ee_orientation_ref_ = traj.orientation_goal;
      traj.active = false;
    }

    cart_vel_ref_.setZero();

  }

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "ref vel: lin %.4f m/s, ang %.4f rad/s",
    cart_vel_ref_.head<3>().norm(), cart_vel_ref_.tail<3>().norm());
  
  // --- Read joint state ---
  Eigen::Matrix<double, N_JOINTS, 1> q, q_dot, tau_measured;
  for (int i = 0; i < N_JOINTS; ++i) {
    q(i)            = position_state_[i].get().get_optional().value_or(0.0);
    q_dot(i)        = velocity_state_[i].get().get_optional().value_or(0.0);
    tau_measured(i) = effort_state_[i].get().get_optional().value_or(0.0);
  }

  // --- FK and Jacobian ---
  const Eigen::Affine3d T    = get_fk(q);
  const Eigen::Vector3d    ee_position(T.translation());
  Eigen::Quaterniond       ee_orientation(T.linear());
  const JacobianMatrix    J = get_jacobian(q);

  // --- Diagnostic publisher ---
  if (diagnostics_rt_pub_ &&
      diagnostics_pub_->get_subscription_count() > 0 &&
      diagnostics_rt_pub_->trylock()) {
    fill_diagnostics(diagnostics_rt_pub_->msg_, time, q, q_dot, J, T.linear());
    diagnostics_rt_pub_->unlockAndPublish();
  }

  // --- Pose error (same method as iiwa) ---
  Eigen::Matrix<double, 6, 1> ee_pose_error;
  ee_pose_error.head(3) = ee_position - ee_position_ref_;

  if (ee_orientation_ref_.coeffs().dot(ee_orientation.coeffs()) < 0.0)
    ee_orientation.coeffs() = -ee_orientation.coeffs();
  // const Eigen::Quaterniond error_quat = ee_orientation.inverse() * ee_orientation_ref_;
  // ee_pose_error.tail(3) = 
  //         -T.linear() * Eigen::Vector3d(error_quat.x(), error_quat.y(), error_quat.z());
  const Eigen::Quaterniond error_quat =
      (ee_orientation.inverse() * ee_orientation_ref_).normalized();
  const Eigen::AngleAxisd error_aa(error_quat);
  ee_pose_error.tail(3) = -T.linear() * (error_aa.axis() * error_aa.angle());



  // --- Impedance torques ---

  // Measured Cartesian velocity from the geometric Jacobian.
  const Eigen::Matrix<double, 6, 1> cart_vel = J * q_dot;

  // First-order low-pass filter for measured Cartesian velocity.
  // This filters only the velocity used by the damping feedback.
  const double dt = std::clamp(period.seconds(), 1e-5, 0.05);

  const double alpha = dt / (measured_velocity_filter_tau_ + dt);

  cart_vel_filtered_ += alpha * (cart_vel - cart_vel_filtered_);

  // Desired-minus-measured Cartesian velocity.
  const Eigen::Matrix<double, 6, 1> cart_vel_error = cart_vel_ref_ - cart_vel_filtered_;

  // Cartesian impedance wrench. Kept separate because the torque-minimisation
  // objective needs F to evaluate tau_c(q) = J(q)^T F at perturbed q.
  const Eigen::Matrix<double, 6, 1> F_task = -K_ * ee_pose_error + D_ * cart_vel_error;

  // Cartesian impedance torque (unchanged result).
  const Eigen::VectorXd tau_task = J.transpose() * F_task;

  // --- Moore-Penrose pseudo-inverse of the Jacobian transpose ---
  Eigen::MatrixXd J_T_pinv;
  pseudo_inverse(J.transpose(), &J_T_pinv);

  // --- Nullspace projector (torque space): N = I - J^T (J^T)^+ ---
  const Eigen::MatrixXd null_proj =
      Eigen::MatrixXd::Identity(N_JOINTS, N_JOINTS) - J.transpose() * J_T_pinv;

  // --- Assistance torques ---
  Eigen::Matrix<double, 6, 1> ee_offset_cart;
  ee_offset_cart.head(3) = ee_offset_ee_;
  ee_offset_cart.tail(3).setZero();

  Eigen::VectorXd tau_ext = Eigen::VectorXd::Zero(N_JOINTS);
  if (z_margin_latch_ != 0.0) {
    if (latched_) {
      if (ee_position(2) < (z_pose_ - z_margin_unlatch_) ||
          ee_position(2) > (z_pose_ + z_margin_unlatch_))
        latched_ = false;
      else
        tau_ext = J.transpose() * (K_ext_ * ee_offset_cart);
    } else {
      if (ee_position(2) < (z_pose_ + z_margin_latch_) &&
          ee_position(2) > (z_pose_ - z_margin_latch_)) {
        tau_ext = J.transpose() * (K_ext_ * ee_offset_cart);
        latched_ = true;
      }
    }
  } else {
    tau_ext = J.transpose() * (K_ext_ * ee_offset_cart);
  }



  // ==================================================================
  // Modular null-space controller
  //
  //   tau_0   = - sum_k w_k grad c_k(q)  - d_ns q_dot
  //   tau_ns  = sat( s(rho) * N tau_0 ),  N = I - J^T (J^T)^+
  //
  // Objectives (weight w_k > 0 enables it):
  //   A) joint-limit centering   w = ns_weight_joint_limit_  (k_ns)
  //   B) torque minimisation     w = ns_weight_torque_min_   (k_tm)
  // To add objective C: write ns_grad_C(), declare a weight parameter,
  // and add one line to the sum below.
  //
  // ns_enabled_ == false  ->  tau_ns == 0  ->  identical to baseline CIC.
  // N is a Tikhonov-damped projector (pseudo_inverse lambda), so tau_ns
  // can leak into task space; measured by static_leakage_* below.
  // ==================================================================
  Eigen::Matrix<double, N_JOINTS, 1> tau_ns =
      Eigen::Matrix<double, N_JOINTS, 1>::Zero();
  Eigen::Matrix<double, N_JOINTS, 1> tau_ns_unprojected =
      Eigen::Matrix<double, N_JOINTS, 1>::Zero();
  Eigen::Matrix<double, N_JOINTS, 1> tau_ns_projected =
      Eigen::Matrix<double, N_JOINTS, 1>::Zero();

  // Primary-task torque utilisation (logging always; used for scaling if on).
  double ns_primary_torque_utilisation = 0.0;
  for (int i = 0; i < N_JOINTS; ++i) {
    ns_primary_torque_utilisation = std::max(
        ns_primary_torque_utilisation,
        std::abs(tau_task(i) + tau_ext(i)) / effort_limits_[i]);
  }
  double ns_headroom_scale = 1.0;


  // Cost values for logging only.
  const double ns_cost_joint_limit_val = ns_joint_limit_cost(q);
  const double ns_cost_torque_val = ns_torque_cost(J, F_task);
  const double ns_cost_singularity_val = ns_singularity_cost(J);





  Eigen::Matrix<double, N_JOINTS, 1> ns_null_direction =
      Eigen::Matrix<double, N_JOINTS, 1>::Zero();
  bool ns_null_direction_valid = false;

  const bool any_directional_objective =
      std::any_of(
          directional_objectives_.begin(),
          directional_objectives_.end(),
          [](const DirectionalObjective &objective)
          {
            return std::isfinite(objective.weight) &&
                   objective.weight > 0.0;
          });

  if (ns_enabled_ && any_directional_objective)
  {
    const Eigen::JacobiSVD<JacobianMatrix> ns_svd(
        J, Eigen::ComputeFullV);

    if (ns_svd.info() == Eigen::Success)
    {
      const auto singular_values = ns_svd.singularValues();

      constexpr int kTaskDof = 6;
      constexpr double kRelativeRankTolerance = 1e-6;

      const double sigma_max = singular_values(0);
      const double sigma_min_task = singular_values(kTaskDof - 1);

      const bool full_row_rank =
          std::isfinite(sigma_max) &&
          std::isfinite(sigma_min_task) &&
          sigma_max > 0.0 &&
          sigma_min_task > kRelativeRankTolerance * sigma_max;

      if (full_row_rank)
      {
        ns_null_direction =
            ns_svd.matrixV().col(N_JOINTS - 1);
        ns_null_direction_valid =
            ns_null_direction.allFinite() &&
            std::abs(ns_null_direction.norm() - 1.0) < 1e-6;
      }
    }
  }

  const Eigen::Matrix3d R_ee_base = T.linear().transpose();

  // J is the geometric Jacobian at the EE origin.
  // cart_vel is currently J * q_dot, expressed in base axes.
  const Eigen::Vector3d v_ee =
      R_ee_base * cart_vel.head<3>();

  // Your existing reconstruction is a least-squares equivalent wrench estimate.
  // Rotate only its force component; do not transform it as a point.
  const Eigen::VectorXd wrench_base =
      J_T_pinv * tau_measured;
  const Eigen::Vector3d f_ee =
      R_ee_base * wrench_base.head<3>();

  if (ns_enabled_) {
    Eigen::Matrix<double, N_JOINTS, 1> tau_0 =
        Eigen::Matrix<double, N_JOINTS, 1>::Zero();

    // A) joint-limit centering: tau_0 -= k_ns * grad c_joint
    if (ns_weight_joint_limit_ > 0.0)
      tau_0 -= ns_weight_joint_limit_ * ns_grad_joint_limit(q);

    // B) torque minimisation: tau_0 -= k_tm * (dc_torque/dpsi) n
    if (ns_weight_torque_min_ > 0.0)
      tau_0 -= ns_weight_torque_min_ * ns_grad_torque_min(q, J, F_task);

    // C) singularity avoidance:
    //
    // tau_0 -= k_sing * (dc_sing/dpsi) n
    //
    // Since c_sing = -log(w), this torque moves in the direction that
    // increases manipulability and therefore repels singular configurations.
    if (ns_weight_singularity_ > 0.0)
      tau_0 -= ns_weight_singularity_
               * ns_grad_singularity(q, J);

    // Directional objectives selected for this trajectory.
    if (ns_null_direction_valid)
    {
      for (const auto &objective : directional_objectives_)
      {
        if (!std::isfinite(objective.weight) || objective.weight <= 0.0)
        {
          continue;
        }

        const auto grad = ns_grad_directional_objective(
            q, ns_null_direction, T.linear(), objective);

        if (grad.valid)
        {
          tau_0 -= objective.weight * grad.gradient;
        }
      }
    }

    // Stabiliser: null-space damping (removed by the projector wherever it
    // would act on the task).
    tau_0 -= ns_damping_ * q_dot;

    tau_ns_unprojected = tau_0;
    tau_ns_projected   = null_proj * tau_0;

    // Optional headroom scaling: null-space torque fades out linearly
    // between rho_soft and 1 (primary task near an effort limit).
    if (ns_headroom_scaling_ &&
        ns_primary_torque_utilisation > ns_tau_soft_ratio_) {
      const double denom = 1.0 - ns_tau_soft_ratio_;
      ns_headroom_scale = denom > 1e-9
          ? std::clamp((1.0 - ns_primary_torque_utilisation) / denom, 0.0, 1.0)
          : 0.0;
    }

    // TODO, remove this or change it:
    // for now, headroom scaling disabled
    // tau_ns = ns_headroom_scale * tau_ns_projected;
    tau_ns = tau_ns_projected;


    // Direction-preserving saturation: scaling the whole vector keeps tau_ns
    // inside the null space (per-component clipping would not).
    const double tau_ns_inf = tau_ns.cwiseAbs().maxCoeff();
    if (tau_ns_inf > ns_tau_max_ && tau_ns_inf > 1e-12)
      tau_ns *= ns_tau_max_ / tau_ns_inf;
  }







  // Static task leakage diagnostic:
  //
  // F_leak = (J^T)^+ tau_ns.
  //
  // An exact Moore-Penrose static projector gives approximately zero.
  // With the current damped pseudo-inverse and any numerical effects,
  // this can be non-zero. This is diagnostic only: it does not modify
  // the commanded torque.
  const Eigen::Matrix<double, 6, 1> static_leakage_wrench =
      J_T_pinv * tau_ns;

  const double static_leakage_norm = static_leakage_wrench.norm();

  RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "NS: |tau_ns|=%.3f Nm, rho=%.3f, headroom=%.3f, "
      "|F_static_leak|=%.6f",
      tau_ns.norm(),
      ns_primary_torque_utilisation,
      ns_headroom_scale,
      static_leakage_norm);

  // --- Sum and final actuator-limit clamp ---
  //
  // When ns_enabled_ is false, tau_ns is exactly zero, so this is
  // identical to the original CIC:
  //     tau = tau_task + tau_ext.
  Eigen::VectorXd tau = tau_task + tau_ns + tau_ext;
  
  for (int i = 0; i < N_JOINTS; ++i)
    tau(i) = std::clamp(tau(i), -effort_limits_[i], effort_limits_[i]);

  // --- Write to hardware ---
  for (int i = 0; i < N_JOINTS; ++i) {
    if (!simulation_)
      std::ignore = position_cmd_[i].get().set_value(q(i));
    std::ignore = effort_cmd_[i].get().set_value(tau(i));
  }

  // --- Publish ---
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = base_link_;
    msg.pose.position.x    = ee_position.x();
    msg.pose.position.y    = ee_position.y();
    msg.pose.position.z    = ee_position.z();
    msg.pose.orientation.x = ee_orientation.x();
    msg.pose.orientation.y = ee_orientation.y();
    msg.pose.orientation.z = ee_orientation.z();
    msg.pose.orientation.w = ee_orientation.w();
    cartesian_pose_pub_->publish(msg);
  }
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id  = base_link_;
    msg.twist.linear.x   = cart_vel(0); msg.twist.linear.y  = cart_vel(1); msg.twist.linear.z  = cart_vel(2);
    msg.twist.angular.x  = cart_vel(3); msg.twist.angular.y = cart_vel(4); msg.twist.angular.z = cart_vel(5);
    cartesian_twist_pub_->publish(msg);
  }
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = base_link_;
    msg.pose.position.x    = ee_position_ref_.x();
    msg.pose.position.y    = ee_position_ref_.y();
    msg.pose.position.z    = ee_position_ref_.z();
    msg.pose.orientation.x = ee_orientation_ref_.x();
    msg.pose.orientation.y = ee_orientation_ref_.y();
    msg.pose.orientation.z = ee_orientation_ref_.z();
    msg.pose.orientation.w = ee_orientation_ref_.w();
    trajectory_pub_->publish(msg);
  }
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(N_JOINTS);
    for (int i = 0; i < N_JOINTS; ++i) msg.data[i] = tau(i);
    commanded_torque_pub_->publish(msg);
  }
  {
    const Eigen::VectorXd wrench = J_T_pinv * tau_measured;
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.frame_id = base_link_;
    msg.wrench.force.x  = wrench(0); msg.wrench.force.y  = wrench(1); msg.wrench.force.z  = wrench(2);
    msg.wrench.torque.x = wrench(3); msg.wrench.torque.y = wrench(4); msg.wrench.torque.z = wrench(5);
    cartesian_wrench_pub_->publish(msg);
  }
  {
  geometry_msgs::msg::TwistStamped filtered_twist_msg;
  filtered_twist_msg.header.stamp = time;
  filtered_twist_msg.header.frame_id = base_link_;
  filtered_twist_msg.twist.linear.x = cart_vel_filtered_(0);
  filtered_twist_msg.twist.linear.y = cart_vel_filtered_(1);
  filtered_twist_msg.twist.linear.z = cart_vel_filtered_(2);
  filtered_twist_msg.twist.angular.x = cart_vel_filtered_(3);
  filtered_twist_msg.twist.angular.y = cart_vel_filtered_(4);
  filtered_twist_msg.twist.angular.z = cart_vel_filtered_(5);
  filtered_cartesian_twist_pub_->publish(filtered_twist_msg);
  }
  {
  std_msgs::msg::Float64MultiArray ns_msg;
  ns_msg.data.resize(N_JOINTS);

  for (int i = 0; i < N_JOINTS; ++i) {
    ns_msg.data[i] = tau_ns(i);
  }

  nullspace_torque_pub_->publish(ns_msg);
  }
  {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {ns_cost_joint_limit_val,
              ns_cost_torque_val,
              ns_cost_singularity_val,
              ns_primary_torque_utilisation,
              ns_headroom_scale,
              static_leakage_norm};
  ns_costs_pub_->publish(msg);
  }
  {
  geometry_msgs::msg::TwistStamped desired_twist_msg;
  desired_twist_msg.header.stamp = time;
  desired_twist_msg.header.frame_id = base_link_;
  desired_twist_msg.twist.linear.x = cart_vel_ref_(0);
  desired_twist_msg.twist.linear.y = cart_vel_ref_(1);
  desired_twist_msg.twist.linear.z = cart_vel_ref_(2);
  desired_twist_msg.twist.angular.x = cart_vel_ref_(3);
  desired_twist_msg.twist.angular.y = cart_vel_ref_(4);
  desired_twist_msg.twist.angular.z = cart_vel_ref_(5);
  desired_cartesian_twist_pub_->publish(desired_twist_msg);
  }
  {
  geometry_msgs::msg::TwistStamped velocity_msg;
  velocity_msg.header.stamp = time;
  velocity_msg.header.frame_id = end_effector_link_;
  velocity_msg.twist.linear.x = v_ee.x();
  velocity_msg.twist.linear.y = v_ee.y();
  velocity_msg.twist.linear.z = v_ee.z();
  // Leave angular components at zero for this visualization.
  ee_velocity_pub_->publish(velocity_msg);
  }
  {
  geometry_msgs::msg::WrenchStamped force_msg;
  force_msg.header.stamp = time;
  force_msg.header.frame_id = end_effector_link_;
  force_msg.wrench.force.x = f_ee.x();
  force_msg.wrench.force.y = f_ee.y();
  force_msg.wrench.force.z = f_ee.z();
  // Leave torque components at zero for this visualization.
  ee_force_pub_->publish(force_msg);
  }

  std_msgs::msg::ColorRGBA velocity_color;
  velocity_color.r = 0.1f;
  velocity_color.g = 0.4f;
  velocity_color.b = 1.0f;
  velocity_color.a = 0.35f;

  std_msgs::msg::ColorRGBA force_color;
  force_color.r = 1.0f;
  force_color.g = 0.2f;
  force_color.b = 0.1f;
  force_color.a = 0.35f;

  velocity_ellipsoid_pub_->publish(
      make_linear_capability_ellipsoid(
          J, T.linear(), time,
          "velocity_capability", 0, velocity_color));

  force_ellipsoid_pub_->publish(
      make_linear_capability_ellipsoid(
          J, T.linear(), time,
          "force_capability", 1, force_color));

  return controller_interface::return_type::OK;
}

void Ctrl::update_joint_names() {
  for (int i = 0; i < N_JOINTS; ++i)
    joint_names_[i] = robot_name_ + "_A" + std::to_string(i + 1);
}

void Ctrl::update_ns_optimizer_params() {
  auto node = get_node();
  ns_enabled_ = node->get_parameter("ns_enabled").as_bool();

  // Negative weights would maximise the cost, so clamp them at 0.
  ns_weight_joint_limit_ =
      std::max(0.0, node->get_parameter("ns_weight_joint_limit").as_double());

  ns_weight_torque_min_ =
      std::max(0.0, node->get_parameter("ns_weight_torque_min").as_double());

  ns_weight_singularity_ =
      std::max(0.0,
               node->get_parameter("ns_weight_singularity").as_double());
  ns_singularity_fd_step_ =
      std::max(1e-6,
               node->get_parameter("ns_singularity_fd_step").as_double());
  ns_singularity_epsilon_ =
      std::max(1e-12,
               node->get_parameter("ns_singularity_epsilon").as_double());
  ns_singularity_length_scale_ =
      std::max(1e-6,
               node->get_parameter("ns_singularity_length_scale").as_double());

  ns_damping_ = std::max(0.0, node->get_parameter("ns_damping").as_double());
  ns_headroom_scaling_ = node->get_parameter("ns_headroom_scaling").as_bool();
  ns_tau_soft_ratio_ =
      std::clamp(node->get_parameter("ns_torque_soft_ratio").as_double(), 0.0, 0.999);
  ns_tau_max_ = std::max(0.0, node->get_parameter("ns_max_torque").as_double());




  const auto types =
      node->get_parameter("ns_directional_types").as_string_array();
  const auto weights =
      node->get_parameter("ns_directional_weights").as_double_array();
  const auto directions =
      node->get_parameter("ns_directional_directions_ee").as_double_array();

  directional_objectives_.clear();

  if (types.size() != weights.size() ||
      directions.size() != 3 * types.size())
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Directional NS parameters have inconsistent sizes: "
        "types=%zu weights=%zu directions=%zu; disabling directional objectives.",
        types.size(), weights.size(), directions.size());
    return;
  }

  directional_objectives_.reserve(types.size());

  for (size_t i = 0; i < types.size(); ++i)
  {
    DirectionalObjective objective;

    if (types[i] == "linear_velocity")
    {
      objective.type = DirectionalObjectiveType::LINEAR_VELOCITY;
    }
    else if (types[i] == "angular_velocity")
    {
      objective.type = DirectionalObjectiveType::ANGULAR_VELOCITY;
    }
    else if (types[i] == "force")
    {
      objective.type = DirectionalObjectiveType::FORCE;
    }
    else if (types[i] == "moment")
    {
      objective.type = DirectionalObjectiveType::MOMENT;
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(),
                   "Unknown directional objective type '%s'; "
                   "disabling directional objectives.",
                   types[i].c_str());
      directional_objectives_.clear();
      return;
    }

    objective.weight = weights[i];
    if (!std::isfinite(objective.weight) || objective.weight < 0.0)
    {
      RCLCPP_ERROR(node->get_logger(),
                   "Directional objective weight at index %zu is invalid; "
                   "disabling directional objectives.",
                   i);
      directional_objectives_.clear();
      return;
    }

    objective.direction_ee = Eigen::Vector3d(
        directions[3 * i],
        directions[3 * i + 1],
        directions[3 * i + 2]);

    if (!objective.direction_ee.allFinite() ||
        objective.direction_ee.norm() < 1e-12)
    {
      RCLCPP_ERROR(node->get_logger(),
                   "Directional objective direction at index %zu is invalid; "
                   "disabling directional objectives.",
                   i);
      directional_objectives_.clear();
      return;
    }

    objective.direction_ee.normalize();
    directional_objectives_.push_back(objective);
  }

  ns_directional_fd_step_ = std::max(
      1e-6,
      node->get_parameter("ns_directional_fd_step").as_double());
}





Ctrl::DirectionalMetrics Ctrl::directional_metrics(
    const Eigen::Matrix<double, 3, N_JOINTS> &J_block_base,
    const Eigen::Vector3d &direction_ee,
    const Eigen::Matrix3d &R_base_ee) const
{
  DirectionalMetrics result;

  if (!direction_ee.allFinite() ||
      std::abs(direction_ee.norm() - 1.0) > 1e-6)
  {
    return result;
  }

  const Eigen::Matrix<double, 3, N_JOINTS> J_block_ee =
      R_base_ee.transpose() * J_block_base;

  Eigen::Matrix<double, N_JOINTS, N_JOINTS> W2 =
      Eigen::Matrix<double, N_JOINTS, N_JOINTS>::Zero();
  Eigen::Matrix<double, N_JOINTS, N_JOINTS> T_inv2 =
      Eigen::Matrix<double, N_JOINTS, N_JOINTS>::Zero();

  for (int i = 0; i < N_JOINTS; ++i)
  {
    if (!std::isfinite(qdot_limits_[i]) || qdot_limits_[i] <= 0.0 ||
        !std::isfinite(effort_limits_[i]) || effort_limits_[i] <= 0.0)
    {
      return result;
    }

    W2(i, i) = qdot_limits_[i] * qdot_limits_[i];
    T_inv2(i, i) =
        1.0 / (effort_limits_[i] * effort_limits_[i]);
  }

  const Eigen::Matrix3d A_velocity =
      J_block_ee * W2 * J_block_ee.transpose();

  const Eigen::LDLT<Eigen::Matrix3d> ldlt(A_velocity);
  if (ldlt.info() == Eigen::Success && ldlt.isPositive())
  {
    const Eigen::Vector3d solved = ldlt.solve(direction_ee);
    const double quadratic = direction_ee.dot(solved);

    if (ldlt.info() == Eigen::Success &&
        solved.allFinite() &&
        std::isfinite(quadratic) &&
        quadratic > 0.0)
    {
      const double radius = 1.0 / std::sqrt(quadratic);
      if (std::isfinite(radius) && radius > 0.0)
      {
        result.velocity_radius = radius;
        result.velocity_valid = true;
      }
    }
  }

  const Eigen::Matrix<double, N_JOINTS, 1> Jt_d =
      J_block_ee.transpose() * direction_ee;
  const double wrench_quadratic =
      Jt_d.dot(T_inv2 * Jt_d);

  if (std::isfinite(wrench_quadratic) &&
      wrench_quadratic > 1e-18)
  {
    const double radius = 1.0 / std::sqrt(wrench_quadratic);
    if (std::isfinite(radius) && radius > 0.0)
    {
      result.wrench_radius = radius;
      result.wrench_valid = true;
    }
  }

  return result;
}


// CAPABILITY ELLIPSOID MARKER VISUALIZATION

visualization_msgs::msg::Marker Ctrl::make_linear_capability_ellipsoid(
    const JacobianMatrix &J,
    const Eigen::Matrix3d &R_base_ee,
    const rclcpp::Time &stamp,
    const std::string &name_space,
    int id,
    const std_msgs::msg::ColorRGBA &color) const
{
  using Marker = visualization_msgs::msg::Marker;

  const bool is_velocity = (name_space == "velocity_capability");
  const bool is_force = (name_space == "force_capability");

  Marker marker;
  marker.header.frame_id = end_effector_link_;
  marker.header.stamp = stamp;
  marker.ns = name_space;
  marker.id = id;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color = color;
  marker.frame_locked = true;
  marker.lifetime = rclcpp::Duration::from_seconds(0.15);

  if (!is_velocity && !is_force)
  {
    marker.action = Marker::DELETE;
    return marker;
  }

  const Eigen::Matrix<double, 3, N_JOINTS> Jv_base = J.topRows<3>();

  // R_base_ee maps EE coordinates into base coordinates.
  // Rotate the translational Jacobian into the EE frame.
  const Eigen::Matrix<double, 3, N_JOINTS> Jv_ee =
      R_base_ee.transpose() * Jv_base;

  Eigen::Matrix<double, N_JOINTS, N_JOINTS> joint_weight =
      Eigen::Matrix<double, N_JOINTS, N_JOINTS>::Zero();

  for (int i = 0; i < N_JOINTS; ++i)
  {
    if (is_velocity)
    {
      const double limit = qdot_limits_[i];
      if (!std::isfinite(limit) || limit <= 0.0)
      {
        marker.action = Marker::DELETE;
        return marker;
      }

      // Velocity ellipsoid: Jv * diag(qdot_limit^2) * Jv^T.
      joint_weight(i, i) = limit * limit;
    }
    else
    {
      const double limit = effort_limits_[i];
      if (!std::isfinite(limit) || limit <= 0.0)
      {
        marker.action = Marker::DELETE;
        return marker;
      }

      // Force ellipsoid: inverse squared torque limits.
      joint_weight(i, i) = (limit * limit);
    }
  }

  const Eigen::Matrix3d A =
      Jv_ee * joint_weight * Jv_ee.transpose();
  const Eigen::Matrix3d A_symmetric = 0.5 * (A + A.transpose());

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(A_symmetric);

  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite())
  {
    marker.action = Marker::DELETE;
    return marker;
  }

  const double lambda_max = eig.eigenvalues().maxCoeff();
  const double lambda_min = eig.eigenvalues().minCoeff();

  if (lambda_max <= 0.0 || lambda_min <= 1e-12 * lambda_max)
  {
    marker.action = Marker::DELETE;
    return marker;
  }

  Eigen::Vector3d radii;
  if (is_velocity)
  {
    // Eigenvalues are squared velocity radii.
    radii = eig.eigenvalues().cwiseSqrt();
  }
  else
  {
    // Eigenvalues are inverse squared force radii.
    radii = eig.eigenvalues().cwiseSqrt();
  }


  Eigen::Matrix3d axes_rotation = eig.eigenvectors();
  if (axes_rotation.determinant() < 0.0)
  {
    axes_rotation.col(2) *= -1.0;
  }

  Eigen::Quaterniond orientation(axes_rotation);
  orientation.normalize();

  marker.pose.orientation.x = orientation.x();
  marker.pose.orientation.y = orientation.y();
  marker.pose.orientation.z = orientation.z();
  marker.pose.orientation.w = orientation.w();

  // Marker scale is the full ellipsoid diameter.
  marker.scale.x = 2.0 * radii.x();
  marker.scale.y = 2.0 * radii.y();
  marker.scale.z = 2.0 * radii.z();

  return marker;
}



// ---------------------------------------------------------------------
// Objective A: joint-range centering
//   c_joint(q) = (1/7) sum ((q_i - q_mid_i)/h_i)^2,  q_mid = 0 (Med 14 limits
//   are symmetric).  grad_i = 2 q_i / (7 h_i^2) = kG[i] q_i.
//   Half-ranges: A1/A3/A5 2.967060, A2/A4/A6 2.094395, A7 3.054326 rad.
// ---------------------------------------------------------------------
Eigen::Matrix<double, 7, 1> Ctrl::ns_grad_joint_limit(
    const Eigen::Matrix<double, N_JOINTS, 1> &q) const {
  static constexpr double kG[N_JOINTS] = {
      0.03245483, 0.06513505, 0.03245483, 0.06513505,
      0.03245483, 0.06513505, 0.03062677};
  Eigen::Matrix<double, N_JOINTS, 1> g;
  for (int i = 0; i < N_JOINTS; ++i)
    g(i) = kG[i] * q(i);
  return g;
}

// c = 0.5 q.grad because grad_i = kG_i q_i and c = 0.5 sum kG_i q_i^2.
double Ctrl::ns_joint_limit_cost(
    const Eigen::Matrix<double, N_JOINTS, 1> &q) const {
  return 0.5 * q.dot(ns_grad_joint_limit(q));
}

// ---------------------------------------------------------------------
// Objective B: normalised squared joint torque
//   c_torque(q) = 1/2 sum_i ( tau_c,i(q) / tau_lim_i )^2,
//   tau_c(q) = J(q)^T F_task    (F_task held FIXED).
// ---------------------------------------------------------------------
double Ctrl::ns_torque_cost(
    const JacobianMatrix &Jq,
    const Eigen::Matrix<double, 6, 1> &F) const
{
  const Eigen::Matrix<double, N_JOINTS, 1> tau_c = Jq.transpose() * F;
  double c = 0.0;
  for (int i = 0; i < N_JOINTS; ++i)
  {
    const double r = tau_c(i) / effort_limits_[i];
    c += 0.5 * r * r;
  }
  return c;
}

// Returns (dc/dpsi) n, i.e. the null-space component n n^T grad c.
//   * n = right singular vector of the zero singular value of J (J n = 0).
//   * dc/dpsi = n^T grad c is taken by central finite difference along n,
//     so dJ/dq is never needed.
//   * (dc/dpsi) n does not depend on the sign of n.
// The caller applies  tau_0 -= k_tm * (this).
Eigen::Matrix<double, 7, 1> Ctrl::ns_grad_torque_min(
    const Eigen::Matrix<double, N_JOINTS, 1> &q,
    const JacobianMatrix &J,
    const Eigen::Matrix<double, 6, 1> &F_task)
{
  // Zero wrench -> zero cost for every q -> zero gradient.
  if (F_task.norm() < 1e-9)
    return Eigen::Matrix<double, N_JOINTS, 1>::Zero();

  const Eigen::JacobiSVD<JacobianMatrix> svd(J, Eigen::ComputeFullV);
  const Eigen::Matrix<double, N_JOINTS, 1> n = svd.matrixV().col(N_JOINTS - 1);

  constexpr double kFdStep = 1e-3; // rad along n
  const double c_plus = ns_torque_cost(get_jacobian(q + kFdStep * n), F_task);
  const double c_minus = ns_torque_cost(get_jacobian(q - kFdStep * n), F_task);
  const double dc_dpsi = (c_plus - c_minus) / (2.0 * kFdStep);

  return dc_dpsi * n;
}

// ---------------------------------------------------------------------
// Objective C: singularity avoidance through manipulability maximisation
//
// The Jacobian contains linear and angular velocity rows with different
// units. We therefore use the length-scaled Jacobian
//
//     J_L = diag(I_3, L I_3) J,
//
// where L is a characteristic length. The Yoshikawa manipulability measure
// is
//
//     w(q) = sqrt(det(J_L J_L^T))
//
// and is equal to the product of the six singular values of J_L.
// It approaches zero at a rank-deficient configuration.
//
// We minimise
//
//     c_sing(q) = -log(w(q) + epsilon).
//
// Thus low manipulability is penalised strongly, while increasing an already
// large manipulability value gives diminishing returns.
//
// The implementation only needs the derivative along the one-dimensional
// self-motion direction n. It estimates dc_sing/dpsi with a central
// finite difference and returns (dc_sing/dpsi) n.
// ---------------------------------------------------------------------

double Ctrl::ns_singularity_cost(
    const JacobianMatrix &J) const {
  // Length-scaled Jacobian: J_L = diag(I_3, L I_3) J
  Eigen::Matrix<double, 6, 7> J_scaled = J;
  const double L = std::max(ns_singularity_length_scale_, 1e-9);
  J_scaled.bottomRows<3>() *= L;

  // Compute A = J_scaled * J_scaled^T (6x6, symmetric positive semi-definite).
  const Eigen::Matrix<double, 6, 6> A = J_scaled * J_scaled.transpose();

  // Eigenvalues of A are sigma_i^2. Use SelfAdjointEigenSolver (no U/V needed).
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(A);
  if (es.info() != Eigen::Success) {
    // Fallback: return a large cost if eigendecomposition fails.
    return 1e6;
  }

  const auto &evals = es.eigenvalues();  // ascending, all >= 0 (up to numerical noise)
  double log_manipulability = 0.0;

  for (int i = 0; i < 6; ++i) {
    // sigma_i = sqrt(lambda_i), but log(sigma_i) = 0.5 * log(lambda_i)
    const double lam = std::max(evals(i), ns_singularity_epsilon_ * ns_singularity_epsilon_);
    log_manipulability += 0.5 * std::log(lam);
  }

  // c_sing = -log(w) = -sum log(sigma_i) = -log_manipulability
  return -log_manipulability;
}

Eigen::Matrix<double, 7, 1> Ctrl::ns_grad_singularity(
    const Eigen::Matrix<double, N_JOINTS, 1> &q,
    const JacobianMatrix &J) {
  const Eigen::JacobiSVD<JacobianMatrix> svd(
      J, Eigen::ComputeFullV);

  // For a 6x7 full-row-rank Jacobian, the last right singular vector spans
  // the one-dimensional joint-velocity null space: J*n approximately equals 0.
  const Eigen::Matrix<double, N_JOINTS, 1> n =
      svd.matrixV().col(N_JOINTS - 1);

  const double step = std::max(ns_singularity_fd_step_, 1e-6);

  const double c_plus =
      ns_singularity_cost(get_jacobian(q + step * n));
  const double c_minus =
      ns_singularity_cost(get_jacobian(q - step * n));

  const double dc_dpsi =
      (c_plus - c_minus) / (2.0 * step);

  // The sign of n is arbitrary, but dc_dpsi changes sign with n as well,
  // so their product is invariant.
  return dc_dpsi * n;
}


// ---------------------------------------------------------------------
// Objective D: directional velocity  metrics
// ---------------------------------------------------------------------

double Ctrl::directional_objective_cost(
    const JacobianMatrix &J,
    const Eigen::Matrix3d &R_base_ee,
    const DirectionalObjective &objective,
    bool *valid) const {
  const bool angular =
      objective.type == DirectionalObjectiveType::ANGULAR_VELOCITY ||
      objective.type == DirectionalObjectiveType::MOMENT;
  const bool velocity =
      objective.type == DirectionalObjectiveType::LINEAR_VELOCITY ||
      objective.type == DirectionalObjectiveType::ANGULAR_VELOCITY;

  const Eigen::Matrix<double, 3, N_JOINTS> block =
      angular ? J.bottomRows<3>() : J.topRows<3>();

  const auto metric =
      directional_metrics(block, objective.direction_ee, R_base_ee);

  *valid = velocity ? metric.velocity_valid : metric.wrench_valid;
  if (!*valid) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double radius =
      velocity ? metric.velocity_radius : metric.wrench_radius;
  return -std::log(radius);
}

Ctrl::DirectionalGradient Ctrl::ns_grad_directional_objective(
    const Eigen::Matrix<double, N_JOINTS, 1> &q,
    const Eigen::Matrix<double, N_JOINTS, 1> &n,
    const Eigen::Matrix3d &R_base_ee,
    const DirectionalObjective &objective)
{
  DirectionalGradient result;

  const bool angular =
      objective.type == DirectionalObjectiveType::ANGULAR_VELOCITY ||
      objective.type == DirectionalObjectiveType::MOMENT;

  const bool velocity_objective =
      objective.type == DirectionalObjectiveType::LINEAR_VELOCITY ||
      objective.type == DirectionalObjectiveType::ANGULAR_VELOCITY;

  const double h = ns_directional_fd_step_;
  if (!std::isfinite(h) || h <= 0.0 ||
      !objective.direction_ee.allFinite() ||
      std::abs(objective.direction_ee.norm() - 1.0) > 1e-6)
  {
    return result;
  }

  const JacobianMatrix J_plus = get_jacobian(q + h * n);
  const JacobianMatrix J_minus = get_jacobian(q - h * n);

  Eigen::Matrix<double, 3, N_JOINTS> block_plus;
  Eigen::Matrix<double, 3, N_JOINTS> block_minus;


  if (angular)
  {
    block_plus = J_plus.bottomRows<3>();
    block_minus = J_minus.bottomRows<3>();
  }
  else
  {
    block_plus = J_plus.topRows<3>();
    block_minus = J_minus.topRows<3>();
  }

  const auto plus =
      directional_metrics(block_plus, objective.direction_ee, R_base_ee);
  const auto minus =
      directional_metrics(block_minus, objective.direction_ee, R_base_ee);


  if (velocity_objective)
  {
    if (!plus.velocity_valid || !minus.velocity_valid)
    {
      return result;
    }
  }
  else
  {
    if (!plus.wrench_valid || !minus.wrench_valid)
    {
      return result;
    }
  }

  const double radius_plus = velocity_objective
                                 ? plus.velocity_radius
                                 : plus.wrench_radius;
  const double radius_minus = velocity_objective
                                  ? minus.velocity_radius
                                  : minus.wrench_radius;

  const double c_plus = -std::log(radius_plus);
  const double c_minus = -std::log(radius_minus);
  const double dc_dpsi = (c_plus - c_minus) / (2.0 * h);

  if (!std::isfinite(dc_dpsi))
  {
    return result;
  }

  result.gradient = dc_dpsi * n;
  result.valid = result.gradient.allFinite();
  return result;
}



void Ctrl::update_stiffness_damping() {
  const bool separate = get_node()->get_parameter("separate_axis").as_bool();

  if (!separate) {
    const double k_t  = get_node()->get_parameter("translational_stiffness").as_double();
    const double k_r  = get_node()->get_parameter("rotational_stiffness").as_double();
    const double dr_t = get_node()->get_parameter("translational_damping_ratio").as_double();
    const double dr_r = get_node()->get_parameter("rotational_damping_ratio").as_double();
    K_.topLeftCorner(3,3)     = k_t * Eigen::Matrix3d::Identity();
    K_.bottomRightCorner(3,3) = k_r * Eigen::Matrix3d::Identity();
    D_.topLeftCorner(3,3)     = 2.0 * dr_t * std::sqrt(k_t) * Eigen::Matrix3d::Identity();
    D_.bottomRightCorner(3,3) = 2.0 * dr_r * std::sqrt(k_r) * Eigen::Matrix3d::Identity();
  } else {
    const double kx  = get_node()->get_parameter("translational_stiffness_x").as_double();
    const double ky  = get_node()->get_parameter("translational_stiffness_y").as_double();
    const double kz  = get_node()->get_parameter("translational_stiffness_z").as_double();
    const double ka  = get_node()->get_parameter("rotational_stiffness_alpha").as_double();
    const double kt  = get_node()->get_parameter("rotational_stiffness_theta").as_double();
    const double kp  = get_node()->get_parameter("rotational_stiffness_phi").as_double();
    const double dr_t = get_node()->get_parameter("translational_damping_ratio").as_double();
    const double dr_r = get_node()->get_parameter("rotational_damping_ratio").as_double();
    K_(0,0) = kx; D_(0,0) = 2.0 * dr_t * std::sqrt(kx);
    K_(1,1) = ky; D_(1,1) = 2.0 * dr_t * std::sqrt(ky);
    K_(2,2) = kz; D_(2,2) = 2.0 * dr_t * std::sqrt(kz);
    K_(3,3) = ka; D_(3,3) = 2.0 * dr_r * std::sqrt(ka);
    K_(4,4) = kt; D_(4,4) = 2.0 * dr_r * std::sqrt(kt);
    K_(5,5) = kp; D_(5,5) = 2.0 * dr_r * std::sqrt(kp);
  }

  if (get_node()->get_parameter("assistance").as_bool()) {
    const double k_ext = get_node()->get_parameter("assistance_stiffness").as_double();
    const double z_off = get_node()->get_parameter("assistance_z_offset").as_double();
    ee_offset_ee_ = Eigen::Vector3d(0.0, 0.0, -z_off);
    K_ext_.topLeftCorner(3,3)     = k_ext * Eigen::Matrix3d::Identity();
    K_ext_.bottomRightCorner(3,3).setZero();
  } else {
    ee_offset_ee_.setZero();
    K_ext_.setZero();
    latched_ = false;
  }

  if (get_node()->get_parameter("sticky_assistance").as_bool()) {
    z_pose_          = get_node()->get_parameter("sticky_assistance_z_pose").as_double();
    z_margin_latch_  = get_node()->get_parameter("sticky_assistance_z_margin_latch").as_double();
    z_margin_unlatch_= get_node()->get_parameter("sticky_assistance_z_margin_unlatch").as_double();
  } else {
    z_pose_ = 0.0; z_margin_latch_ = 0.0; z_margin_unlatch_ = 0.0;
    latched_ = false;
  }
}

Eigen::Affine3d Ctrl::get_fk(const Eigen::Matrix<double, N_JOINTS, 1> &q) {
  q_kdl_.data = q;
  fk_solver_->JntToCart(q_kdl_, kdl_frame_);
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      T.linear()(i, j) = kdl_frame_.M(i, j);
  for (int i = 0; i < 3; ++i)
    T.translation()(i) = kdl_frame_.p(i);
  return T;
}

Ctrl::JacobianMatrix Ctrl::get_jacobian(const Eigen::Matrix<double, N_JOINTS, 1> &q) {
  q_kdl_.data = q;
  jac_solver_->JntToJac(q_kdl_, kdl_jac_);

  JacobianMatrix J;
  J = kdl_jac_.data;
  return J;
}

rclcpp_action::GoalResponse Ctrl::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CartesianTrajectoryExecution::Goal> goal) {
  if (goal->translational_velocity_goal <= 0.0 ||
      goal->rotational_velocity_goal    <= 0.0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Goal rejected: velocities must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Ctrl::handle_cancel(std::shared_ptr<GoalHandleCTE>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Ctrl::handle_accepted(std::shared_ptr<GoalHandleCTE> goal_handle) {
  std::thread([this, goal_handle]() {
    execute_trajectory(goal_handle);
  }).detach();
}


void Ctrl::fill_diagnostics(
    NullspaceDiagnostics &msg, const rclcpp::Time &stamp,
    const Eigen::Matrix<double, N_JOINTS, 1> &q,
    const Eigen::Matrix<double, N_JOINTS, 1> &q_dot,
    const JacobianMatrix &J,
    const Eigen::Matrix3d &R_base_ee) const {

  msg.stamp = stamp;
  for (int i = 0; i < N_JOINTS; ++i) {
    msg.joint_position[i] = q(i);
    msg.joint_velocity[i] = q_dot(i);
  }

  constexpr double kEps         = 1e-12;
  constexpr double kTibiaLength = 0.35;
  constexpr double kInf         = std::numeric_limits<double>::infinity();

  // 1) Base frame, translational rows only
  const Eigen::Matrix<double, 3, N_JOINTS> Jv = J.topRows<3>();
  const Eigen::Matrix3d Avv = Jv * Jv.transpose();
  const double det_Avv = Avv.determinant();

  if (det_Avv > kEps) {
    msg.w_v = std::sqrt(det_Avv);
    const Eigen::Matrix3d Avv_inv = Avv.inverse();
    for (int i = 0; i < 3; ++i) {
      msg.velocity_manip_xyz_base[i] = 1.0 / std::sqrt(Avv_inv(i, i));
      msg.force_manip_xyz_base[i]    = 1.0 / std::sqrt(Avv(i, i));
    }
  } else {
    msg.w_v = 0.0;
    for (int i = 0; i < 3; ++i) {
      msg.velocity_manip_xyz_base[i] = 0.0;
      msg.force_manip_xyz_base[i]    = 1.0 / std::sqrt(std::max(Avv(i, i), kEps));
    }
  }
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      msg.velocity_matrix_v_base[3 * row + col] = Avv(row, col);

  // 2) Rotational block
  const Eigen::Matrix<double, 3, N_JOINTS> Jw = J.bottomRows<3>();
  const Eigen::Matrix3d Aww = Jw * Jw.transpose();
  msg.w_omega = std::sqrt(std::max(Aww.determinant(), 0.0));

  // 3) Full 6D, base and EE frames
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  using Vec6 = Eigen::Matrix<double, 6, 1>;

  const Mat6 A_base = J * J.transpose();
  Mat6 R6 = Mat6::Zero();
  R6.topLeftCorner<3, 3>()     = R_base_ee.transpose();
  R6.bottomRightCorner<3, 3>() = R_base_ee.transpose();
  const Mat6 A_ee = R6 * A_base * R6.transpose();

  const Eigen::SelfAdjointEigenSolver<Mat6> es(A_base);
  const bool es_ok = (es.info() == Eigen::Success);
  Vec6 lam = Vec6::Zero();
  if (es_ok) lam = es.eigenvalues();
  const bool singular = !es_ok || lam(0) <= kEps;

  msg.w_6 = es_ok
      ? kTibiaLength * kTibiaLength * kTibiaLength * std::sqrt(std::max(lam.prod(), 0.0))
      : 0.0;

  Mat6 A_inv_base = Mat6::Zero();
  Mat6 A_inv_ee   = Mat6::Zero();
  if (!singular) {
    const Vec6 lam_inv = lam.cwiseInverse();
    A_inv_base = es.eigenvectors() * lam_inv.asDiagonal() * es.eigenvectors().transpose();
    A_inv_ee   = R6 * A_inv_base * R6.transpose();
  }

  for (int i = 0; i < 6; ++i) {
    msg.force_manip_6d_base[i] = 1.0 / std::sqrt(std::max(A_base(i, i), kEps));
    msg.force_manip_6d_ee[i]   = 1.0 / std::sqrt(std::max(A_ee(i, i), kEps));
    msg.velocity_manip_6d_base[i] = singular ? 0.0 : 1.0 / std::sqrt(A_inv_base(i, i));
    msg.velocity_manip_6d_ee[i]   = singular ? 0.0 : 1.0 / std::sqrt(A_inv_ee(i, i));
  }
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < 6; ++col)
      msg.velocity_matrix_6d_ee[6 * row + col] = A_ee(row, col);

  // 4) Scaled singular values
  Vec6 d;
  d << 1.0, 1.0, 1.0, kTibiaLength, kTibiaLength, kTibiaLength;
  const Mat6 A_L = d.asDiagonal() * A_base * d.asDiagonal();
  const Eigen::SelfAdjointEigenSolver<Mat6> es_L(A_L, Eigen::EigenvaluesOnly);

  if (es_L.info() == Eigen::Success) {
    const double lam_min = es_L.eigenvalues()(0);
    const double lam_max = es_L.eigenvalues()(5);
    msg.sigma_min_scaled = std::sqrt(std::max(lam_min, 0.0));
    msg.cond_scaled = (lam_min > kEps) ? std::sqrt(lam_max / lam_min) : kInf;
  } else {
    msg.sigma_min_scaled = 0.0;
    msg.cond_scaled      = kInf;
  }

  // Directional metrics
  const std::array<Eigen::Vector3d, 3> ee_axes = {
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ()};

  for (int i = 0; i < 3; ++i)
  {
    const auto linear = directional_metrics(Jv, ee_axes[i], R_base_ee);
    const auto angular = directional_metrics(Jw, ee_axes[i], R_base_ee);

    msg.velocity_manip_linear_ee[i] =
        linear.velocity_valid
            ? linear.velocity_radius
            : std::numeric_limits<double>::quiet_NaN();

    msg.force_manip_linear_ee[i] =
        linear.wrench_valid
            ? linear.wrench_radius
            : std::numeric_limits<double>::quiet_NaN();

    msg.velocity_manip_angular_ee[i] =
        angular.velocity_valid
            ? angular.velocity_radius
            : std::numeric_limits<double>::quiet_NaN();

    msg.moment_manip_angular_ee[i] =
        angular.wrench_valid
            ? angular.wrench_radius
            : std::numeric_limits<double>::quiet_NaN();
  }
}


void Ctrl::execute_trajectory(std::shared_ptr<GoalHandleCTE> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback   = std::make_shared<CartesianTrajectoryExecution::Feedback>();
  auto result     = std::make_shared<CartesianTrajectoryExecution::Result>();

  auto seed_ref_pose = [this]() {
    if (position_state_.size() == static_cast<size_t>(N_JOINTS)) {
      Eigen::Matrix<double, N_JOINTS, 1> q_hold;
      for (int i = 0; i < N_JOINTS; ++i)
        q_hold(i) = position_state_[i].get().get_optional().value_or(0.0);
      Eigen::Affine3d T_hold = get_fk(q_hold);
      CartesianReference hold;
      hold.position    = T_hold.translation();
      hold.orientation = Eigen::Quaterniond(T_hold.linear()).normalized();
      rt_ref_pose_.writeFromNonRT(hold);
    }
  };

  Eigen::Vector3d    curr_pos = ee_position_ref_;
  Eigen::Quaterniond curr_ori = ee_orientation_ref_;
  if (position_state_.size() == static_cast<size_t>(N_JOINTS)) {
    Eigen::Matrix<double, N_JOINTS, 1> q;
    for (int i = 0; i < N_JOINTS; ++i)
      q(i) = position_state_[i].get().get_optional().value_or(0.0);
    Eigen::Affine3d T = get_fk(q);
    curr_pos = T.translation();
    curr_ori = Eigen::Quaterniond(T.linear()).normalized();
  }

  const Eigen::Vector3d pos_start(
      goal->pose_start.pose.position.x == 99 ? curr_pos.x() : goal->pose_start.pose.position.x,
      goal->pose_start.pose.position.y == 99 ? curr_pos.y() : goal->pose_start.pose.position.y,
      goal->pose_start.pose.position.z == 99 ? curr_pos.z() : goal->pose_start.pose.position.z);
  const Eigen::Quaterniond ori_start(
      goal->pose_start.pose.orientation.w == 99 ? curr_ori.w() : goal->pose_start.pose.orientation.w,
      goal->pose_start.pose.orientation.x == 99 ? curr_ori.x() : goal->pose_start.pose.orientation.x,
      goal->pose_start.pose.orientation.y == 99 ? curr_ori.y() : goal->pose_start.pose.orientation.y,
      goal->pose_start.pose.orientation.z == 99 ? curr_ori.z() : goal->pose_start.pose.orientation.z);

  const Eigen::Vector3d pos_goal(
      goal->pose_goal.pose.position.x == 99 ? curr_pos.x() : goal->pose_goal.pose.position.x,
      goal->pose_goal.pose.position.y == 99 ? curr_pos.y() : goal->pose_goal.pose.position.y,
      goal->pose_goal.pose.position.z == 99 ? curr_pos.z() : goal->pose_goal.pose.position.z);
  const Eigen::Quaterniond ori_goal(
      goal->pose_goal.pose.orientation.w == 99 ? curr_ori.w() : goal->pose_goal.pose.orientation.w,
      goal->pose_goal.pose.orientation.x == 99 ? curr_ori.x() : goal->pose_goal.pose.orientation.x,
      goal->pose_goal.pose.orientation.y == 99 ? curr_ori.y() : goal->pose_goal.pose.orientation.y,
      goal->pose_goal.pose.orientation.z == 99 ? curr_ori.z() : goal->pose_goal.pose.orientation.z);

  auto traj = std::make_shared<CartesianTrajectoryGoal>();
  traj->position_start    = pos_start;
  traj->orientation_start = ori_start.normalized();
  traj->position_goal     = pos_goal;
  traj->orientation_goal  = ori_goal.normalized();

  const double dist_t = (pos_goal - pos_start).norm();
  const double dist_r = ori_start.normalized().angularDistance(ori_goal.normalized());
  const double v_t    = goal->translational_velocity_goal;
  const double v_r    = goal->rotational_velocity_goal;

  traj->v_ref_translation = v_t;
  traj->v_ref_rotation    = v_r;
  traj->translation_vector = dist_t > 1e-6 ? (pos_goal - pos_start).normalized()
                                            : Eigen::Vector3d::Zero();

  const double total_time_t = dist_t > 1e-6 ? dist_t / v_t : 0.0;
  const double total_time_r = dist_r > 1e-6 ? dist_r / v_r : 0.0;
  traj->total_time = std::max(total_time_t, total_time_r);

  if (traj->total_time < 1e-6) {
    seed_ref_pose();
    result->success = true;
    goal_handle->succeed(result);
    return;
  }

  traj->time_factor_translation = (total_time_t > total_time_r && total_time_t > 1e-6)
                                      ? 1.0
                                      : (total_time_t > 1e-6 ? total_time_t / traj->total_time : 1.0);
  traj->time_factor_rotation    = (total_time_r > total_time_t && total_time_r > 1e-6)
                                      ? 1.0
                                      : (total_time_r > 1e-6 ? total_time_r / traj->total_time : 1.0);

  traj->elapsed_time = 0.0;
  traj->active       = true;
  rt_trajectory_.writeFromNonRT(traj);

  const auto wall_start = get_node()->now();
  rclcpp::Rate rate(50);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      seed_ref_pose();
      rt_trajectory_.writeFromNonRT(nullptr);
      result->success = false;
      goal_handle->canceled(result);
      return;
    }
    const double wall_elapsed = (get_node()->now() - wall_start).seconds();
    if (wall_elapsed > traj->total_time * time_limit_) {
      seed_ref_pose();
      rt_trajectory_.writeFromNonRT(nullptr);
      result->success = false;
      RCLCPP_WARN(get_node()->get_logger(),
                  "Cartesian trajectory aborted: time limit (%.1fx) exceeded.", time_limit_);
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

  {
    CartesianReference goal_ref;
    goal_ref.position    = pos_goal;
    goal_ref.orientation = ori_goal;
    rt_ref_pose_.writeFromNonRT(goal_ref);
  }

  const int settling_ms = get_node()->get_parameter("settling_time_ms").as_int();
  if (settling_ms > 0)
    rclcpp::sleep_for(std::chrono::milliseconds(settling_ms));

  bool goal_reached = true;
  if (position_state_.size() == static_cast<size_t>(N_JOINTS)) {
    Eigen::Matrix<double, N_JOINTS, 1> q;
    for (int i = 0; i < N_JOINTS; ++i)
      q(i) = position_state_[i].get().get_optional().value_or(0.0);
    Eigen::Affine3d T_curr = get_fk(q);
    const double trans_err = (T_curr.translation() - pos_goal).norm();
    const double rot_err   = Eigen::Quaterniond(T_curr.linear()).normalized()
                                 .angularDistance(ori_goal.normalized());
    if (trans_err > goal_translation_tolerance_) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Translation tolerance not met: %.4f m (limit %.4f m)",
                  trans_err, goal_translation_tolerance_);
      goal_reached = false;
    }
    if (rot_err > goal_rotation_tolerance_) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Rotation tolerance not met: %.4f rad (limit %.4f rad)",
                  rot_err, goal_rotation_tolerance_);
      goal_reached = false;
    }
  } else {
    goal_reached = false;
  }

  seed_ref_pose();
  result->success = goal_reached;
  if (goal_reached)
    goal_handle->succeed(result);
  else
    goal_handle->abort(result);
}

} // namespace lbr_impedance_control

PLUGINLIB_EXPORT_CLASS(
    lbr_impedance_control::CartesianImpedanceController,
    controller_interface::ControllerInterface)
