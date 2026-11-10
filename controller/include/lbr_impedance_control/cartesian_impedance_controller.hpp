#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include <string>
#include <std_msgs/msg/color_rgba.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lbr_impedance_control/action/cartesian_trajectory_execution.hpp"
#include "lbr_impedance_control/pseudo_inversion.hpp"

#include "lbr_impedance_control/msg/nullspace_diagnostics.hpp"
#include "realtime_tools/realtime_publisher.hpp"


using NullspaceDiagnostics = lbr_impedance_control::msg::NullspaceDiagnostics;


namespace lbr_impedance_control {

// Same structure as CartesianImpedanceController, but the null-space torque is produced by a
// projected joint-range gradient (Option A) instead of a fixed joint-space target q_nullspace.
class CartesianImpedanceController
    : public controller_interface::ControllerInterface {
public:
  static constexpr int N_JOINTS = 7;

  using JacobianMatrix = Eigen::Matrix<double, 6, N_JOINTS>;

  using CartesianTrajectoryExecution =
      lbr_impedance_control::action::CartesianTrajectoryExecution;
  using GoalHandleCTE = rclcpp_action::ServerGoalHandle<CartesianTrajectoryExecution>;

  struct CartesianReference {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Matrix<double, 6, 1> twist =
      Eigen::Matrix<double, 6, 1>::Zero();
};

  struct CartesianTrajectoryGoal {
    Eigen::Vector3d position_start{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation_start{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d position_goal{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation_goal{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d translation_vector{Eigen::Vector3d::Zero()};
    double v_ref_translation{0.0};
    double v_ref_rotation{0.0};
    double total_time{0.0};
    double time_factor_translation{1.0};
    double time_factor_rotation{1.0};
    double elapsed_time{0.0};
    bool active{false};
  };

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type
  update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  KDL::Chain eef_chain_;
  KDL::Chain lbr_chain_;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive>  fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver>         jac_solver_;

  KDL::JntArray  q_kdl_;
  KDL::Frame     kdl_frame_;
  KDL::Jacobian  kdl_jac_;

  Eigen::Matrix<double, 6, 6> K_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> D_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> K_ext_{Eigen::Matrix<double, 6, 6>::Zero()};


  // --- Modular null-space controller parameters (runtime-settable) ---
  bool   ns_enabled_{false};              // master switch (false -> baseline CIC)

  double ns_weight_joint_limit_{0.0};     // k_ns : joint-limit centering weight

  double ns_weight_torque_min_{0.0};      // k_tm : torque-minimisation weight

  double ns_weight_singularity_{0.0};
  double ns_singularity_fd_step_{1e-3};
  double ns_singularity_epsilon_{1e-8};
  double ns_singularity_length_scale_{0.35};

  double ns_damping_{1.0};          // d_ns : null-space damping
  bool ns_headroom_scaling_{false}; // optional torque-headroom scaling
  double ns_tau_soft_ratio_{0.70};
  double ns_tau_max_{10.0};

  double ns_directional_fd_step_{1e-3};

  std::vector<double> qdot_limits_{1.483529864, 1.483529864, 1.745329252, 1.308996939,
      2.268928028, 2.356194490, 2.356194490}; // candidate rated-payload axis speeds, rad/s

  // --- Objective helpers (each returns grad c_k wrt q; caller applies -w_k) ---
  Eigen::Matrix<double, N_JOINTS, 1> ns_grad_joint_limit(
      const Eigen::Matrix<double, N_JOINTS, 1> &q) const;
  double ns_joint_limit_cost(
      const Eigen::Matrix<double, N_JOINTS, 1> &q) const;

  Eigen::Matrix<double, N_JOINTS, 1> ns_grad_torque_min(
      const Eigen::Matrix<double, N_JOINTS, 1> &q,
      const JacobianMatrix &J,
      const Eigen::Matrix<double, 6, 1> &F_task); // non-const: uses get_jacobian()
  double ns_torque_cost(
      const JacobianMatrix &Jq,
      const Eigen::Matrix<double, 6, 1> &F) const;

  Eigen::Matrix<double, N_JOINTS, 1> ns_grad_singularity(
      const Eigen::Matrix<double, N_JOINTS, 1> &q,
      const JacobianMatrix &J);
  double ns_singularity_cost(
      const JacobianMatrix &J) const;




  enum class DirectionalObjectiveType
  {
      LINEAR_VELOCITY,
      ANGULAR_VELOCITY,
      FORCE,
      MOMENT
  };

  struct DirectionalObjective
  {
      DirectionalObjectiveType type;
      Eigen::Vector3d direction_ee;
      double weight;
  };

  struct DirectionalMetrics
  {
      double velocity_radius{0.0};
      double wrench_radius{0.0};
      bool velocity_valid{false};
      bool wrench_valid{false};
  };

  struct DirectionalGradient
  {
      Eigen::Matrix<double, N_JOINTS, 1> gradient =
          Eigen::Matrix<double, N_JOINTS, 1>::Zero();
      bool valid{false};
  };

  std::vector<DirectionalObjective> directional_objectives_;

  double directional_objective_cost(
      const JacobianMatrix &J,
      const Eigen::Matrix3d &R_base_ee,
      const DirectionalObjective &objective,
      bool *valid) const;

  DirectionalMetrics directional_metrics(
      const Eigen::Matrix<double, 3, N_JOINTS> &J_block_base,
      const Eigen::Vector3d &direction_ee,
      const Eigen::Matrix3d &R_base_ee) const;

  DirectionalGradient ns_grad_directional_objective(
      const Eigen::Matrix<double, N_JOINTS, 1> &q,
      const Eigen::Matrix<double, N_JOINTS, 1> &n,
      const Eigen::Matrix3d &R_base_ee,
      const DirectionalObjective &objective);





  // --- Logging publisher: [c_joint, c_torque, rho, headroom, leak] ---
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ns_costs_pub_;

  Eigen::Vector3d    ee_position_ref_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond ee_orientation_ref_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d    ee_position_home_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond ee_orientation_home_{Eigen::Quaterniond::Identity()};

  Eigen::Matrix<double, 6, 1> cart_vel_ref_ =
      Eigen::Matrix<double, 6, 1>::Zero();

  Eigen::Matrix<double, 6, 1>
      cart_vel_filtered_ =
          Eigen::Matrix<double, 6, 1>::Zero();

  double measured_velocity_filter_tau_{0.02};

  Eigen::Vector3d ee_offset_ee_{Eigen::Vector3d::Zero()};
  bool   latched_{false};
  double z_pose_{0.0};
  double z_margin_latch_{0.0};
  double z_margin_unlatch_{0.0};

  std::array<double, N_JOINTS> effort_limits_{};
  double goal_translation_tolerance_{0.01};
  double goal_rotation_tolerance_{0.1};
  double time_limit_{2.0};
  int    settling_time_ms_{0};

  std::string robot_name_{"lbr"};
  std::string base_link_{"lbr_link_0"};
  std::string end_effector_link_{"lbr_link_ee"};
  bool simulation_{false};
  std::array<std::string, N_JOINTS> joint_names_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      position_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      effort_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      position_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      velocity_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      effort_state_;

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::atomic<bool> set_reference_requested_{false};
  std::atomic<bool> set_home_requested_{false};
  std::atomic<bool> set_reference_to_home_requested_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_reference_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_reference_to_home_srv_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_twist_sub_;
  realtime_tools::RealtimeBuffer<CartesianReference> rt_ref_pose_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   cartesian_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  cartesian_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  commanded_torque_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr cartesian_wrench_pub_;

  rclcpp::Publisher<NullspaceDiagnostics>::SharedPtr diagnostics_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<NullspaceDiagnostics>>
      diagnostics_rt_pub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      desired_cartesian_twist_pub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      filtered_cartesian_twist_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      nullspace_torque_pub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ee_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ee_force_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      velocity_ellipsoid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      force_ellipsoid_pub_;



  rclcpp_action::Server<CartesianTrajectoryExecution>::SharedPtr action_server_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CartesianTrajectoryGoal>>
      rt_trajectory_;

  visualization_msgs::msg::Marker make_linear_capability_ellipsoid(
      const JacobianMatrix &J,
      const Eigen::Matrix3d &R_base_ee,
      const rclcpp::Time &stamp,
      const std::string &name_space,
      int id,
      const std_msgs::msg::ColorRGBA &color) const;

  void update_joint_names();
  void update_stiffness_damping();
  void update_ns_optimizer_params();
  void fill_diagnostics(NullspaceDiagnostics &msg,
                        const rclcpp::Time &stamp,
                        const Eigen::Matrix<double, N_JOINTS, 1> &q,
                        const Eigen::Matrix<double, N_JOINTS, 1> &q_dot,
                        const JacobianMatrix &J,
                        const Eigen::Matrix3d &R_base_ee) const;

  Eigen::Affine3d get_fk(const Eigen::Matrix<double, N_JOINTS, 1> &q);
  JacobianMatrix get_jacobian(const Eigen::Matrix<double, N_JOINTS, 1> &q);

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const CartesianTrajectoryExecution::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandleCTE>);
  void handle_accepted(std::shared_ptr<GoalHandleCTE> goal_handle);
  void execute_trajectory(std::shared_ptr<GoalHandleCTE> goal_handle);
};

} // namespace lbr_impedance_control
