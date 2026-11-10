"""
lbr_JIC_CIC_gazebo.launch.py

Combined simulation launch for the knee-task workflow.

Starts:
- Gazebo and the simulation clock bridge.
- Robot state publisher and robot spawning.
- Joint-state broadcaster.
- Joint impedance controller active.
- Cartesian impedance controller loaded but inactive.
- RViz with the packaged visualization configuration.

The integration executor performs initialization, controller switching,
and Cartesian task replay. This launch does not execute a knee task.

The supported initial demonstration uses med14, robot_name=lbr,
and the no-gravity world. Other exposed model/name combinations are not
yet validated by the project integration workflow.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("lbr_impedance_control")
    rviz_config = os.path.join(pkg, "config", "kuka_knee_task_viz.rviz")
    world_no_gravity = os.path.join(pkg, "worlds", "empty_no_gravity.sdf")

    return LaunchDescription([
        DeclareLaunchArgument(name="model", default_value="med14",
                               choices=["iiwa7", "iiwa14", "med7", "med14"]),
        DeclareLaunchArgument(name="robot_name", default_value="lbr"),
        DeclareLaunchArgument(name="gravity", default_value="false",
                            description=("Enable Gazebo gravity. The validated initial replay target "
                                "is gravity=false; gravity=true requires separate validation."),),
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    FindExecutable(name="xacro"), " ",
                    PathSubstitution(FindPackageShare("lbr_ros2_control"))
                    / "system_integration" / LaunchConfiguration("model")
                    / LaunchConfiguration("model"), ".xacro",
                    " robot_name:=", LaunchConfiguration("robot_name"),
                    " mode:=gazebo",
                    " system_config_path:=",
                    PathSubstitution(FindPackageShare("lbr_impedance_control"))
                    / "config" / "lbr_system_config.yaml",
                    " initial_joint_positions_path:=",
                    PathSubstitution(FindPackageShare("lbr_ros2_control"))
                    / "config" / "initial_joint_positions.yaml",
                    " joint_limits_path:=",
                    PathSubstitution(FindPackageShare("lbr_ros2_control"))
                    / "system_integration" / LaunchConfiguration("model")
                    / "joint_limits.yaml",
                ]),
                "use_sim_time": True,
            }],
            namespace=LaunchConfiguration("robot_name"),
        ),

        # No-gravity world (default)
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare("ros_gz_sim")) / "launch" / "gz_sim.launch.py",
            launch_arguments={"gz_args": f"-r {world_no_gravity}"}.items(),
            condition=UnlessCondition(LaunchConfiguration("gravity")),
        ),
        # Normal gravity world
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare("ros_gz_sim")) / "launch" / "gz_sim.launch.py",
            launch_arguments={"gz_args": "-r empty.sdf"}.items(),
            condition=IfCondition(LaunchConfiguration("gravity")),
        ),

        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),

        Node(
            package="ros_gz_sim", executable="create",
            arguments=[
                "-topic", "robot_description",
                "-name", LaunchConfiguration("robot_name"),
                "-allow_renaming",
                "-x", "0.0", "-y", "0.0", "-z", "0.0",
                "-R", "0.0", "-P", "0.0", "-Y", "0.0",
            ],
            output="screen",
            namespace=LaunchConfiguration("robot_name"),
        ),

        # joint_state_broadcaster + JIC active, CIC loaded but INACTIVE.
        # Spawner's default behaviour is to activate whatever it spawns, so
        # CIC is spawned with --inactive and activated later at runtime via
        # switch_controller (called from go_home_then_flexion70.py).
        Node(
            package="controller_manager", executable="spawner",
            output="screen",
            arguments=[
                "--controller-manager", "controller_manager",
                "joint_state_broadcaster",
                "joint_impedance_controller",
            ],
            namespace=LaunchConfiguration("robot_name"),
        ),
        Node(
            package="controller_manager", executable="spawner",
            output="screen",
            arguments=[
                "--controller-manager", "controller_manager",
                "--inactive",
                "cartesian_impedance_controller",
            ],
            namespace=LaunchConfiguration("robot_name"),
        ),



        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],  # <-- THIS loads your saved config
            parameters=[{
                "robot_description": Command([
                    FindExecutable(name="xacro"), " ",
                    PathSubstitution(FindPackageShare("lbr_ros2_control"))
                    / "system_integration" / LaunchConfiguration("model")
                    / LaunchConfiguration("model"), ".xacro",
                    " robot_name:=", LaunchConfiguration("robot_name"),
                    " mode:=gazebo",
                    " system_config_path:=",
                    PathSubstitution(FindPackageShare("lbr_impedance_control"))
                    / "config" / "lbr_system_config.yaml",
                    " initial_joint_positions_path:=",
                    PathSubstitution(FindPackageShare("lbr_ros2_control"))
                    / "config" / "initial_joint_positions.yaml",
                    " joint_limits_path:=",
                    PathSubstitution(FindPackageShare("lbr_ros2_control"))
                    / "system_integration" / LaunchConfiguration("model")
                    / "joint_limits.yaml",
                ]),
                "use_sim_time": True,
            }],
            namespace=LaunchConfiguration("robot_name"),
        ),

        
    ])
