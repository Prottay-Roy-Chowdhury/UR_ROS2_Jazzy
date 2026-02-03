"""Launch file for IAAC UR10e robot with MoveIt and visualization nodes."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the 'sim' argument to toggle between simulation and real robot mode
    declared_arguments = [
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="Launch in simulation mode if true, real robot mode if false",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Specify the joint controller to use",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
            ],
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="iaac_ur_description",
            description="Package containing the URDF description of the robot.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="deco2_mosaic.xacro",
            description="URDF/Xacro description file of the robot.",
        ),
    ]

    # Define launch configurations for use in the IncludeLaunchDescription
    sim = LaunchConfiguration("sim")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    ur_description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    # Build an absolute path to the xacro so the UR driver can always find it
    # (Fixes Jazzy where the included launch may call xacro with a relative filename.)
    description_file_path = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "urdf", description_file]
    )

    # Define the path to the UR and MoveIt launch files
    ur_bringup_launch_file = os.path.join(
        FindPackageShare("ur_robot_driver").find("ur_robot_driver"),
        "launch",
        "ur10e.launch.py",
    )

    moveit_launch_file = os.path.join(
        FindPackageShare("ur_moveit_config").find("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py",
    )

    # Define the arguments for both simulation and real robot modes
    # Note: Jazzy UR driver uses use_mock_hardware/mock_sensor_commands (not use_fake_hardware/fake_sensor_commands).
    sim_arguments = {
        "initial_joint_controller": initial_joint_controller,
        "robot_ip": "0.0.0.0",
        "use_mock_hardware": "true",
        "mock_sensor_commands": "true",
        "activate_joint_controller": "true",
        "launch_rviz": "false",
        "description_package": ur_description_package,
        "description_file": description_file_path,
    }

    real_robot_arguments = {
        "initial_joint_controller": initial_joint_controller,
        "robot_ip": "192.168.56.101",
        "use_mock_hardware": "false",
        "mock_sensor_commands": "false",
        "activate_joint_controller": "true",
        "launch_rviz": "false",
        "description_package": ur_description_package,
        "description_file": description_file_path,
    }

    moveit_arguments = {
        "ur_type": "ur10e",
        "description_package": ur_description_package,
        "description_file": description_file_path,
    }

    # moveit_arguments = {
    #     "ur_type": "ur10e",
    #     "launch_rviz": "true",          # or true if you want
    #     "use_sim_time": sim,             # makes MoveIt follow sim clock if sim:=true
    #     "launch_servo": "false",         # optional
    # }


    # Define the IncludeLaunchDescription with conditional arguments
    ur_bringup_launch = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_bringup_launch_file),
                launch_arguments=sim_arguments.items(),
                condition=IfCondition(sim),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_bringup_launch_file),
                launch_arguments=real_robot_arguments.items(),
                condition=UnlessCondition(sim),
            ),
        ]
    )

    # Launch moveit
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments=moveit_arguments.items(),
    )

    # Launch the pose visualization node
    visualize_pose_srv_node = Node(
        package="ur_commander",
        executable="commander_viz.py",
        name="visualize_pose_srv_node",
        output="screen",
    )

    # Return the full launch description
    return LaunchDescription(
        declared_arguments + [ur_bringup_launch, moveit_launch, visualize_pose_srv_node]
    )