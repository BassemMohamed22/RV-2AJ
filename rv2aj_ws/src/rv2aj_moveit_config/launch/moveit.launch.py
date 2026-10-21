"""MoveIt for the RV-2AJ: robot_state_publisher, move_group and optionally
RViz with the MotionPlanning display. "Plan & Execute" runs through
rv2aj_control's planner (its FollowJointTrajectory controller).

The robot's joint states come from the rv2aj driver (/rv2aj/joint_states),
so the model in RViz follows the real (or simulated) robot.

  ros2 launch rv2aj_moveit_config moveit.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

SNAP_GTK_VARS = ("GTK_PATH", "GDK_PIXBUF_MODULEDIR", "GDK_PIXBUF_MODULE_FILE", "LOCPATH",
                 "GIO_MODULE_DIR", "GTK_IM_MODULE_FILE", "SNAP_LIBRARY_PATH",
                 "GSETTINGS_SCHEMA_DIR", "GTK_EXE_PREFIX")


def moveit_config():
    description = os.path.join(get_package_share_directory("rv2aj_description"), "urdf", "rv2aj.urdf.xacro")
    return (
        MoveItConfigsBuilder("rv2aj", package_name="rv2aj_moveit_config")
        .robot_description(file_path=description)
        .robot_description_semantic(file_path="config/rv2aj.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )


def generate_launch_description():
    config = moveit_config()
    joint_states = LaunchConfiguration("joint_states_topic")
    rviz_config = os.path.join(get_package_share_directory("rv2aj_moveit_config"), "config", "moveit.rviz")
    gui_env = {k: v for k, v in os.environ.items() if k not in SNAP_GTK_VARS}

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("joint_states_topic", default_value="/rv2aj/joint_states"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[config.robot_description], remappings=[("joint_states", joint_states)]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[config.to_dict(), {
                 "allow_trajectory_execution": True,
                 # The robot stops at each waypoint, so it is slower than MoveIt's timing.
                 "trajectory_execution.execution_duration_monitoring": False,
             }],
             remappings=[("joint_states", joint_states)]),
        Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_config],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines, config.joint_limits],
             env=gui_env, condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
