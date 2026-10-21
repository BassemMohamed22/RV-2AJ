"""Show the RV-2AJ model in RViz with joint sliders (no robot needed).

  ros2 launch rv2aj_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

SNAP_GTK_VARS = ("GTK_PATH", "GDK_PIXBUF_MODULEDIR", "GDK_PIXBUF_MODULE_FILE", "LOCPATH",
                 "GIO_MODULE_DIR", "GTK_IM_MODULE_FILE", "SNAP_LIBRARY_PATH",
                 "GSETTINGS_SCHEMA_DIR", "GTK_EXE_PREFIX")


def generate_launch_description():
    share = get_package_share_directory("rv2aj_description")
    description = ParameterValue(Command(["xacro ", os.path.join(share, "urdf", "rv2aj.urdf.xacro")]),
                                 value_type=str)
    gui_env = {k: v for k, v in os.environ.items() if k not in SNAP_GTK_VARS}
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", env=gui_env),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "display.rviz")],
             env=gui_env),
    ])
