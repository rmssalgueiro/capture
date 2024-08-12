#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # -----------------------------------------------
    # -------------- SIMULATOR LAUNCH  --------------
    # -----------------------------------------------

    # Launch an empty world in gazebo
    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gazebo'), 'launch/worlds/empty.launch.py')),
        launch_arguments={
            'gui': 'true',
            }.items())

    # ----------------------------------------
    # ---- RETURN THE LAUNCH DESCRIPTION -----
    # ----------------------------------------
    return LaunchDescription([gazebo_launch_file,])