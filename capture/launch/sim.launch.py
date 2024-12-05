#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    '''
    iris_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gazebo'), 'launch/vehicles/iris.launch.py')),
        launch_arguments={  # ENU coordinates
            'x':  '3.0',
            'y':  '0.0',
            'z':  '0.0',
            'launch_pegasus': 'false',
            'vehicle_id': '1',
        }.items())
    '''
    target_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gazebo'), 'launch/vehicles/iris.launch.py')),
        launch_arguments={  # ENU coordinates
            'x':  '0.0',
            'y':  '0.0',
            'z':  '0.0',
            'launch_pegasus': 'false',
            'vehicle_id': '2',
        }.items())
    
    # -----------------------------------------------
    # ---------- CONTROL SYSTEM LANCH  --------------
    # -----------------------------------------------

    # Define which file to use for the drone parameters
    drone_params_file_arg = DeclareLaunchArgument(
        'drone_params', 
        default_value=os.path.join(get_package_share_directory('capture'), 'config', 'simulation.yaml'),
        description='The directory where the drone parameters such as mass, thrust curve, etc. are defined')
    '''
    # Call MAVLINK interface package launch file 
    mavlink_interface_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'vehicle_id': '1', 
            'namespace': 'drone',
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': 'udp://:14540',
            'mavlink_forward': "['']"
        }.items(),
    )
    '''
    # Call MAVLINK interface package launch file 
    mavlink2_interface_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'vehicle_id': '2', 
            'namespace': 'drone',
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': 'udp://:14541',
            'mavlink_forward': "['']"
        }.items(),
    )
    '''
    # Call autopilot package launch file
    autopilot_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
        # Define costume launch arguments/parameters used 
        launch_arguments={
            'vehicle_id': '1',
            'namespace': 'drone',
            'autopilot_yaml': LaunchConfiguration('drone_params'),
        }.items(),
    )
    '''
    # Call autopilot package launch file
    autopilot2_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
        # Define costume launch arguments/parameters used 
        launch_arguments={
            'vehicle_id': '2',
            'namespace': 'drone',
            'autopilot_yaml': LaunchConfiguration('drone_params'),
        }.items(),
    )
    
    # ----------------------------------------
    # ---- RETURN THE LAUNCH DESCRIPTION -----
    # ----------------------------------------
    return LaunchDescription([
        # Launch files for simulation
        gazebo_launch_file,
        #iris_launch_file,
        target_launch_file,
        #files for the control system
        drone_params_file_arg,
        #mavlink_interface_launch_file,
        mavlink2_interface_launch_file,
        #autopilot_launch_file,
        autopilot2_launch_file
    ])