#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # ----------------------------------------
    # ---- DECLARE THE LAUNCH ARGUMENTS ------
    # ----------------------------------------

    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')

    # -----------------------------------------------
    # -------------- SIMULATOR LAUNCH  --------------
    # -----------------------------------------------

    # Launch an empty world in gazebo
    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gazebo'), 'launch/worlds/empty.launch.py')),
        launch_arguments={
            'gui': 'true',
            }.items())
    
    iris_fpv_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus_gazebo'), 'launch/vehicles/iris.launch.py')),
        launch_arguments={  # ENU coordinates
            'x':  '0.0',
            'y':  '0.0',
            'z':  '0.0',
            'launch_pegasus': 'false',
        }.items())
    
    # -----------------------------------------------
    # ---------- CONTROL SYSTEM LANCH  --------------
    # -----------------------------------------------

    # Define the drone MAVLINK IP and PORT
    mav_connection_arg = DeclareLaunchArgument('connection', default_value='udp://:14540', description='The interface used to connect to the vehicle')
    
    # Define the drone MAVLINK forward ips and ports (by default in simulation we do not need to forward mavlink connections)
    mavlink_forward_arg = DeclareLaunchArgument('mavlink_forward', default_value="['']", description='A list of ips where to forward mavlink messages')

    # Define which file to use for the drone parameters
    drone_params_file_arg = DeclareLaunchArgument(
        'drone_params', 
        default_value=os.path.join(get_package_share_directory('capture'), 'config', 'simulation.yaml'),
        description='The directory where the drone parameters such as mass, thrust curve, etc. are defined')
    
    # Call MAVLINK interface package launch file 
    mavlink_interface_launch_file = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'), 
            'namespace': LaunchConfiguration('vehicle_ns'),
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': LaunchConfiguration('connection'),
            'mavlink_forward': LaunchConfiguration('mavlink_forward')
        }.items(),
    )

    # Call autopilot package launch file
    autopilot_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
        # Define costume launch arguments/parameters used 
        launch_arguments={
            'id': LaunchConfiguration('vehicle_id'),
            'namespace': LaunchConfiguration('vehicle_ns'),
            'autopilot_yaml': LaunchConfiguration('drone_params'),
        }.items(),
    )

    # ----------------------------------------
    # ---- RETURN THE LAUNCH DESCRIPTION -----
    # ----------------------------------------
    return LaunchDescription([
        # Launch arguments
        id_arg, 
        namespace_arg,
        # Launch files for simulation
        gazebo_launch_file,
        iris_fpv_launch_file,
        # Launch files for the control system
        mav_connection_arg,
        mavlink_forward_arg,
        drone_params_file_arg,
        mavlink_interface_launch_file,
        autopilot_launch_file,
    ])