"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'aruco_mission_2.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    return LaunchDescription([
          # Launch Gazebo, loading tello.world
          ExecuteProcess(cmd=[
               'gazebo',
               '--verbose',
               '-s', 'libgazebo_ros_init.so',  # Publish /clock
               '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
               world_path
          ], output='screen'),

        # Spawn tello.urdf 15s after gazebo has been launched (for my wsl machine it is required...)
          TimerAction(
               period = 1.0,
               actions = [
                    Node(
                         package = "gazebo_ros",
                         executable = "spawn_entity.py",
                         arguments = [
                              "-entity", "drone_ipsa",
                              "-topic", "/robot_description",
                              "-x", "0.0"
                         ]
                    )
               ]
          ),          

          # Publish static transforms
          Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               arguments=[urdf_path]),

          # Joystick driver, generates /namespace/joy messages
          #Node(package='joy', executable='joy_node', output='screen',
          #     namespace=ns),

          # Joystick controller, generates /namespace/cmd_vel messages
          Node(package='tello_driver', executable='tello_joy_main', output='screen',
               namespace=ns),
    ])
