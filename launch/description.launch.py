# Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from os import path
from os import getenv
from xacro import process_file
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    ## ***** File paths ******
    pkg_share = FindPackageShare('platform_description').find('platform_description')
    urdf_dir = path.join(pkg_share, 'urdf')
    xacro_file = path.join(urdf_dir, 'platform.xacro')
    doc = process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    # Ignition gazebo
    pkg_ros_gazebo = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gazebo, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-v 0 -r test_copy.sdf'}.items(),
    ) 

    ## ***** Nodes *****
    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}],
        output = 'screen'
        )

    joint_state_publisher = Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        parameters=[
            {'use_sim_time': True}],
        output = 'screen'
        )

    # Spawn
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', 'my_custom_model',
                    '-x', '1.2',
                    '-z', '2.3',
                    '-Y', '3.4',
                    '-topic', '/robot_description'],
                 output='screen')
     # Bridge
    bridge = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments = ['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',],
        parameters = [{'use_sim_time': True}],
        output='screen')
    
    return LaunchDescription([
      gazebo,
      robot_state_publisher,
      joint_state_publisher,
      spawn,
      bridge,
    ])
# EOF