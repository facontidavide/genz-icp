# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
# Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare("genz_icp")
    return LaunchDescription(
        [
            # Offline-specific parameters
            DeclareLaunchArgument("bag_path", description="Path to MCAP/db3 bag file"),
            DeclareLaunchArgument("pointcloud_topic", description="PointCloud2 topic name in bag"),
            # ROS 2 parameters
            DeclareLaunchArgument("visualize", default_value="true"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value=""),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            # GenZ-ICP parameters
            DeclareLaunchArgument("deskew", default_value="false"),
            DeclareLaunchArgument("constrain_to_2d", default_value="true"),
            DeclareLaunchArgument("max_range", default_value="20.0"),
            DeclareLaunchArgument("min_range", default_value="0.6"),
            DeclareLaunchArgument("voxel_size", default_value="0.3"),
            DeclareLaunchArgument("map_cleanup_radius", default_value="100.0"),
            DeclareLaunchArgument("desired_num_voxelized_points", default_value="2000"),
            DeclareLaunchArgument("planarity_threshold", default_value="0.2"),
            DeclareLaunchArgument("max_points_per_voxel", default_value="1"),
            DeclareLaunchArgument("max_num_iterations", default_value="100"),
            DeclareLaunchArgument("convergence_criterion", default_value="0.0001"),
            DeclareLaunchArgument("initial_threshold", default_value="2.0"),
            DeclareLaunchArgument("min_motion_th", default_value="0.1"),
            DeclareLaunchArgument("config_file", default_value=""),
            Node(
                package="genz_icp",
                executable="offline_node",
                name="offline_node",
                output="screen",
                parameters=[
                    {
                        "bag_path": LaunchConfiguration("bag_path"),
                        "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "deskew": LaunchConfiguration("deskew"),
                        "constrain_to_2d": LaunchConfiguration("constrain_to_2d"),
                        "max_range": LaunchConfiguration("max_range"),
                        "min_range": LaunchConfiguration("min_range"),
                        "voxel_size": LaunchConfiguration("voxel_size"),
                        "map_cleanup_radius": LaunchConfiguration("map_cleanup_radius"),
                        "desired_num_voxelized_points": LaunchConfiguration("desired_num_voxelized_points"),
                        "planarity_threshold": LaunchConfiguration("planarity_threshold"),
                        "max_points_per_voxel": LaunchConfiguration("max_points_per_voxel"),
                        "max_num_iterations": LaunchConfiguration("max_num_iterations"),
                        "convergence_criterion": LaunchConfiguration("convergence_criterion"),
                        "initial_threshold": 2.0,
                        "min_motion_th": 0.1,
                        "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
                        "visualize": LaunchConfiguration("visualize"),
                        "config_file": LaunchConfiguration("config_file"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output={"both": "log"},
                arguments=["-d", PathJoinSubstitution([current_pkg, "rviz", "genz_icp_ros2.rviz"])],
                condition=IfCondition(LaunchConfiguration("visualize")),
            ),
        ]
    )
