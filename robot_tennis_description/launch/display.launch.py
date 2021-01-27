import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = FindPackageShare("robot_tennis_description").find("robot_tennis_description")
    xacro_path = os.path.join(pkg_share,"urdf","robot.urdf.xacro")
    rviz_config_file = os.path.join(pkg_share,"config","display.rviz")
    robot_state_publisher_node = Node(package="robot_state_publisher",executable="robot_state_publisher",parameters= [{"robot_description" : Command(["xacro"," ", xacro_path])}])
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d",rviz_config_file])
    joint_state_publisher_node = Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui")
    
    return LaunchDescription([robot_state_publisher_node,joint_state_publisher_node,rviz_node])
