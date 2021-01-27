import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():
	pkg_share = FindPackageShare("robot_tennis_description").find("robot_tennis_description")
	xacro_path = os.path.join(pkg_share,"urdf","robot.urdf.xacro")
	gazebo_path = os.path.join(get_package_share_directory('gazebo_ros'), "launch")
	gazebo_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([gazebo_path, '/gazebo.launch.py']))
	
	robot_state_publisher_node = Node(package='robot_state_publisher',executable='robot_state_publisher',parameters= [{"robot_description" : Command(["xacro"," ", xacro_path])}])
	spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', 'robot_1_description','-topic','/robot_description'])


	


	return LaunchDescription([robot_state_publisher_node,gazebo_node,spawn_entity])
