import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
	pkg_share = get_package_share_directory("robot_tennis_description")
	model_file_urdf = os.path.join(pkg_share, "urdf", "robot.urdf")
	model_file_xacro = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")

	#conv from urdf to xacro
	urdf_file = xacro.process_file(model_file_xacro).toxml()
	#print(urdf_file)
	print(model_file_urdf)
	f = open(model_file_urdf, 'w')
	f.write(urdf_file)
	f.close()

	robot_state_publisher_node = Node(package="robot_state_publisher", node_executable="robot_state_publisher",  arguments=[model_file_urdf])
	#gazebo_node = Node(package="gazebo_ros", node_executable="gazebo.launch.py",  arguments=[model_file])

	# GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
	gazebo_spawn_entity_node = Node(package='gazebo_ros', 
		node_executable='spawn_entity.py', 
		arguments=['-entity', "rob", '-file', model_file_urdf, '-x', '0', '-y', '13', '-z', '0', '-Y', '1.57'], output='screen')


	catcher_ctrl_node = Node(package="catcher_control", node_executable="ctrl")

	#gazebo_spawn_entity_node = Node(package='gazebo_ros', node_executable='spawn_entity.py',
    #                   arguments=['-entity', 'demo', '-database', 'double_pendulum_with_base'],
    #""                   output='screen')

	return LaunchDescription([robot_state_publisher_node, gazebo_spawn_entity_node, catcher_ctrl_node])