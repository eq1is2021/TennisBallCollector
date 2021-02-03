import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

ROS_DISTRO_ELOQUENT = "eloquent"
ROS_DISTRO_FOXY = "foxy"
ROS_DISTRO = os.environ.get("ROS_DISTRO")


def generate_launch_description():

    path = os.path.join(get_package_share_directory('tennis_court'), "launch")
    #Spawn court
    court_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([path, '/tennis_court.launch.py']))
    
    #Spawn robot
    if(ROS_DISTRO == ROS_DISTRO_FOXY):
        robot_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([path, '/launcher_robot_foxy.launch.py']))
    else:
        robot_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([path, '/launcher_robot_eloquent.launch.py']))
    return LaunchDescription([
        court_node,
	robot_node
    ])
