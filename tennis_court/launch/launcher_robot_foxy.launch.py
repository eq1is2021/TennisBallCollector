import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    #Spawn robot
    pkg_share_2 = FindPackageShare("robot_tennis_description").find("robot_tennis_description")
    xacro_path = os.path.join(pkg_share_2,"urdf","robot.urdf.xacro")

    robot_state_publisher_node = Node(package='robot_state_publisher',executable='robot_state_publisher',parameters= [{"robot_description" : Command(["xacro"," ", xacro_path])}])
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', 'robot_tennis_description','-topic','robot_description','-x','0','-y','13','-Y','1.57'])
    #Load catcher ctrl
    catcher_ctrl_node = Node(package='catcher_control',executable='ctrl')
    return LaunchDescription([
        robot_state_publisher_node,
        spawn_entity,
        catcher_ctrl_node

    ])
