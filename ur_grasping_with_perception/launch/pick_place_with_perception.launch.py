import os

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "box_name",
            description="box name in gazebo simulation.",
            default_value="box_1",
        )
    )

    
    box_name = LaunchConfiguration("box_name")
    
    #moveit_config = MoveItConfigsBuilder("my").to_moveit_configs()
    moveit_config = MoveItConfigsBuilder("name", package_name="ur_grasp_moveit_config").to_moveit_configs()
    #moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    #generate_move_group_launch(moveit_config)

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="mathys",
        package="ur_grasping_with_perception",
        executable="mathys",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    box_descrition_file = PathJoinSubstitution(
       [FindPackageShare("ur_grasping_example"), "urdf", "grasp_box.urdf" ]
    )
    # Spawn cube
    gazebo_spawn_cube = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_cube",
        #arguments=["-entity", "ur", "-topic", "robot_description"],
        arguments=['-file', box_descrition_file,
                                   '-x', "0.34", '-y', "0.13", '-z', "0.1",
                                   '-entity', box_name],
        output="screen",
    )

    return LaunchDescription(
        [
        #gazebo_spawn_cube,
        moveit_cpp_node,
        ]
    )
