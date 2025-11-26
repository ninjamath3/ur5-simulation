from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    box_name = LaunchConfiguration("box_name")
    box_namespace = LaunchConfiguration("box_namespace")
    box_package = LaunchConfiguration("box_package")
    box_file = LaunchConfiguration("box_file")
    launch_rviz = LaunchConfiguration("launch_rviz")


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(box_package), "urdf", box_file]
            ),
        ]
    )
            
    robot_description_content=launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace = box_namespace,
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
#        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )
    
    joint_state_publisher_node = Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                namespace = box_namespace,

    )

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_box",
        #arguments=["-entity", "ur", "-topic", "robot_description"], <!> 0.34 0.13 0.1
        arguments=['-robot_namespace', box_namespace, '-topic', ['/', box_namespace, '/robot_description'],
                                   '-x', "0.34", '-y', "0.13", '-z', "0.1", 
                                   '-entity', box_name],
        output="screen",
    )
#    gazebo_spawn_robot = Node(
#        package="gazebo_ros",
#        executable="spawn_entity.py",
#        name="spawn_box",
#        #arguments=["-entity", "ur", "-topic", "robot_description"],
#        arguments=['-robot_namespace', box_namespace, '-topic', ['/', box_namespace, '/robot_description'],
#                                   '-x', "0", '-y', "0", '-z', "0",
#                                   '-entity', box_name],
#        output="screen",
#    )
    
#   gazebo_spawn_robot = Node(
#        package="gazebo_ros",
#        executable="spawn_entity.py",
#        name="spawn_apriltag",
#        #arguments=["-entity", "ur", "-topic", "robot_description"],
#        arguments=['-robot_namespace', 'box', '-topic', '/box/robot_description',
#                                   '-x', "0.34", '-y', "0.13", '-z', "0.1",
#                                   '-entity', 'box_demo'],
#        output="screen",
#    )


    nodes_to_start = [
    	robot_state_publisher_node,
    	joint_state_publisher_node,
#        gazebo,
        gazebo_spawn_robot,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "box_name",
            description="box name in gazebo simulation.",
            default_value="box_N",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "box_namespace",
            description="robot namespace in gazebo simulation.",
            default_value="box",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "box_package",
            default_value="simple_box_with_marker_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "box_file",
            default_value="simple_box.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])



#from launch import LaunchDescription
#from launch.actions import (
#    DeclareLaunchArgument,
#    IncludeLaunchDescription,
#    OpaqueFunction,
#    RegisterEventHandler,
#)
#from launch.conditions import IfCondition, UnlessCondition
#from launch.event_handlers import OnProcessExit
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
#from launch_ros.actions import Node
#from launch_ros.substitutions import FindPackageShare
#import launch_ros.descriptions
#import os
#from ament_index_python.packages import get_package_share_directory


#def generate_launch_description():

#    declared_arguments = []
    
#    declared_arguments.append(
#        DeclareLaunchArgument(
#            "box_name",
#            description="box name in gazebo simulation.",
#            default_value="box_1",
#        )
#    )

#    declared_arguments.append(
#        DeclareLaunchArgument(
#            "box_namespace",
#            description="robot namespace in gazebo simulation.",
#            default_value="box",
#        )
#    )
    
#    declared_arguments.append(
#        DeclareLaunchArgument(
#            "box_package",
#            default_value="simple_box_with_marker_description",
#            description="Description package with robot URDF/XACRO files. Usually the argument \
#        is not set, it enables use of a custom description.",
#        )
#    )
    
#    declared_arguments.append(
#        DeclareLaunchArgument(
#            "box_file",
#            default_value="simple_box.urdf.xacro",
#            description="URDF/XACRO description file with the robot.",
#        )
#    )

#    declared_arguments.append(
#        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
#    )

    
#    box_name = LaunchConfiguration("box_name")
#    box_namespace = LaunchConfiguration("box_namespace")
#    box_package = LaunchConfiguration("box_package")
#    box_file = LaunchConfiguration("box_file")
#    launch_rviz = LaunchConfiguration("launch_rviz")

    
#    box_descrition_file = PathJoinSubstitution(
#       [FindPackageShare("ur_grasping_example"), "urdf", "grasp_box.urdf" ]
#    )

    # Spawn cube
#    gazebo_spawn_cube = Node(
#        package="gazebo_ros",
#        executable="spawn_entity.py",
#        name="spawn_cube",
        #arguments=["-entity", "ur", "-topic", "robot_description"],
#        arguments=['-file', box_descrition_file,
#                                   '-x', "0.34", '-y', "0.13", '-z', "0.1",
#                                   '-entity', box_name],
#        output="screen",
#    )

#    return LaunchDescription(
#        [
#        gazebo_spawn_cube,
#        moveit_cpp_node,
#        ]
#    )
    


#    box_description_content = Command(
#        [
#            PathJoinSubstitution([FindExecutable(name="xacro")]),
#            " ",
#            PathJoinSubstitution(
#                [FindPackageShare("simple_box_with_marker_description"), "urdf", "simple_box.urdf.xacro"]
#            ),
#        ]
#    )
            
#    box_description_content=launch_ros.descriptions.ParameterValue(box_description_content, value_type=str)

#    box_description = {"robot_description": box_description_content}

#    robot_state_publisher_node = Node(
#        package="robot_state_publisher",
#        executable="robot_state_publisher",
#        output="both",
#        namespace = 'box',
#        parameters=[{"use_sim_time": True}, box_description],
#    )

#    rviz_node = Node(
#        package="rviz2",
#        executable="rviz2",
#        name="rviz2",
#        output="log",
##        arguments=["-d", rviz_config_file],
#        condition=IfCondition(launch_rviz),
#    )
    
#    joint_state_publisher_node = Node(
#                package="joint_state_publisher",
#                executable="joint_state_publisher",
#                name="joint_state_publisher",
#                namespace = 'box',

#    )

    #Gazebo nodes
#    gazebo = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource(
#            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
#        ),
#    )

    # Spawn robot
#    gazebo_spawn_robot = Node(
#        package="gazebo_ros",
#        executable="spawn_entity.py",
#        name="spawn_apriltag",
#        #arguments=["-entity", "ur", "-topic", "robot_description"],
#        arguments=['-robot_namespace', 'box', '-topic', '/box/robot_description',
#                                   '-x', "0.34", '-y', "0.13", '-z', "0.1",
#                                   '-entity', 'box_demo'],
#        output="screen",
#    )
    
#    return LaunchDescription(
#        [
#    	 robot_state_publisher_node,
#    	 joint_state_publisher_node,
#         gazebo,
#         gazebo_spawn_robot,
#         rviz_node,
#        ]
#    )
    
    
    
