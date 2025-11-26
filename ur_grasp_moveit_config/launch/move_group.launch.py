#from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_move_group_launch


#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("name", package_name="ur_grasp_moveit_config").to_moveit_configs()
#    return generate_move_group_launch(moveit_config)

from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="ur_grasp_moveit_config").to_moveit_configs()
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("ur_grasp_moveit_config")
        .yaml("config/ur_servo.yaml")
        .to_dict()
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "ur_manipulator"}
    
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    
    return LaunchDescription(
        [move_group_node,
        servo_node,]
    )
