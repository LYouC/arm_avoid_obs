import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnExecutionComplete

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("rm_65_description", package_name="rm_65_config").to_moveit_configs()
    avoid_obs = Node(package='obs_avoid', executable='avoid_obs',
                        parameters=[
                            {'use_sim_time': True},
                            # moveit_config.robot_description,
                            # moveit_config.robot_description_semantic,
                            # moveit_config.robot_description_kinematics,
                        ], 
                        emulate_tty=True,
                        output='screen')
    model_node = Node(package='obs_avoid', executable='test_model.py',
                        parameters=[
                            {'use_sim_time': True},
                        ], 
                        output='screen',
                        emulate_tty=True
                        )
    # finish_evt = RegisterEventHandler(
    #         event_handler=OnExecutionComplete(
    #             target_action=avoid_obs,
    #             on_completion=[model_node],
    #         )
    # )
    return LaunchDescription([
        avoid_obs,
        model_node,
    ])