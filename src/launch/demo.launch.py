import os
from ament_index_python.packages import get_package_prefix
import launch.substitutions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():

    emit_shutdown_action = launch.actions.Shutdown(
        reason='launch is shutting down')

    config = os.path.join(
        get_package_prefix('behavior_tree_coordinator'),
        '..',
        '..',
        'src',
        'behavior-tree-coordinator',
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        # Node(
        #     package='petra_dummies',
        #     executable='ManipulatorDummy',
        #     name='ManipulatorDummy',
        #     output='log'
        # ),
        # Node(
        #     package='petra_dummies',
        #     executable='BatteryDummy',
        #     name='BatteryDummy',
        #     output='log'
        # ),
        Node(
            package='behavior_tree_coordinator',
            executable='detect_ball_node.py',
            name='detect_ball_node',
            output='log'
        ),
        Node(
            package='behavior_tree_coordinator',
            executable='Coordinator',
            # name='Coordinator',
            output='screen',
            parameters=[config],
            on_exit=[LogInfo(
                msg=["Coordinator has stopped. Stopping everything..."]), emit_shutdown_action]
        )
    ])
