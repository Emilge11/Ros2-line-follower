import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('line_follower')

    world = os.path.join(package_dir, 'worlds', 'e-puck_botstudio_with_floor_sensors.wbt')
    params = os.path.join(package_dir, 'config', 'line_follower_params.yaml')

    webots = WebotsLauncher(world=world)

    line_follower_node = Node(
        package='line_follower',
        executable='line_follower_node',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([
        webots,
        line_follower_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])