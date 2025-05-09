from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaacsim_turtlebot_controller',
            namespace='Lima',
            executable='isaac_tb3_controller',
            output='screen',
            parameters=[{'robot_name': 'Lima',
                         }]
        ),

        # Node(
        #     package='isaacsim_turtlebot_controller',
        #     namespace='Alpha',
        #     executable='isaac_tb3_controller',
        #     output='screen',
        #     parameters=[{'robot_name': 'Alpha',
        #                  }]
        # ),

    ])