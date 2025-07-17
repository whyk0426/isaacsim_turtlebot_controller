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
                         'x_offset' : 0.0,
                         'y_offset' : 0.0,
                         'theta_offset' : 0.0,
                         }]
        ),

        Node(
            package='isaacsim_turtlebot_controller',
            namespace='Alpha',
            executable='isaac_tb3_controller',
            output='screen',
            parameters=[{'robot_name': 'Alpha',
                         'x_offset' : -1.0,     #-1.0
                         'y_offset' : 2.0,          #2.0
                         'theta_offset' : 0.0,
                         }]
        ),

    ])