from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='0',
            description='ID of the robot'
        ),
        Node(
            package='fluttergoalpose',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False  
            }],
            arguments=[LaunchConfiguration('robot_id')]
        )
    ])

# ros2 launch fluttergoalpose goalposer_launch.py robot_id:=my_robot_id
# ros2 run fluttergoalpose goal_publisher my_robot_id