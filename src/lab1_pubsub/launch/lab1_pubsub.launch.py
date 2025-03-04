from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pubsub',
            executable='talker_node',
            name='talker_node'
        ),
        Node(
            package='lab1_pubsub',
            executable='listener_node',
            name='listener_node'
        ),
    ])
