from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="pong_ros_server", executable="pong_ros_server"),
            Node(package="pong_pro_player", executable="pong_pro_player"),
            Node(package="pong_display", executable="pong_display"),
            Node(
                package="pong_mobile_player",
                executable="pong_mobile_player",
                parameters=[{"phyphox_url": "http://192.168.1.187/get"}],
                #parameters=[{"phyphox_url": "http://10.100.5.227/get"}],
                #parameters=[{"phyphox_url": "http://172.20.10.1/get"}],
            ),
        ]
    )
