from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mbedROS2',
            executable='mbed_motor_ctrl',
            name='mbedMotorCtrl'
        ),
    ])