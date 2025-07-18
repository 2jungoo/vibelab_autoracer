from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vibelab_autoracer',
            executable='gstcam_publisher',
            name='gstcam_pub_node',
            output='screen'
        ),
        Node(
            package='vibelab_autoracer',
            executable='cmd_vel_servo',
            name='cmd_vel_servo_node',
            output='screen'
        ),
        Node(
            package='vibelab_autoracer',
            executable='map_scan_tf_node',
            name='map_scan_tf_node',
            output='screen'
        ),
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlda_laser_publisher',
            output='screen'
        ),
    ])

