from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Drivers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("lexus_bringup"), '/launch/drivers', '/gps_duro_reference.launch.py'])
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                FindPackageShare("lexus_bringup"), '/launch/drivers', '/can_pacmod3.launch.xml'])
        ),
        # General nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("lexus_bringup"), '/launch', '/tf_static.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("lexus_bringup"), '/launch', '/3d_marker.launch.py'])
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                FindPackageShare("lexus_bringup"), '/launch', '/foxglove_bridge_launch.xml'])
        ),
        Node(
            package='lexus_bringup',
            executable='path_steering_and_kmph',
            name='path_steering_and_kmph_d',
            output='screen',
            parameters=[{"marker_color": "y", "path_size": 550}]
        ),
        Node(
            package='wayp_plan_tools',
            executable='waypoint_saver',
            name='wayp_saver',
            output='screen',
            parameters=[
                {"file_dir": "/mnt/bag/waypoints/"},
                {"file_name": "gyor1.csv"}],
        )     
    ])
