from launch import LaunchDescription
from launch.actions import TimerAction
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
        TimerAction(
            period=2.0, # delay
            actions=[
                Node(
                    package='lexus_bringup',
                    executable='path_steering_and_kmph',
                    name='path_steering_and_kmph_d',
                    output='screen',
                    parameters=[{"marker_color": "b", "path_size": 550}]
                ), 
            ]),
        # Control nodes   
        TimerAction(
            period=2.0, # delay
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("lexus_bringup"), '/launch', '/speed_control.launch.py'])
                ),
            ]),     
        Node(
            package='wayp_plan_tools',
            executable='waypoint_loader',
            name='wayp_load',
            output='screen',
            parameters=[
                {"file_dir": "/mnt/bag/waypoints/"},
                #{"file_name": "gyor1.csv"}],
                #{"file_name": "zala02unitest.csv"}],
                #{"file_name": "zala03uniteljeskor.csv"}],
                {"file_name": "zala01uni.csv"}],
                #{"file_name": "zala04smartteljeskor.csv"}],

        ),
        Node(
            package='wayp_plan_tools',
            executable='waypoint_to_target',
            name='wayp2target',
            output='screen',
        ),
        # ros2 launch wayp_plan_tools singe_goal_pursuit.launch.py 
        TimerAction(
            period=3.0, # delay
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("wayp_plan_tools"), '/launch', '/single_goal_pursuit.launch.py'])
                ),        
            ]),  
        # ros2 run rqt_reconfigure rqt_reconfigure 
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_rec',
            #output='screen',
        ),
    ])
