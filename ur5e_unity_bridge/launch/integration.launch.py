from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('ur5e_unity_bridge')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. Bringup (Robot, Controllers, MoveGroup, Bridge Node)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ur_type': 'ur5e',
        }.items()
    )

    # 2. MoveIt Servo
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'servo.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        bringup_launch,
        servo_launch
    ])
