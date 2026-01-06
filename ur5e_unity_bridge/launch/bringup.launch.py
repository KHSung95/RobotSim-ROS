import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 0. Launch Arguments
    ur_type = LaunchConfiguration('ur_type', default='ur5e')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 패키지 경로
    ur_moveit_config_pkg = FindPackageShare('ur_moveit_config').find('ur_moveit_config')
    ur5e_bridge_pkg = FindPackageShare('ur5e_unity_bridge').find('ur5e_unity_bridge')
    
    # 1. Robot Description (In-memory renaming to 'ur' to match moveit_config)
    robot_description_content = Command(
        [
            "sh -c '", 
            FindExecutable(name="xacro"), " ", os.path.join(ur5e_bridge_pkg, "urdf", "ur5e.urdf"),
            " | sed \"s/robot name=\\\"ur5e\\\"/robot name=\\\"ur\\\"/\"'"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Controller Manager & Config
    controllers_file = os.path.join(ur5e_bridge_pkg, "config", "ur5e_controllers.yaml")
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # 3. MoveIt2 Launch (This will use 'ur' as robot name by default)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_moveit_config_pkg, "launch", "ur_moveit.launch.py")
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "true",
            "use_fake_hardware": "true",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 4. Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    scaled_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # 5. Bridge & Rosbridge
    bridge_node = Node(
        package="ur5e_unity_bridge",
        executable="bridge_node",
        name="ur5e_unity_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        parameters=[{"port": 9090, "use_sim_time": use_sim_time}]
    )

    moveit_bridge_node = Node(
        package="ur5e_unity_bridge",
        executable="moveit_bridge",
        name="moveit_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    icp_server_node = Node(
        package="ur5e_unity_bridge",
        executable="icp_server",
        name="icp_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        ros2_control_node,
        robot_state_publisher_node,
        ur_moveit_launch,
        joint_state_broadcaster_spawner,
        scaled_joint_trajectory_controller_spawner,
        bridge_node,
        rosbridge_node,
        moveit_bridge_node,
        icp_server_node,
    ])
