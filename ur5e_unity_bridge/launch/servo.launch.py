import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            data = yaml.safe_load(file)
            if data and "/**" in data:
                return data["/**"]["ros__parameters"]
            return data
    except EnvironmentError:
        return None

def generate_launch_description():
    ur5e_bridge_pkg = FindPackageShare('ur5e_unity_bridge').find('ur5e_unity_bridge')
    ur_moveit_config_path = get_package_share_directory('ur_moveit_config')
    
    servo_yaml = load_yaml("ur5e_unity_bridge", "config/ur5e_servo.yaml")
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    
    # Kinematics parameters (standard ur_manipulator group)
    if "robot_description_kinematics" in kinematics_yaml:
        kinematics_dict = kinematics_yaml
    else:
        kinematics_dict = {"robot_description_kinematics": kinematics_yaml}

    # IMPORTANT: We use 'sed' to rename the robot to 'ur' in memory
    robot_description_content = Command(
        [
            "sh -c '", 
            FindExecutable(name="xacro"), " ", os.path.join(ur5e_bridge_pkg, "urdf", "ur5e.urdf"),
            " | sed \"s/robot name=\\\"ur5e\\\"/robot name=\\\"ur\\\"/\"'"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Set name:=ur to generate SRDF for robot 'ur' with group 'ur_manipulator'
    robot_description_semantic_content = Command(
        [FindExecutable(name="xacro"), " ", os.path.join(ur_moveit_config_path, "srdf", "ur.srdf.xacro"), " ", "name:=ur"]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            {"moveit_servo": servo_yaml},
            kinematics_dict,
            robot_description,
            robot_description_semantic,
            {"use_sim_time": False},
        ],
        output="screen",
        remappings=[
            ("~/joint_trajectory", "/scaled_joint_trajectory_controller/joint_trajectory"),
        ]
    )

    return LaunchDescription([servo_node])
