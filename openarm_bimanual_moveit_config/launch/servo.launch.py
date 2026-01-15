import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("openarm_bimanual_moveit_config")
    desc_share = get_package_share_directory("openarm_description")

    # -----------------------------
    # Robot description (URDF / SRDF / kinematics)
    # -----------------------------
    urdf_path = os.path.join(desc_share, "urdf", "openarm_bimanual_sim.urdf")
    srdf_path = os.path.join(pkg_share, "config", "openarm_bimanual.srdf")
    kin_path = os.path.join(pkg_share, "config", "kinematics.yaml")

    with open(urdf_path, "r") as f:
        robot_description = {"robot_description": f.read()}
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}
    with open(kin_path, "r") as f:
        robot_description_kinematics = {"robot_description_kinematics": yaml.safe_load(f)}

    # -----------------------------
    # Servo parameter files（保持嵌套 moveit_servo.ros__parameters）
    # -----------------------------
    servo_left_yaml_path = os.path.join(pkg_share, "config", "servo_left.yaml")
    servo_right_yaml_path = os.path.join(pkg_share, "config", "servo_right.yaml")
    with open(servo_left_yaml_path, "r") as f:
        servo_left_raw = yaml.safe_load(f) or {}
    with open(servo_right_yaml_path, "r") as f:
        servo_right_raw = yaml.safe_load(f) or {}

    # 取出 moveit_servo.ros__parameters 平铺给节点，避免多包一层导致参数名错误
    servo_left_params = {
        "moveit_servo": servo_left_raw.get("moveit_servo", {}).get("ros__parameters", {})
    }
    servo_right_params = {
        "moveit_servo": servo_right_raw.get("moveit_servo", {}).get("ros__parameters", {})
    }

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # -----------------------------
    # Left Servo
    # -----------------------------
    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_servo_node",
        namespace="left_servo_node",
        output="screen",
        parameters=[
            servo_left_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # -----------------------------
    # Right Servo
    # -----------------------------
    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_servo_node",
        namespace="right_servo_node",
        output="screen",
        parameters=[
            servo_right_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            left_servo_node,
            right_servo_node,
        ]
    )