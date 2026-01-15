import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def load_yaml(package_name: str, relative_path: str):
    pkg_path = get_package_share_directory(package_name)
    yaml_path = os.path.join(pkg_path, relative_path)
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()

    # 显式加载运动学参数，确保 rviz 与 move_group 一致
    moveit_config.robot_description_kinematics = load_yaml(
        "openarm_bimanual_moveit_config", "config/kinematics.yaml"
    )
    rviz_ld = generate_moveit_rviz_launch(moveit_config)

    # 补充参数，防止 to_dict 丢失该字段
    extra_params = {
        "robot_description_kinematics": moveit_config.robot_description_kinematics
    }
    for action in rviz_ld.entities:
        if getattr(action, "parameters", None) is not None:
            action.parameters.append(extra_params)

    return rviz_ld
