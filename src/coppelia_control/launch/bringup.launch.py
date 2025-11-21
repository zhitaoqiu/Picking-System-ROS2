import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('coppelia_control')

    # -------------------------------
    # 文件路径
    # -------------------------------
    urdf_path = os.path.join(pkg_path, 'urdf','panda.urdf')
    srdf_path = os.path.join(pkg_path, 'srdf','panda.srdf')
    kinematics_yaml_path = os.path.join(pkg_path, 'config', 'kinematics.yaml')

    # -------------------------------
    # 读取 URDF
    # -------------------------------
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # -------------------------------
    # 读取 SRDF
    # -------------------------------
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    # -------------------------------
    # 读取 kinematics.yaml（必须用 safe_load）
    # -------------------------------
    with open(kinematics_yaml_path, 'r') as f:
        robot_description_kinematics = yaml.safe_load(f)

    # -------------------------------
    # 参数字典
    # -------------------------------
    params = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": robot_description_kinematics
    }

    # -------------------------------
    # 主节点
    # -------------------------------
    main_node = Node(
        package="coppelia_control",
        executable="main_node",
        output="screen",
        parameters=[params]
    )

    return LaunchDescription([
        main_node
    ])
