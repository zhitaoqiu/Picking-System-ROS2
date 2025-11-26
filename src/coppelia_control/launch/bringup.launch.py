from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_path(package_name)
    yaml_file = os.path.join(package_path, file_path)

    with open(yaml_file, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():

    # -------------------------
    # 路径
    # -------------------------
    panda_description = get_package_share_path("moveit_resources_panda_description")
    panda_moveit_config = get_package_share_path("moveit_resources_panda_moveit_config")

    urdf_path = os.path.join(panda_description, "urdf", "panda.urdf")
    srdf_path = os.path.join(panda_moveit_config, "config", "panda.srdf")

    # Load yaml configs
    kinematics_yaml = load_yaml("moveit_resources_panda_moveit_config", "config/kinematics.yaml")
    ompl_yaml = load_yaml("moveit_resources_panda_moveit_config", "config/ompl_planning.yaml")

    # Parameters
    robot_description = ParameterValue(open(urdf_path).read(), value_type=str)
    robot_description_semantic = ParameterValue(open(srdf_path).read(), value_type=str)

    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"robot_description_semantic": robot_description_semantic},
                {"kinematics": kinematics_yaml},
                {"ompl": ompl_yaml},

                # ---- 最关键修复项 ----
                {"planning_pipelines": ["ompl"]},
                # ----------------------

                {"default_planning_pipeline": "ompl"},
            ]
        )
    ])
