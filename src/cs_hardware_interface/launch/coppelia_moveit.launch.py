import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# ==========================================
# 辅助函数：安全加载 YAML
# ==========================================
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        print(f"ERROR: Cannot find {absolute_file_path}")
        return None

def generate_launch_description():
    driver_pkg = "cs_hardware_interface"
    moveit_config_pkg = "panda_moveit_config"

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    # ====================================================
    # 2. 机器人描述 (URDF & SRDF) 
    # ====================================================
    # 方案A：从 config 包读取 
    xacro_file = os.path.join(get_package_share_directory(moveit_config_pkg), "config", "Franka.urdf.xacro")
    
    # 如果你的 xacro 需要参数，可以在这里加
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " use_fake_hardware:=false" 
    ])
    robot_description = {"robot_description": robot_description_content}

    # 根据你的截图，文件名是 Franka.srdf
    srdf_file = os.path.join(get_package_share_directory(moveit_config_pkg), "config", "Franka.srdf")
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # ====================================================
    # 3. 加载 MoveIt 核心配置
    # ====================================================
    kinematics_yaml = load_yaml(moveit_config_pkg, "config/kinematics.yaml")
    joint_limits_yaml = load_yaml(moveit_config_pkg, "config/joint_limits.yaml")
    
    # OMPL 规划算法 
    ompl_planning_yaml = load_yaml(moveit_config_pkg, "config/ompl_planning.yaml")
    planning_pipelines_config = {
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml
    }

    # MoveIt 控制器映射
    moveit_controllers_yaml = load_yaml(moveit_config_pkg, "config/moveit_controllers.yaml")
    
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # ====================================================
    # 4. 底层驱动 (ROS2 Control Node)
    # ====================================================
    ros2_controllers = os.path.join(get_package_share_directory(moveit_config_pkg), "config", "ros2_controllers.yaml")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers, {"use_sim_time": use_sim_time}],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ====================================================
    # 5. 加载控制器 (Spawners)
    # ====================================================
    
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "--controller-manager", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
    )

    # ====================================================
    # 6. Move Group 节点 (大脑)
    # ====================================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            planning_pipelines_config,
            trajectory_execution,
            moveit_controllers_yaml,
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
    )

    # ====================================================
    # 7. Rviz 节点
    # ====================================================
    rviz_config_file = os.path.join(get_package_share_directory(moveit_config_pkg), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipelines_config,
            {"use_sim_time": use_sim_time}
        ],
    )

    # 延迟启动控制器，确保 controller_manager 准备好了
    delay_controllers_start = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner, hand_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_controllers_start,
        move_group_node,
        rviz_node
    ])