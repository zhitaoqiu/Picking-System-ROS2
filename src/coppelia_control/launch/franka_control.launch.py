import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

# --- 辅助函数：加载 YAML 文件 ---
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        raise FileNotFoundError(f"无法找到配置文件: {absolute_file_path}")

def generate_launch_description():
    # 1. 声明参数
    # ---------------------------------------------------------
    use_smooth_arg = DeclareLaunchArgument(
        'use_smooth_trajectory', default_value='true'
    )
    
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='arm', 
        description='MoveIt Planning Group Name'
    )

    # 2. 准备 MoveIt 配置
    # ---------------------------------------------------------
    moveit_config_pkg = 'franka_moveit_config'
    
    # A. Robot Description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([get_package_share_directory(moveit_config_pkg), "config", "Franka.urdf.xacro"])
        ],
        on_stderr='ignore'
    )
    robot_description = {"robot_description": robot_description_content}

    # B. SRDF
    srdf_path = os.path.join(get_package_share_directory(moveit_config_pkg), "config", "Franka.srdf")
    try:
        with open(srdf_path, 'r') as f:
            robot_description_semantic = {"robot_description_semantic": f.read()}
    except FileNotFoundError:
        raise FileNotFoundError(f"找不到 SRDF 文件: {srdf_path}")

    # C. Kinematics & Limits
    kinematics_yaml = load_yaml(moveit_config_pkg, "config/kinematics.yaml")
    joint_limits_yaml = load_yaml(moveit_config_pkg, "config/joint_limits.yaml")
    
    # D. Planning Pipeline (OMPL)
    ompl_planning_yaml = load_yaml(moveit_config_pkg, "config/ompl_planning.yaml")
    
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # E. Trajectory Execution
    # 加载控制器配置 (即使是 Topic 驱动，加载这个可以防止一些伪控制器报错)
    try:
        moveit_controllers = {
            "moveit_simple_controller_manager": load_yaml(moveit_config_pkg, "config/moveit_controllers.yaml"),
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }
    except:
        moveit_controllers = {}

    # 3. 定义节点 (核心修改：全部加入 use_sim_time: True)
    # ---------------------------------------------------------
    
    # [Node 1] Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {'use_sim_time': True} # <--- 关键修改
        ],
    )

    # [Node 2] Move Group (服务端)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers,
            {'use_sim_time': True} # <--- 关键修改
        ],
    )

    # [Node 3] RViz
    rviz_config_file = os.path.join(
        get_package_share_directory(moveit_config_pkg), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': True} # <--- 关键修改
        ],
    )

    # [Node 4] 你的控制节点 (客户端)
    franka_control_node = Node(
        package='coppelia_control',
        executable='franka_control_node',
        name='franka_control_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            {
                'use_smooth_trajectory': LaunchConfiguration('use_smooth_trajectory'),
                'planning_group': LaunchConfiguration('planning_group'),
                'joint_cmd_topic': '/coppelia/joint_cmd',
                'gripper_cmd_topic': '/coppelia/gripper_cmd',
                'use_sim_time': True # <--- 关键修改
            }
        ],
    )

    return LaunchDescription([
        use_smooth_arg,
        planning_group_arg,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        franka_control_node,
    ])