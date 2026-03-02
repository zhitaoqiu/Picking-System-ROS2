import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径 (如果有 config 文件可以这里加载，现在直接写参数里也行)
    # config = os.path.join(...)

    return LaunchDescription([
        # ============================================================
        # 1. 静态 TF 发布 (关键！)
        # ============================================================
        # 作用：告诉 ROS，"HandCamera" 这个坐标系是粘在机器人的 "panda_hand" 上的
        # 参数：x y z yaw pitch roll parent child
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=[
                '0', '0', '0',   # XYZ: 假设相机在手掌心前方上方 5cm
                '0', '0', '0',         # RPY: 假设相机方向和手掌一致 (需根据实际调整)
                'FrankaGripper',          # 父: MoveIt 标准手掌 Link 名
                'HandCamera'           # 子: Lua 脚本里发的 frame_id
            ]
        ),

        # ============================================================
        # 2. Vision Node - 视觉核心
        # ============================================================
        Node(
            package='coppelia_vision',
            executable='vision_node', # 确保 setup.py 里 entry_point 也是这个名字
            name='vision_node',
            output='screen',
            parameters=[{
                # === 话题匹配 Lua ===
                'sub_image_topic': '/camera/image_raw',
                'pub_result_topic': '/vision/object_pose',
                
                # === 坐标系匹配 ===
                'base_frame': 'robot_base', # Panda 机器人的基座
                'camera_frame': 'HandCamera',
                
                # === 相机参数 (根据 CoppeliaSim 设置) ===
                'fov_degrees': 60.0,    # 默认通常是 60 或 90
                'target_z_height': 0.02, # 方块高度/2 (中心点高度)
                
                # === 调试 ===
                'enable_visualization': True,
                'model_type': 'color'
            }]
        )
    ])