import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='global_camera_tf_publisher',
        arguments=[
            '--x', '-1.0', 
            '--y', '0.0', 
            '--z', '0.93',         # 高度 1.0米
            '--yaw', '0.0', 
            '--pitch', '0', 
            '--roll', '-3.14159',
            '--frame-id', 'robot_base',           # 父坐标系
            '--child-frame-id', 'global_camera_link' # 子坐标系 (和 Python 参数对应)
        ]
    )


    vision_node = Node(
        package='coppelia_vision',
        executable='vision_node', # 请确认 setup.py 里 entry_points 是这个名字
        name='global_vision_node',
        output='screen',
        
        # 【重点修改】使用 parameters 直接覆盖 Python 里的默认值
        parameters=[{
            # (1) 订阅话题：必须和 Lua 脚本发出的名字一致
            'sub_image_topic': '/global_camera/image',
            
            # (2) 相机坐标系：必须和 TF 发布器里的 child-frame-id 一致
            # 如果这里不改，默认是 HandCamera，算出来的坐标全是错的
            'camera_frame': 'global_camera_link',
            
            # (3) 基座坐标系：告诉节点相对于谁计算坐标
            'base_frame': 'robot_base',
            
            # (4) 结果发布话题 (可选，保持默认即可)
            'pub_result_topic': '/vision/object_pose',
            
            # (5) 相机视野 (和你仿真器设置保持一致，截图里是60度)
            'fov_degrees': 60.0,
            
            # (6) 物体高度 (红方块中心高度，大约2cm)
            'target_z_height': -0.050
        }]
    )

    return LaunchDescription([
        camera_tf_node,
        vision_node
    ])