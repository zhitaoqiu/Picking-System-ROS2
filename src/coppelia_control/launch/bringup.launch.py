from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('coppelia_control')

    urdf_path = os.path.join(pkg_path, 'src', 'urdf', 'panda.urdf')
    srdf_path = os.path.join(pkg_path, 'src', 'srdf', 'panda.srdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
    }

    main_node = Node(
        package='coppelia_control',
        executable='main_node',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        main_node
    ])
