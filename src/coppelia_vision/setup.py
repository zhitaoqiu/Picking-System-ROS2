from setuptools import setup
import os
from glob import glob

package_name = 'coppelia_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.detectors'], # 注意加上子包
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 安装 config 文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosqiu',
    maintainer_email='rosqiu@todo.todo',
    description='Modular Vision Package for CoppeliaSim',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 名字 = 包名.文件名:主函数
            'vision_node = coppelia_vision.vision_node:main',
        ],
    },
)