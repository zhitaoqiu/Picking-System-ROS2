# 基于 ROS 2 与 CoppeliaSim 的抓取系统 (Picking-System-ROS2)

## 1. 项目背景
本项目实现一个基于 ROS2 与 CoppeliaSim 的机械臂视觉抓取系统。
系统包含视觉识别、三维坐标计算、运动规划以及仿真执行模块，
用于探索机器人视觉引导抓取（Vision-guided Picking）的基本流程。

## 2. 系统架构
系统基于 **ROS 2 Humble** 框架开发，主要分为三个核心层级：
* **感知层**：使用 OpenCV 进行目标颜色识别，并结合深度图进行 ROI 采样，通过相机内参反投影计算目标三维坐标。
* **规划层**：集成 MoveIt 2 进行避障路径规划与运动学求解。
* **执行层**：利用 C++ 硬件接口插件 `cs_hardware_interface`，通过 **ZeroMQ (ZMQ)** 与 CoppeliaSim 仿真器进行双向实时通信（1000Hz）日后有待更新。

## 3. 技术栈
* **语言**: C++17 (核心接口与控制), Python 3.10 (视觉算法)
* **框架**: ROS 2 Humble (rclcpp, rclpy)
* **仿真**: CoppeliaSim (V-REP)
* **算法**: 手眼变换 (Hand-Eye Calibration), 图像反投影, MoveIt 2 运动规划

## 4. 核心功能与进度
- [x] **仿真环境搭建**: 完成 Franka Panda 机械臂与传感器配置。
- [x] **ROS 2 硬件接口**: 实现基于 ZMQ 的 `SystemInterface` 插件。
- [x] **视觉解算**: 实现 2D 像素坐标到 3D 基座坐标系的转换。
- [ ] **抓取策略优化**: 正在调试 MoveIt 2 的预抓取位姿规划 (Working on it...)

## 5. 快速开始
```bash
# 进入工作空间
cd ~/coppelia_control
# 编译项目
colcon build --symlink-install
# 运行硬件接口 (示例)
ros2 launch your_package_name demo.launch.py
