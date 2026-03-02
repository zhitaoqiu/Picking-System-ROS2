# 基于 ROS 2 与 CoppeliaSim 的抓取系统 (Picking-System-ROS2)

## 1. 项目背景
本项目为个人研一阶段研究课题，旨在非结构化环境下实现机械臂对农产品的自主识别与抓取。目前已在 CoppeliaSim 仿真环境下搭建起全链路控制系统。记录第一次

## 2. 系统架构
系统基于 **ROS 2 Humble** 框架开发，主要分为三个核心层级：
* **感知层**：暂时利用 YOLOv8 进行目标检测，结合深度图进行 ROI 采样，通过反投影算法解算 3D 空间坐标。
* **规划层**：集成 MoveIt 2 进行避障路径规划与运动学求解。
* **执行层**：自主开发 C++ 硬件接口插件 `cs_hardware_interface`，通过 **ZeroMQ (ZMQ)** 与 CoppeliaSim 仿真器进行双向实时通信（1000Hz）日后有待更新。

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
