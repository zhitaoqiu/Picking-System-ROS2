#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>
#include <memory>
//planning 的主要功能是生成轨迹
namespace coppelia_control
{

class PlanningInterface
{
public:
    /**
     * @brief 构造函数
     * @param node ROS 2 节点指针
     * @param planning_group 初始规划组名称 (例如 "panda_arm" 或 "franka_arm")
     */
    PlanningInterface(const rclcpp::Node::SharedPtr& node, const std::string& planning_group);

    /**
     * @brief 初始化 MoveGroupInterface
     * @return true 如果初始化成功
     */
    bool initialize();

    /**
     * @brief [新功能] 动态更改规划组
     * @param new_planning_group 新的规划组名称
     * @return true 如果切换成功
     */
    bool changePlanningGroup(const std::string& new_planning_group);

    /**
     * @brief [新功能] 配置规划器参数
     * @param planner_id 规划算法ID (例如 "RRTConnectkConfigDefault", "RRTstar", "Pilz" 等)
     * @param planning_time 最大规划时间 (秒)
     * @param attempts 最大尝试次数
     */
    void setupPlanner(const std::string& planner_id, double planning_time, int attempts);

    /**
     * @brief [新功能] 设置速度和加速度缩放因子
     * @param max_vel_factor 最大速度比例 (0.0 - 1.0)
     * @param max_acc_factor 最大加速度比例 (0.0 - 1.0)
     */
    void setScalingFactors(double max_vel_factor, double max_acc_factor);

    // --- 核心规划功能 ---

    bool planToPose(const geometry_msgs::msg::Pose& target_pose, moveit::planning_interface::MoveGroupInterface::Plan& plan);

    bool planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, moveit_msgs::msg::RobotTrajectory& trajectory);

    bool planToNamedTarget(const std::string& target_name, moveit::planning_interface::MoveGroupInterface::Plan& plan);

    bool planToJointState(const std::vector<double>& joint_positions, moveit::planning_interface::MoveGroupInterface::Plan& plan);

    // 轨迹平滑处理
    bool smoothTrajectory(moveit_msgs::msg::RobotTrajectory& trajectory);

    // 获取当前信息
    geometry_msgs::msg::Pose getCurrentPose();
    std::string getCurrentPlanningGroup() const { return planning_group_name_; }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    
    // 配置参数成员变量
    std::string planning_group_name_;
    std::string current_planner_id_;
    double planning_time_;
    int planning_attempts_;
    double max_vel_scaling_;
    double max_acc_scaling_;
};

} // namespace coppelia_control