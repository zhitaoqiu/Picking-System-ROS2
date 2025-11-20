#include "coppelia_control/PlanningInterface.hpp"
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace coppelia_control
{
PlanningInterface::PlanningInterface(const rclcpp::Node::SharedPtr& node) :node_(node)
{
    if(!initialize())
    {
        RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface初始化失败。请检查ROS 2环境和MoveIt配置。");
    }
}

bool PlanningInterface::initialize()
{
    try
    {
        move_group_=std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,PLANNING_GROUP);
        move_group_->setPlanningTime(10.0);
        move_group_->setEndEffectorLink("参数待定");
        RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface已成功初始化规划组 %s。", PLANNING_GROUP.c_str());
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "初始化MoveGroupInterface时出错 %s", e.what());
        return false;
    }
}
bool PlanningInterface::planTopose(const geometry_msgs::msg::Pose& target_pose,moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    move_group_->setPoseTarget(target_pose);
    return (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
bool PlanningInterface::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,moveit_msgs::msg::RobotTrajectory& trajectory)
{
    if (!move_group_ || waypoints.size() < 2) return false;
    const double jump_threshold= 0.0;//禁用跳跃检测
    const double eef_step=0.01; //终端步长
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    return fraction > 0.9;
}
bool PlanningInterface::planToNamedTarget(const std::string& target_name,moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    move_group_->setNamedTarget(target_name);
    return (move_group_->plan(plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
void PlanningInterface::smoothTrajectory(moveit_msgs::msg::RobotTrajectory& trajectory)
{
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), PLANNING_GROUP);
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt, 0.5, 0.5); // 设置最大速度和加速度
    
    if (success) {
        rt.getRobotTrajectoryMsg(trajectory);
        RCLCPP_INFO(node_->get_logger(), "时间参数化成功。");
    } else {
        RCLCPP_WARN(node_->get_logger(), "时间参数化失败。轨迹可能没有时间戳。");
    }
}
geometry_msgs::msg::Pose PlanningInterface::getCurrentPose()
{
    return move_group_->getCurrentPose().pose;
}
}