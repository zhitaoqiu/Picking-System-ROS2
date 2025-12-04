#include "coppelia_control/PlanningInterface.hpp"
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace coppelia_control
{

PlanningInterface::PlanningInterface(const rclcpp::Node::SharedPtr& node, const std::string& planning_group)
    : node_(node), 
      planning_group_name_(planning_group),
      // 设置默认参数
      current_planner_id_("RRTConnectkConfigDefault"),
      planning_time_(5.0),
      planning_attempts_(5),
      max_vel_scaling_(0.5),
      max_acc_scaling_(0.5)
{
}

bool PlanningInterface::initialize()
{
    if (!node_) return false;

    try
    {
        // 重新创建 MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_name_);
        
        // 应用当前的配置参数
        move_group_->setPlannerId(current_planner_id_);
        move_group_->setPlanningTime(planning_time_);
        move_group_->setNumPlanningAttempts(planning_attempts_);
        move_group_->setMaxVelocityScalingFactor(max_vel_scaling_);
        move_group_->setMaxAccelerationScalingFactor(max_acc_scaling_);

        // 默认不启用 start_state_monitor 的等待，防止仿真卡顿
        // move_group_->setStartStateToCurrentState(); 

        RCLCPP_INFO(node_->get_logger(), "规划接口初始化完成 | Group: %s | Planner: %s", 
                    planning_group_name_.c_str(), current_planner_id_.c_str());
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "MoveGroup 初始化失败: %s", e.what());
        return false;
    }
}

bool PlanningInterface::changePlanningGroup(const std::string& new_planning_group)
{
    if (new_planning_group == planning_group_name_ && move_group_) {
        RCLCPP_INFO(node_->get_logger(), "规划组已经是 %s，无需切换", new_planning_group.c_str());
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "正在切换规划组: %s -> %s ...", 
                planning_group_name_.c_str(), new_planning_group.c_str());
    
    planning_group_name_ = new_planning_group;
    
    // 重新初始化 move_group_ 指针
    return initialize();
}

void PlanningInterface::setupPlanner(const std::string& planner_id, double planning_time, int attempts)
{
    current_planner_id_ = planner_id;
    planning_time_ = planning_time;
    planning_attempts_ = attempts;

    if (move_group_) {
        move_group_->setPlannerId(planner_id);
        move_group_->setPlanningTime(planning_time);
        move_group_->setNumPlanningAttempts(attempts);
        RCLCPP_INFO(node_->get_logger(), "规划器参数已更新: ID=%s, Time=%.1f, Attempts=%d", 
                    planner_id.c_str(), planning_time, attempts);
    }
}

void PlanningInterface::setScalingFactors(double max_vel_factor, double max_acc_factor)
{
    max_vel_scaling_ = max_vel_factor;
    max_acc_scaling_ = max_acc_factor;

    if (move_group_) {
        move_group_->setMaxVelocityScalingFactor(max_vel_factor);
        move_group_->setMaxAccelerationScalingFactor(max_acc_factor);
        RCLCPP_INFO(node_->get_logger(), "速度/加速度缩放已更新: V=%.2f, A=%.2f", 
                    max_vel_factor, max_acc_factor);
    }
}

bool PlanningInterface::planToPose(const geometry_msgs::msg::Pose& target_pose, 
                                 moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    if (!move_group_) return false;
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target_pose);
    return static_cast<bool>(move_group_->plan(plan));
}

bool PlanningInterface::planToNamedTarget(const std::string& target_name, 
                                        moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    if (!move_group_) return false;
    move_group_->clearPoseTargets();
    
    if (!move_group_->setNamedTarget(target_name)) {
        RCLCPP_ERROR(node_->get_logger(), "目标 '%s' 在 SRDF 中未定义", target_name.c_str());
        return false;
    }
    return static_cast<bool>(move_group_->plan(plan));
}

bool PlanningInterface::planToJointState(const std::vector<double>& joint_positions, 
                                       moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    if (!move_group_) return false;
    move_group_->clearPoseTargets();
    
    move_group_->setJointValueTarget(joint_positions);
    return static_cast<bool>(move_group_->plan(plan));
}

bool PlanningInterface::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, 
                                        moveit_msgs::msg::RobotTrajectory& trajectory)
{
    if (!move_group_ || waypoints.empty()) return false;
    move_group_->clearPoseTargets();

    const double jump_threshold = 0.0; 
    const double eef_step = 0.01;      
    
    moveit_msgs::msg::RobotTrajectory temp_trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, temp_trajectory);

    if (fraction >= 0.9) {
        trajectory = temp_trajectory;
        return true;
    }
    RCLCPP_WARN(node_->get_logger(), "笛卡尔路径覆盖率低: %.2f%%", fraction * 100.0);
    return false;
}

bool PlanningInterface::smoothTrajectory(moveit_msgs::msg::RobotTrajectory& trajectory)
{
    if (!move_group_) return false;

    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), planning_group_name_);
    
    // 获取当前状态作为起点
    auto current_state = move_group_->getCurrentState();
    if (!current_state) return false;
    
    rt.setRobotTrajectoryMsg(*current_state, trajectory);

    // 使用配置好的成员变量进行平滑
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt, max_vel_scaling_, max_acc_scaling_);
    
    if (success) {
        rt.getRobotTrajectoryMsg(trajectory);
        return true;
    }
    return false;
}

geometry_msgs::msg::Pose PlanningInterface::getCurrentPose()
{
    if (!move_group_) return geometry_msgs::msg::Pose();
    return move_group_->getCurrentPose().pose;
}

} // namespace coppelia_control