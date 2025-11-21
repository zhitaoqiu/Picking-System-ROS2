#include "coppelia_control/ManipulationInterface.hpp"
#include <chrono>
#include <thread>
#include <algorithm>

namespace coppelia_control
{
ManipulationInterface::ManipulationInterface(const rclcpp::Node::SharedPtr& node):node_(node)
{
    node_->declare_parameter<std::string>("joint_cmd_topic", "/coppelia/joint_cmd");
    node_->declare_parameter<std::string>("gripper_cmd_topic", "/coppelia/gripper_cmd");
    node_->declare_parameter<bool>("use_smooth_trajectory", true);

    node_->get_parameter("joint_cmd_topic", joint_cmd_topic_);
    node_->get_parameter("gripper_cmd_topic", gripper_cmd_topic_);
    node_->get_parameter("use_smooth_trajectory", use_smooth_trajectory_);
    joint_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_cmd_topic_, 10);
    gripper_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(gripper_cmd_topic_, 10);

}
bool ManipulationInterface::executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
    if (use_smooth_trajectory_)
    {
        return execute_smooth_trajectory(trajectory);
    }
    else
    {
        return execute_point_to_point_trajectory(trajectory);
    }
}
void ManipulationInterface::setJointNames(const std::vector<std::string>& joint_names)
{
    joint_names_ = joint_names;
}
void ManipulationInterface::setGripperJointNames(const std::vector<std::string>& gripper_joint_names)
{
    gripper_joint_names_ = gripper_joint_names;
}
void ManipulationInterface::setJointCmdTopic(const std::string& topic)
{
    joint_cmd_topic_ = topic;
    joint_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_cmd_topic_, 10);
}

void ManipulationInterface::setGripperCmdTopic(const std::string& topic)
{
    gripper_cmd_topic_ = topic;
    gripper_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(gripper_cmd_topic_, 10);
}
void ManipulationInterface::useSmoothTrajectoryExecution(bool use_smooth)
{
    use_smooth_trajectory_ = use_smooth;
}
bool ManipulationInterface::execute_smooth_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    const auto& points = trajectory.joint_trajectory.points;
    const auto& last_point = points.back();

    RCLCPP_INFO(node_->get_logger(), "Executing in SMOOTH mode: Sending final goal position only.");

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = joint_names;
    cmd.position = last_point.positions;

    joint_cmd_publisher_->publish(cmd);
    
    double total_duration_sec = static_cast<double>(last_point.time_from_start.sec) + 
                                static_cast<double>(last_point.time_from_start.nanosec) * 1e-9;
    
    double min_wait_sec = 0.5;
    double effective_wait_sec = std::max(total_duration_sec, min_wait_sec);
    
    RCLCPP_INFO(node_->get_logger(), 
                "Total trajectory time: %.2f sec. Waiting for %.2f sec...", 
                total_duration_sec, effective_wait_sec);

    std::this_thread::sleep_for(std::chrono::duration<double>(effective_wait_sec)); 
    
    RCLCPP_INFO(node_->get_logger(), "Execution finished in SMOOTH mode.");
    return true;
}
bool ManipulationInterface::execute_point_to_point_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    const auto& points = trajectory.joint_trajectory.points;

    RCLCPP_INFO(node_->get_logger(), "Executing in POINT-TO-POINT mode: Sending each trajectory point sequentially.");

    for (const auto& point : points)
    {
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = node_->now();
        cmd.name = joint_names;
        cmd.position = point.positions;

        joint_cmd_publisher_->publish(cmd);

        double point_duration_sec = static_cast<double>(point.time_from_start.sec) + 
                                    static_cast<double>(point.time_from_start.nanosec) * 1e-9;

        RCLCPP_INFO(node_->get_logger(), 
                    "Sent point with duration: %.2f sec. Waiting...", 
                    point_duration_sec);

        std::this_thread::sleep_for(std::chrono::duration<double>(point_duration_sec)); 
    }

    RCLCPP_INFO(node_->get_logger(), "Execution finished in POINT-TO-POINT mode.");
    return true;
}
bool ManipulationInterface::openGripper()
{
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = gripper_joint_names_;
    cmd.position.resize(gripper_joint_names_.size(), 0.04);

    gripper_cmd_publisher_->publish(cmd);
    return true;
}

bool ManipulationInterface::closeGripper()
{
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = gripper_joint_names_;
    cmd.position.resize(gripper_joint_names_.size(), 0.00);

    gripper_cmd_publisher_->publish(cmd);
    return true;
}
} 