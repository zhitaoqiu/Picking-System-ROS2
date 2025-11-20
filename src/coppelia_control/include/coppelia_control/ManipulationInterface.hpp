#pragma once
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp> // Used to send commands to CoppeliaSim
#include <string>
#include <memory>
#include <vector> 

namespace coppelia_control
{
class ManipulationInterface
{
public:
    explicit ManipulationInterface(const rclcpp::Node::SharedPtr& node);
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    bool openGripper();
    bool closeGripper();
    void setJointNames(const std::vector<std::string>& joint_names);
private:
    bool execute_smooth_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    bool execute_point_to_point_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_cmd_publisher_;
    std::string joint_cmd_topic_;
    std::string gripper_cmd_topic_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> gripper_joint_names_;
    bool use_smooth_trajectory_;
};
}