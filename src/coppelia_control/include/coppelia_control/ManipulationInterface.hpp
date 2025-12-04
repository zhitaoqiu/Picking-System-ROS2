#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <memory>
#include <vector> 
#include <map>
// manipulation 的主要功能是规划完毕发送关节命令到话题
namespace coppelia_control
{

class ManipulationInterface
{
public:
    explicit ManipulationInterface(const rclcpp::Node::SharedPtr& node);

    // 初始化 Publisher
    bool initialize();

    // 执行 MoveIt 规划出的轨迹
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);

    // 夹爪控制
    bool openGripper();
    bool closeGripper();

    // --- 配置接口 ---

    // [新功能] 设置关节名称映射表 (URDF Name -> CoppeliaSim Name)
    // 如果设置了映射，发送指令时会自动替换名字
    void setJointNameRemap(const std::map<std::string, std::string>& remap_rules);
    void setGripperJointNames(const std::vector<std::string>& gripper_joint_names);
    void setJointCmdTopic(const std::string& topic);
    void setGripperCmdTopic(const std::string& topic);
    void useSmoothTrajectoryExecution(bool use_smooth);
    
private:
    // 内部使用的名称转换函数
    std::vector<std::string> remapNames(const std::vector<std::string>& original_names);

    bool execute_smooth_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    bool execute_point_to_point_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_cmd_publisher_;
    
    std::string joint_cmd_topic_;
    std::string gripper_cmd_topic_;
    std::vector<std::string> gripper_joint_names_;
    
    // 映射表: Key=MoveItName, Value=SimName
    std::map<std::string, std::string> joint_name_map_; 
    
    bool use_smooth_trajectory_;
};

} // namespace coppelia_control