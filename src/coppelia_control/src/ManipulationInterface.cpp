#include "coppelia_control/ManipulationInterface.hpp"
#include <chrono>
#include <thread>
#include <algorithm>

namespace coppelia_control
{

ManipulationInterface::ManipulationInterface(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      joint_cmd_topic_("/coppelia/joint_cmd"),
      gripper_cmd_topic_("/coppelia/gripper_cmd"),
      use_smooth_trajectory_(true)
{
    // 构造函数不做 Publisher 初始化，留给 initialize 或 setTopic
}

bool ManipulationInterface::initialize()
{
    if (!node_) return false;
    
    // 创建发布者
    joint_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_cmd_topic_, 10);
    gripper_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(gripper_cmd_topic_, 10);
    
    RCLCPP_INFO(node_->get_logger(), "操作接口初始化完成 | Cmd Topic: %s", joint_cmd_topic_.c_str());
    return true;
}

void ManipulationInterface::setJointNameRemap(const std::map<std::string, std::string>& remap_rules)
{
    joint_name_map_ = remap_rules;
    RCLCPP_INFO(node_->get_logger(), "已设置 %zu 条关节名称映射规则", remap_rules.size());
}

std::vector<std::string> ManipulationInterface::remapNames(const std::vector<std::string>& original_names)
{
    // 如果没有映射表，直接返回原名
    if (joint_name_map_.empty()) {
        return original_names;
    }

    std::vector<std::string> new_names;
    new_names.reserve(original_names.size());

    for (const auto& name : original_names) {
        auto it = joint_name_map_.find(name);
        if (it != joint_name_map_.end()) {
            new_names.push_back(it->second); // 使用映射后的名字
        } else {
            new_names.push_back(name); // 没找到映射就用原名
        }
    }
    return new_names;
}

bool ManipulationInterface::executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
    if (trajectory.joint_trajectory.points.empty()) {
        RCLCPP_WARN(node_->get_logger(), "收到空轨迹，忽略执行");
        return false;
    }
    
    if (!joint_cmd_publisher_) {
        // 尝试懒加载初始化
        initialize(); 
        if (!joint_cmd_publisher_) {
            RCLCPP_ERROR(node_->get_logger(), "Publisher 未初始化!");
            return false;
        }
    }
    
    if (use_smooth_trajectory_) {
        return execute_smooth_trajectory(trajectory);
    } else {
        return execute_point_to_point_trajectory(trajectory);
    }
}

void ManipulationInterface::setGripperJointNames(const std::vector<std::string>& gripper_joint_names)
{
    gripper_joint_names_ = gripper_joint_names;
}

void ManipulationInterface::setJointCmdTopic(const std::string& topic)
{
    joint_cmd_topic_ = topic;
    // 重新创建 publisher
    if (node_) {
        joint_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_cmd_topic_, 10);
    }
}

void ManipulationInterface::setGripperCmdTopic(const std::string& topic)
{
    gripper_cmd_topic_ = topic;
    if (node_) {
        gripper_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(gripper_cmd_topic_, 10);
    }
}

void ManipulationInterface::useSmoothTrajectoryExecution(bool use_smooth)
{
    use_smooth_trajectory_ = use_smooth;
}

bool ManipulationInterface::execute_smooth_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
    const auto& jt = trajectory.joint_trajectory;
    
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    
    // 【关键】进行名称重映射
    cmd.name = remapNames(jt.joint_names);
    
    // 平滑模式通常假设仿真器有插值能力，或者我们只发送终点
    // 这里我们只发送轨迹的最后一个点作为目标
    cmd.position = jt.points.back().positions;
    
    if (cmd.name.empty() || cmd.position.empty()) return false;

    joint_cmd_publisher_->publish(cmd);

    // 计算轨迹总耗时
    double total_duration_sec = 0.5; // 默认最小时间
    if (!jt.points.empty()) {
        double traj_time = rclcpp::Duration(jt.points.back().time_from_start).seconds();
        total_duration_sec = std::max(traj_time, 0.5);
    }

    // 等待执行完成
    std::this_thread::sleep_for(std::chrono::duration<double>(total_duration_sec));

    return true;
}

bool ManipulationInterface::execute_point_to_point_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
    const auto& jt = trajectory.joint_trajectory;
    double last_time_sec = 0.0;
    
    // 缓存重映射后的名字，避免循环里重复查找
    std::vector<std::string> target_names = remapNames(jt.joint_names);
    
    for (const auto& point : jt.points)
    {
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = node_->now();
        cmd.name = target_names;
        cmd.position = point.positions;
        
        joint_cmd_publisher_->publish(cmd);

        double point_duration_sec = rclcpp::Duration(point.time_from_start).seconds();
        double wait_time_sec = point_duration_sec - last_time_sec;
        
        if (wait_time_sec > 0.001) {
            std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_sec)); 
        }
        
        last_time_sec = point_duration_sec;
    }
    return true;
}

bool ManipulationInterface::openGripper()
{
    if (!gripper_cmd_publisher_) return false;
    if (gripper_joint_names_.empty()) return false;
    
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    // 夹爪名字也支持重映射
    cmd.name = remapNames(gripper_joint_names_);
    cmd.position.resize(cmd.name.size(), 0.04); // 0.04m 打开

    gripper_cmd_publisher_->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "Gripper Open 指令已发送");
    return true;
}

bool ManipulationInterface::closeGripper()
{
    if (!gripper_cmd_publisher_) return false;
    if (gripper_joint_names_.empty()) return false;
    
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = remapNames(gripper_joint_names_);
    cmd.position.resize(cmd.name.size(), 0.00); // 0.00m 关闭

    gripper_cmd_publisher_->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "Gripper Close 指令已发送");
    return true;
}

} // namespace coppelia_control