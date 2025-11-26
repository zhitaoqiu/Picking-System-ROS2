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

void ManipulationInterface::setMoveItToCoppeliaJointMap(const std::map<std::string,std::string> & joint_map)
{
    moveit_to_coppelia_joint_map_ = joint_map;
}

sensor_msgs::msg::JointState
ManipulationInterface::translateJointNames(
    const sensor_msgs::msg::JointState& moveit_joint_state)
{
    sensor_msgs::msg::JointState result;
    result.header = moveit_joint_state.header;

    for (size_t i = 0; i < moveit_joint_state.name.size(); ++i)
    {
        const std::string& moveit_name = moveit_joint_state.name[i];

        if (moveit_to_coppelia_joint_map_.count(moveit_name) == 0)
        {
            RCLCPP_WARN(node_->get_logger(),
                "MoveIt joint [%s] not found in mapping table, skipping...",
                moveit_name.c_str());
            continue;
        }

        // 1. 名字替换
        result.name.push_back(moveit_to_coppelia_joint_map_.at(moveit_name));

        // 2. 对应位置复制
        if (i < moveit_joint_state.position.size())
            result.position.push_back(moveit_joint_state.position[i]);
    }

    return result;
}

bool ManipulationInterface::executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
    // 添加轨迹有效性检查
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory is empty!");
        return false;
    }
    
    if (!joint_cmd_publisher_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint command publisher not initialized!");
        return false;
    }
    
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
    const auto& jt = trajectory.joint_trajectory;
    
    sensor_msgs::msg::JointState moveit_final;
    moveit_final.header.stamp = node_->now();
    moveit_final.name = jt.joint_names;
    moveit_final.position = jt.points.back().positions;

    sensor_msgs::msg::JointState cmd = translateJointNames(moveit_final);
    
    // 检查是否有有效的关节命令
    if (cmd.name.empty() || cmd.position.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Translated joint command is empty!");
        return false;
    }

    joint_cmd_publisher_->publish(cmd);

    double total_duration_sec =
        jt.points.back().time_from_start.sec +
        jt.points.back().time_from_start.nanosec * 1e-9;

    // 确保至少等待 0.5 秒
    std::this_thread::sleep_for(
        std::chrono::duration<double>(std::max(total_duration_sec, 0.5)));

    RCLCPP_INFO(node_->get_logger(), "Execution finished in SMOOTH mode.");
    return true;
}

bool ManipulationInterface::execute_point_to_point_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
    const auto& jt = trajectory.joint_trajectory;
    double last_time_sec = 0.0;
    
    for (const auto& point : jt.points)
    {
        sensor_msgs::msg::JointState moveit_state;
        moveit_state.header.stamp = node_->now();
        moveit_state.name = jt.joint_names;
        moveit_state.position = point.positions;

        sensor_msgs::msg::JointState cmd = translateJointNames(moveit_state);
        
        // 检查是否有有效的关节命令
        if (cmd.name.empty() || cmd.position.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Skipping empty translated joint command");
            continue;
        }

        joint_cmd_publisher_->publish(cmd);

        // 计算当前点的时间戳
        double point_duration_sec = static_cast<double>(point.time_from_start.sec) + 
                                    static_cast<double>(point.time_from_start.nanosec) * 1e-9;
        
        // 计算实际需要等待的时间（当前点时间 - 上一个点时间）
        double wait_time_sec = point_duration_sec - last_time_sec;
        
        // 确保最小等待时间
        if (wait_time_sec < 0.001)
        {
            wait_time_sec = 0.001;
        }
        
        // 修正：使用 wait_time_sec 而不是 point_duration_sec
        std::this_thread::sleep_for(std::chrono::duration<double>(wait_time_sec)); 
        
        // 更新上一个点的时间
        last_time_sec = point_duration_sec;
    }

    RCLCPP_INFO(node_->get_logger(), "Execution finished in POINT-TO-POINT mode.");
    return true;
}

bool ManipulationInterface::openGripper()
{
    if (!gripper_cmd_publisher_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Gripper command publisher not initialized!");
        return false;
    }
    
    if (gripper_joint_names_.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Gripper joint names not set!");
        return false;
    }
    
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = gripper_joint_names_;
    cmd.position.resize(gripper_joint_names_.size(), 0.04);

    gripper_cmd_publisher_->publish(cmd);
    
    RCLCPP_INFO(node_->get_logger(), "Gripper opened");
    return true;
}

bool ManipulationInterface::closeGripper()
{
    if (!gripper_cmd_publisher_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Gripper command publisher not initialized!");
        return false;
    }
    
    if (gripper_joint_names_.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Gripper joint names not set!");
        return false;
    }
    
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = gripper_joint_names_;
    cmd.position.resize(gripper_joint_names_.size(), 0.00);

    gripper_cmd_publisher_->publish(cmd);
    
    RCLCPP_INFO(node_->get_logger(), "Gripper closed");
    return true;
}

} // namespace coppelia_control