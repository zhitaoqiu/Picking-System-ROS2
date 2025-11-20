#include <rclcpp/rclcpp.hpp>
#include "coppelia_control/PlanningInterface.hpp"
#include "coppelia_control/ManipulationInterface.hpp"
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("coppelia_moveit_bridge");

    // 创建接口
    auto planner = std::make_shared<coppelia_control::PlanningInterface>(node);
    auto manip   = std::make_shared<coppelia_control::ManipulationInterface>(node);

    // 等一下 joint_states（CoppeliaSim）开始发布
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 目标点（举例一个笛卡尔目标）
    geometry_msgs::msg::Pose target;
    target.position.x = 0.4;
    target.position.y = 0.0;
    target.position.z = 0.4;

    target.orientation.x = 0;
    target.orientation.y = 0;
    target.orientation.z = 0;
    target.orientation.w = 1;

    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if(!planner->planToPose(target, plan))
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Planning succeeded! Sending trajectory to CoppeliaSim...");

    // 发送轨迹到 /joint_commands
    manip->executeTrajectory(plan.trajectory_);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

