#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "coppelia_control/ManipulationInterface.hpp"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("coppelia_control_main");

    // （1）MoveIt2 MoveGroup
    static const std::string PLANNING_GROUP = "panda_arm";
    MoveGroupInterface move_group(node, PLANNING_GROUP);

    RCLCPP_INFO(node->get_logger(), "MoveGroup initialized: %s", PLANNING_GROUP.c_str());

    // （2）初始化你写的控制接口
    coppelia_control::ManipulationInterface ctrl(node);

    // 设置 joint_names（必须要有）
    ctrl.setJointNames({
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7"
    });

    // 设置夹爪 joint names
    ctrl.setGripperJointNames({
        "panda_finger_joint1",
        "panda_finger_joint2"
    });

    // 设置话题（如果你的 CoppeliaSim 是这两个 topic）
    ctrl.setJointCmdTopic("/coppelia/joint_cmd");
    ctrl.setGripperCmdTopic("/coppelia/gripper_cmd");

    // 选择是否平滑模式 or P2P
    ctrl.useSmoothTrajectoryExecution(true);   // true = 平滑模式（只执行终点）
    // ctrl.useSmoothTrajectoryExecution(false); // false = P2P 模式（全部点）

    RCLCPP_INFO(node->get_logger(), "Starting planning...");


    // （3）给一个示例目标位姿
    geometry_msgs::msg::Pose target;
    target.position.x = 0.4;
    target.position.y = 0.0;
    target.position.z = 0.4;
    target.orientation.w = 1.0;

    move_group.setPoseTarget(target);

    // （4）开始规划
    MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);

    if(result == MoveGroupInterface::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Plan successful! Executing...");

        // 用你写的接口执行
        ctrl.executeTrajectory(plan.trajectory_);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
        rclcpp::shutdown();
        return 0;
    }

    // （5）测试夹爪
    std::this_thread::sleep_for(1s);
    ctrl.openGripper();
    std::this_thread::sleep_for(1s);
    ctrl.closeGripper();

    rclcpp::shutdown();
    return 0;
}

