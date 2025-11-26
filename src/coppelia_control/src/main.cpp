#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "coppelia_control/ManipulationInterface.hpp"
#include <thread>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("coppelia_control_main");

    // ----------------------------
    //  启动 ROS2 执行器（spin_some 防止死锁）
    // ----------------------------
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread executor_thread([&executor]() {
        while (rclcpp::ok()) {
            executor.spin_some();
            std::this_thread::sleep_for(5ms);
        }
    });

    // 等待 ROS2 回调系统完全启动
    std::this_thread::sleep_for(500ms);

    // ----------------------------
    //  MoveIt2 初始化
    // ----------------------------
    static const std::string PLANNING_GROUP = "panda_arm";
    MoveGroupInterface move_group(node, PLANNING_GROUP);

    RCLCPP_INFO(node->get_logger(), "MoveGroup initialized: %s", PLANNING_GROUP.c_str());

    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    // ----------------------------
    //  控制接口初始化
    // ----------------------------
    auto ctrl = std::make_shared<coppelia_control::ManipulationInterface>(node);

    std::map<std::string, std::string> joint_map = {
        {"panda_joint1", "joint__16__"},
        {"panda_joint2", "joint__19__"},
        {"panda_joint3", "joint__22__"},
        {"panda_joint4", "joint__24__"},
        {"panda_joint5", "joint__26__"},
        {"panda_joint6", "joint__28__"},
        {"panda_joint7", "joint__30__"}
    };
    ctrl->setMoveItToCoppeliaJointMap(joint_map);

    ctrl->setGripperJointNames({
        "centerJoint__39__",
        "openCloseJoint__45__"
    });

    ctrl->setJointCmdTopic("/joint_commands");
    ctrl->setGripperCmdTopic("/gripper_command");
    ctrl->useSmoothTrajectoryExecution(true);

    RCLCPP_INFO(node->get_logger(), "ManipulationInterface initialized");

    // ----------------------------
    //  设置目标位姿
    // ----------------------------
    geometry_msgs::msg::Pose target;
    target.position.x = 0.40;
    target.position.y = 0.00;
    target.position.z = 0.40;

    target.orientation.w = 1.0;
    target.orientation.x = 0.0;
    target.orientation.y = 0.0;
    target.orientation.z = 0.0;

    move_group.setPoseTarget(target);

    RCLCPP_INFO(node->get_logger(),
        "Target = [%.3f, %.3f, %.3f]",
        target.position.x,
        target.position.y,
        target.position.z
    );

    // ----------------------------
    //  开始规划
    // ----------------------------
    RCLCPP_INFO(node->get_logger(), "Planning...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto error_code = move_group.plan(plan);

    if (!bool(error_code)) {
        RCLCPP_ERROR(node->get_logger(),
            "Planning failed! error_code = %d",
            error_code.val
        );

        executor.cancel();
        executor_thread.join();
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Plan SUCCESS!");

    // 打印 MoveIt 的关节顺序
    RCLCPP_INFO(node->get_logger(), "MoveIt Joint Order:");
    for (auto &name : plan.trajectory_.joint_trajectory.joint_names)
        RCLCPP_INFO(node->get_logger(), "  -- %s", name.c_str());

    // ----------------------------
    //  执行轨迹
    // ----------------------------
    if (!ctrl->executeTrajectory(plan.trajectory_)) {
        RCLCPP_ERROR(node->get_logger(), "Trajectory execution FAILED!");
    } else {
        RCLCPP_INFO(node->get_logger(), "Trajectory executed SUCCESSFULLY!");
    }

    // ----------------------------
    //  测试夹爪
    // ----------------------------
    RCLCPP_INFO(node->get_logger(), "Testing gripper...");

    std::this_thread::sleep_for(1s);
    ctrl->openGripper();

    std::this_thread::sleep_for(1s);
    ctrl->closeGripper();

    // ----------------------------
    //  退出
    // ----------------------------
    std::this_thread::sleep_for(1s);

    executor.cancel();
    if (executor_thread.joinable())
        executor_thread.join();

    rclcpp::shutdown();
    return 0;
}
