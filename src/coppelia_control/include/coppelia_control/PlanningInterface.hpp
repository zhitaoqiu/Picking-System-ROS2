#pragma onece
#include <rclcpp/rclcpp.hpp>
#include<moveit/moveit_group_interface/move_group_interface.h>
#include<moveit_msgs/msg/robot_trajectory.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<string>
#include<vector>

namespace coppelia_control
{
class PlanningInterface
{
public:
    explicit PlanningInterface(const rclcpp::Node::SharedPtr& node);
    bool initialize();
    bool planToPose(const geometry_msgs::msg::Pose& target_pose,moveit::planning_interface::MoveGroupInterface::Plan& plan);
    bool planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,moveit_msgs::msg::RobotTrajectory& trajectory);
    bool planToNamedTarget(const std::string& target_name,moveit::planning_interface::MoveGroupInterface::Plan& plan);
    void smoothTrajectory(moveit_msgs::msg::RobotTrajectory& trajectory);
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    const std::string PLANNING_GROUP ="panda_arm";
};
}