#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp> // 用于表示目标的位姿
#include <string>
#include <memory>
#include <optional>
#include <mutex>

namespace coppelia_control
{
class VisionInterface
{
public:
    explicit VisionInterface(const rclcpp::Node::SharedPtr& node);
    void startListening();
    std::optional<geometry_msgs::msg::Pose> getLatestObjectPose();
private:
    void visionCallback();
    rclcpp::Node::SharedPtr node_;
    std::mutex pose_mutex_;
    std::optional<geometry_msgs::msg::Pose> latest_object_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr vision_subscription_;
    std::string vision_topic_name_="/vision/target_pose";

};
}