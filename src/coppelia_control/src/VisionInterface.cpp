#include "coppelia_control/VisionInterface.hpp"

namespace coppelia_control
{

VisionInterface::VisionInterface(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    // 这里可以提前声明参数（可选）
    node_->declare_parameter<std::string>("vision.topic", "/vision/target_pose");
    node_->get_parameter("vision.topic", vision_topic_name_);
}

void VisionInterface::startListening()
{
    // 最简订阅器，只为了能编译
    vision_subscription_ = node_->create_subscription<geometry_msgs::msg::Pose>(
        vision_topic_name_,
        10,
        [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            latest_object_pose_ = *msg;  // 存一份
        }
    );
}

std::optional<geometry_msgs::msg::Pose> VisionInterface::getLatestObjectPose()
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return latest_object_pose_;  // 可能是空
}

}  // namespace coppelia_control

