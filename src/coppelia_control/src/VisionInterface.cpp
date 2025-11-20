#include "coppelia_control/VisionInterface.hpp"

namespace coppelia_control
{

VisionInterface::VisionInterface(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    // 允许外部修改 topic 名
    node_->declare_parameter<std::string>("vision_topic_name", vision_topic_name_);
    node_->get_parameter("vision_topic_name", vision_topic_name_);

    RCLCPP_INFO(node_->get_logger(),
                "VisionInterface initialized. Vision topic: %s",
                vision_topic_name_.c_str());
}

void VisionInterface::startListening()
{
    // 订阅目标位姿
    vision_subscription_ = node_->create_subscription<geometry_msgs::msg::Pose>(
        vision_topic_name_,
        10,
        std::bind(&VisionInterface::visionCallback, this)
    );

    RCLCPP_INFO(node_->get_logger(),
                "VisionInterface now listening to vision topic: %s",
                vision_topic_name_.c_str());
}

void VisionInterface::visionCallback()
{
    // 这里你头文件没有参数，因此我们只能做占位实现
    // 如果之后你想正式订阅 Pose，这里需要改成接收 msg

    geometry_msgs::msg::Pose dummy_pose;
    dummy_pose.position.x = 0.0;
    dummy_pose.position.y = 0.0;
    dummy_pose.position.z = 0.0;
    dummy_pose.orientation.w = 1.0;
    dummy_pose.orientation.x = dummy_pose.orientation.y = dummy_pose.orientation.z = 0.0;

    // 写入最新目标位姿
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_object_pose_ = dummy_pose;
    }

    RCLCPP_DEBUG(node_->get_logger(), "Dummy vision callback executed.");
}

std::optional<geometry_msgs::msg::Pose> VisionInterface::getLatestObjectPose()
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return latest_object_pose_;
}

}  // namespace coppelia_control
