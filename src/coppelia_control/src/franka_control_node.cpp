#include <rclcpp/rclcpp.hpp>
#include "coppelia_control/PlanningInterface.hpp"
#include "coppelia_control/ManipulationInterface.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <atomic>
#include <thread>

using std::placeholders::_1;

class FrankaCartesianGrasp : public rclcpp::Node
{
public:
    FrankaCartesianGrasp() : Node("franka_cartesian_grasp")
    {
        declare_parameter<std::string>("arm_group", "arm");
        declare_parameter<std::string>("hand_group", "hand");
        
        // ⚠️ 根据实际场景调整的参数
        declare_parameter<double>("safe_height", 0.35);      // 安全高度（不碰到任何东西）
        declare_parameter<double>("approach_height", 0.15);  // 接近高度（能清晰看到方块）
        declare_parameter<double>("grasp_offset", -0.01);    // 抓取偏移（负值=下探，正值=上浮）
        
        enable_vision_ = false;
        has_pose_ = false;
        pose_count_ = 0;
        
        RCLCPP_INFO(get_logger(), "Franka Cartesian Grasp Node Initialized");
    }

    void init()
    {
        auto self = shared_from_this();
        std::string arm, hand;
        get_parameter("arm_group", arm);
        get_parameter("hand_group", hand);

        planner_ = std::make_shared<coppelia_control::PlanningInterface>(self, arm);
        planner_->initialize();
        planner_->setScalingFactors(0.3, 0.3);  // 降低速度提高精度

        exec_ = std::make_shared<coppelia_control::ManipulationInterface>(self);
        exec_->setJointCmdTopic("/coppelia/joint_cmd");
        exec_->setGripperCmdTopic("/coppelia/gripper_cmd");
        exec_->useSmoothTrajectoryExecution(true);
        exec_->initialize();

        vision_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/object_pose", 10,
            std::bind(&FrankaCartesianGrasp::visionCB, this, _1));

        RCLCPP_INFO(get_logger(), "Starting grasp thread...");
        std::thread(&FrankaCartesianGrasp::run, this).detach();
    }

private:
    /* ================= Vision ================= */
    void visionCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!enable_vision_) return;
        
        // 累积多帧取平均，降低抖动
        pose_sum_.position.x += msg->pose.position.x;
        pose_sum_.position.y += msg->pose.position.y;
        pose_sum_.position.z += msg->pose.position.z;
        pose_count_++;
        
        if (pose_count_ >= 5) {  // 累积5帧
            cached_pose_.position.x = pose_sum_.position.x / pose_count_;
            cached_pose_.position.y = pose_sum_.position.y / pose_count_;
            cached_pose_.position.z = pose_sum_.position.z / pose_count_;
            cached_pose_.orientation.w = 1.0;  // 默认姿态
            has_pose_ = true;
            
            RCLCPP_DEBUG(get_logger(), "Vision averaged (%d frames): [%.3f, %.3f, %.3f]",
                        pose_count_,
                        cached_pose_.position.x,
                        cached_pose_.position.y,
                        cached_pose_.position.z);
        }
    }

    geometry_msgs::msg::Pose visionOnce(double timeout = 1.5)
    {
        has_pose_ = false;
        pose_count_ = 0;
        pose_sum_ = geometry_msgs::msg::Pose();
        
        enable_vision_ = true;
        auto start = std::chrono::steady_clock::now();
        
        while (!has_pose_ && rclcpp::ok()) {
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration<double>(elapsed).count() > timeout) {
                RCLCPP_WARN(get_logger(), "Vision timeout after %.1fs!", timeout);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            rclcpp::spin_some(shared_from_this());
        }
        
        enable_vision_ = false;
        return cached_pose_;
    }

    /* ================= Motion ================= */
    bool cartesianTo(const geometry_msgs::msg::Pose& target, double speed_factor = 1.0)
    {
        auto start = planner_->getCurrentPose();
        std::vector<geometry_msgs::msg::Pose> wp{start, target};
        
        // 临时调整速度
        planner_->setScalingFactors(0.3 * speed_factor, 0.3 * speed_factor);
        
        moveit_msgs::msg::RobotTrajectory traj;
        bool success = planner_->planCartesianPath(wp, traj);
        
        // 恢复默认速度
        planner_->setScalingFactors(0.3, 0.3);
        
        if (!success) {
            RCLCPP_ERROR(get_logger(), "Cartesian planning failed!");
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Executing trajectory...");
        return exec_->executeTrajectory(traj);
    }

    bool goNamed(const std::string& name)
    {
        RCLCPP_INFO(get_logger(), "Moving to named target: %s", name.c_str());
        moveit::planning_interface::MoveGroupInterface::Plan p;
        if (!planner_->planToNamedTarget(name, p)) {
            RCLCPP_ERROR(get_logger(), "Failed to plan to: %s", name.c_str());
            return false;
        }
        return exec_->executeTrajectory(p.trajectory_);
    }

    void gripper(const std::string& cmd)
    {
        RCLCPP_INFO(get_logger(), "Gripper: %s", cmd.c_str());
        planner_->changePlanningGroup("hand");
        moveit::planning_interface::MoveGroupInterface::Plan p;
        if (planner_->planToNamedTarget(cmd, p))
            exec_->executeTrajectory(p.trajectory_);
        planner_->changePlanningGroup("arm");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    geometry_msgs::msg::Quaternion downQuat()
    {
        geometry_msgs::msg::Quaternion q;
        q.x = 1.0; q.y = 0.0; q.z = 0.0; q.w = 0.0;  // 向下
        return q;
    }

    /* ================= Main Logic ================= */
    void run()
    {
        // 从参数读取
        double SAFE_Z, APPROACH_Z, GRASP_OFFSET;
        get_parameter("safe_height", SAFE_Z);
        get_parameter("approach_height", APPROACH_Z);
        get_parameter("grasp_offset", GRASP_OFFSET);

        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "Grasp Parameters:");
        RCLCPP_INFO(get_logger(), "  Safe Height:     %.3f m", SAFE_Z);
        RCLCPP_INFO(get_logger(), "  Approach Height: %.3f m", APPROACH_Z);
        RCLCPP_INFO(get_logger(), "  Grasp Offset:    %.3f m", GRASP_OFFSET);
        RCLCPP_INFO(get_logger(), "===========================================\n");

        // 等待系统稳定
        RCLCPP_INFO(get_logger(), "Waiting for system to stabilize...");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        while (rclcpp::ok())
        {
            RCLCPP_INFO(get_logger(), "\n========== NEW GRASP CYCLE ==========\n");

            // ====== Stage 0: 回到起始位置 ======
            RCLCPP_INFO(get_logger(), "[Stage 0] Moving to home position...");
            if (!goNamed("home")) {
                RCLCPP_ERROR(get_logger(), "Failed to reach home, retrying...");
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            
            gripper("open");

            // ====== Stage 1: 在安全高度初步定位 ======
            RCLCPP_INFO(get_logger(), "[Stage 1] Moving to safe height for initial detection...");
            
            // 移动到工作空间上方
            geometry_msgs::msg::Pose pre_look;
            pre_look.position.x = -0.13;  // 基于你的检测数据：x ≈ -0.13
            pre_look.position.y = -0.18;  // y ≈ -0.18
            pre_look.position.z = SAFE_Z;
            pre_look.orientation = downQuat();
            
            if (!cartesianTo(pre_look, 0.8)) {
                RCLCPP_ERROR(get_logger(), "Failed to reach pre-look position");
                continue;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // ====== Stage 2: 第一次视觉检测（粗定位）======
            RCLCPP_INFO(get_logger(), "[Stage 2] Vision #1 - Coarse detection...");
            auto p1 = visionOnce(2.0);
            
            if (pose_count_ == 0) {
                RCLCPP_WARN(get_logger(), "No object detected, retrying cycle...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            
            RCLCPP_INFO(get_logger(), "  Detected at: [%.3f, %.3f, %.3f]",
                       p1.position.x, p1.position.y, p1.position.z);

            // ====== Stage 3: 移动到接近高度精确对齐 ======
            RCLCPP_INFO(get_logger(), "[Stage 3] Moving to approach height...");
            
            geometry_msgs::msg::Pose approach;
            approach.position.x = p1.position.x;
            approach.position.y = p1.position.y;
            approach.position.z = APPROACH_Z;
            approach.orientation = downQuat();
            
            if (!cartesianTo(approach, 0.5)) {
                RCLCPP_ERROR(get_logger(), "Failed to reach approach position");
                continue;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // ====== Stage 4: 第二次视觉检测（精确定位）======
            RCLCPP_INFO(get_logger(), "[Stage 4] Vision #2 - Precise detection...");
            auto p2 = visionOnce(2.0);
            
            if (pose_count_ == 0) {
                RCLCPP_WARN(get_logger(), "Lost object, going back...");
                continue;
            }
            
            RCLCPP_INFO(get_logger(), "  Precise location: [%.3f, %.3f, %.3f]",
                       p2.position.x, p2.position.y, p2.position.z);

            // ====== Stage 5: 执行抓取 ======
            RCLCPP_INFO(get_logger(), "[Stage 5] Executing grasp...");
            
            geometry_msgs::msg::Pose grasp;
            grasp.position.x = p2.position.x;
            grasp.position.y = p2.position.y;
            
            // ⚠️ 关键：p2.position.z 已经是物体顶面的世界坐标
            grasp.position.z = p2.position.z + GRASP_OFFSET;
            grasp.orientation = downQuat();
            
            // 安全检查
            if (grasp.position.z < 0.02) {
                RCLCPP_ERROR(get_logger(), "Grasp height too low (%.3f m), ABORT!", 
                           grasp.position.z);
                continue;
            }
            
            RCLCPP_INFO(get_logger(), "  Target grasp pose: [%.3f, %.3f, %.3f]",
                       grasp.position.x, grasp.position.y, grasp.position.z);
            
            if (!cartesianTo(grasp, 0.3)) {
                RCLCPP_ERROR(get_logger(), "Failed to reach grasp pose!");
                continue;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            RCLCPP_INFO(get_logger(), "  Closing gripper...");
            gripper("close");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // ====== Stage 6: 提升物体 ======
            RCLCPP_INFO(get_logger(), "[Stage 6] Lifting object...");
            
            geometry_msgs::msg::Pose lift = grasp;
            lift.position.z = APPROACH_Z;
            
            if (!cartesianTo(lift, 0.5)) {
                RCLCPP_WARN(get_logger(), "Lift failed, object may have dropped");
            }

            // ====== Stage 7: 返回起始位置 ======
            RCLCPP_INFO(get_logger(), "[Stage 7] Returning home...");
            goNamed("home");
            
            // 可选：在指定位置放下
            // gripper("open");

            RCLCPP_INFO(get_logger(), "\n========== CYCLE COMPLETE ==========\n");
            
            // 等待下一次循环
            RCLCPP_INFO(get_logger(), "Waiting 3 seconds before next cycle...\n");
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    std::shared_ptr<coppelia_control::PlanningInterface> planner_;
    std::shared_ptr<coppelia_control::ManipulationInterface> exec_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub_;

    std::atomic<bool> enable_vision_;
    bool has_pose_;
    geometry_msgs::msg::Pose cached_pose_;
    geometry_msgs::msg::Pose pose_sum_;
    int pose_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaCartesianGrasp>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(node);
    exe.spin();
    rclcpp::shutdown();
    return 0;
}