#include <rclcpp/rclcpp.hpp>
#include "coppelia_control/PlanningInterface.hpp"
#include "coppelia_control/ManipulationInterface.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <atomic>
#include <chrono>
#include <map>

class FrankaControlNode : public rclcpp::Node
{
public:
    FrankaControlNode() : Node("franka_control_node")
    {
        // 1. 声明节点参数
        // 控制演示任务是否自动启动
        this->declare_parameter<bool>("auto_start_demo", true);
        // 规划组名称 (默认为 panda_arm，可通过 launch 修改)
        this->declare_parameter<std::string>("planning_group", "arm"); 
        
        RCLCPP_INFO(this->get_logger(), "节点创建成功，等待调用 initialize_interfaces()...");
    }

    ~FrankaControlNode()
    {
        // 析构时安全等待线程结束
        if (demo_thread_.joinable()) {
            demo_thread_.join();
        }
    }
    
    void initialize_interfaces()
    {
        RCLCPP_INFO(this->get_logger(), "正在初始化控制接口...");

        
        auto node_ptr_base = this->shared_from_this();
        
        try {
            // --- 步骤 1: 初始化规划层 (Planning) ---
            std::string planning_group_name;
            this->get_parameter("planning_group", planning_group_name);
            RCLCPP_INFO(this->get_logger(), "使用规划组: %s", planning_group_name.c_str());

            planning_interface_ = std::make_shared<coppelia_control::PlanningInterface>(node_ptr_base, planning_group_name);
            if (!planning_interface_->initialize()) {throw std::runtime_error("PlanningInterface 初始化失败");}
            // 可选：设置规划速度 (0.0 - 1.0)
            planning_interface_->setScalingFactors(0.5, 0.5);

            // --- 步骤 2: 初始化执行层 (Manipulation) ---
            manipulation_interface_ = std::make_shared<coppelia_control::ManipulationInterface>(node_ptr_base);
            
            // 【关键配置】设置关节名称映射表
            // Key: MoveIt/URDF 里的标准名字 (SRDF里定义的)
            // Value: CoppeliaSim 里通过 ros2 topic echo 看到的实际名字
            std::map<std::string, std::string> name_map = {
                {"joint_0", "/Franka/joint_0"},
                {"joint_1", "/Franka/joint_1"},
                {"joint_2", "/Franka/joint_2"},
                {"joint_3", "/Franka/joint_3"},
                {"joint_4", "/Franka/joint_4"},
                {"joint_5", "/Franka/joint_5"},
                {"joint_6", "/Franka/joint_6"},
                // 夹爪映射
                {"joint_7", "/Franka/FrankaGripper/joint_7"},
                {"Open_CloseJoint", "/Franka/FrankaGripper/Open_CloseJoint"} 
            };
            manipulation_interface_->setJointNameRemap(name_map);

            // 设置夹爪关节名 (这是 MoveIt 里的名字)
            manipulation_interface_->setGripperJointNames({"joint_7", "Open_CloseJoint"});
            
            // 启用平滑轨迹执行
            manipulation_interface_->useSmoothTrajectoryExecution(true);
            
            // 初始化 Publisher (建立与 CoppeliaSim 的通信)
            if (!manipulation_interface_->initialize()) {
                 throw std::runtime_error("ManipulationInterface Publisher 初始化失败");
            }
            
            RCLCPP_INFO(this->get_logger(), "所有接口初始化完成！");
            
            // --- 步骤 3: 启动演示任务线程 ---
            if (this->get_parameter("auto_start_demo").as_bool()) {
                demo_thread_ = std::thread(&FrankaControlNode::runDemoTask, this);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "初始化异常: %s", e.what());
        }
    }

private:
    // 演示任务主逻辑 (运行在独立线程，不会阻塞 ROS)
    void runDemoTask()
    {
        // 等待系统稳定 (等待 Publisher 和 Subscriber 建立连接)
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(this->get_logger(), "=== 开始执行自动化控制流程 ===");
        
        // --- 动作 1: 归位 (Ready) ---
        // 假设 SRDF 中定义了 "ready" 或 "home"
        RCLCPP_INFO(this->get_logger(), "[Action 1] 移动到 Ready 姿态");
        if (!moveToNamedTarget("ready")) {
            RCLCPP_WARN(this->get_logger(), "Ready 姿态规划失败，尝试 Home...");
            if (!moveToNamedTarget("home")) {
                RCLCPP_ERROR(this->get_logger(), "无法回到初始位置，请检查 SRDF 配置");
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // --- 动作 2: 准备抓取 (张开夹爪) ---
        RCLCPP_INFO(this->get_logger(), "[Action 2] 张开夹爪");
        manipulation_interface_->openGripper();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // --- 动作 3: 移动到抓取上方 ---
        RCLCPP_INFO(this->get_logger(), "[Action 3] 移动到抓取预备点");
        // 获取当前位置作为基准
        geometry_msgs::msg::Pose current_pose = planning_interface_->getCurrentPose();
        
        // 定义一个相对于当前位置的目标 (例如：向前 20cm, 向下 10cm)
        geometry_msgs::msg::Pose pre_grasp_pose = current_pose;
        pre_grasp_pose.position.x += 0.2; 
        pre_grasp_pose.position.z -= 0.1;
        
        if (!moveToPose(pre_grasp_pose)) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // --- 动作 4: 笛卡尔直线下降 ---
        RCLCPP_INFO(this->get_logger(), "[Action 4] 执行笛卡尔路径下探");
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(pre_grasp_pose); // 起点
        
        geometry_msgs::msg::Pose grasp_pose = pre_grasp_pose;
        grasp_pose.position.z -= 0.1; // 垂直下降 10cm
        waypoints.push_back(grasp_pose);
        
        if (!executeCartesianPath(waypoints)) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // --- 动作 5: 闭合夹爪 (模拟抓取) ---
        RCLCPP_INFO(this->get_logger(), "[Action 5] 闭合夹爪");
        manipulation_interface_->closeGripper();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // --- 动作 6: 带着物体抬起 ---
        RCLCPP_INFO(this->get_logger(), "[Action 6] 抬起物体");
        waypoints.clear();
        waypoints.push_back(grasp_pose); // 起点
        waypoints.push_back(pre_grasp_pose); // 终点 (回到预备点)
        
        if (!executeCartesianPath(waypoints)) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // --- 动作 7: 结束 ---
        RCLCPP_INFO(this->get_logger(), "=== 演示任务圆满完成 ===");
    }
    
    // --- 封装的辅助函数 (连接 Planning 和 Manipulation) ---

    bool moveToNamedTarget(const std::string& target_name)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // 1. 规划
        if (!planning_interface_->planToNamedTarget(target_name, plan)) {
            RCLCPP_ERROR(this->get_logger(), "规划到 %s 失败", target_name.c_str());
            return false;
        }
        // 2. 平滑
        planning_interface_->smoothTrajectory(plan.trajectory_);
        // 3. 执行
        return manipulation_interface_->executeTrajectory(plan.trajectory_);
    }

    bool moveToPose(const geometry_msgs::msg::Pose& target_pose)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!planning_interface_->planToPose(target_pose, plan)) {
            RCLCPP_ERROR(this->get_logger(), "Pose 规划失败");
            return false;
        }
        planning_interface_->smoothTrajectory(plan.trajectory_);
        return manipulation_interface_->executeTrajectory(plan.trajectory_);
    }

    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints)
    {
        moveit_msgs::msg::RobotTrajectory trajectory;
        if (!planning_interface_->planCartesianPath(waypoints, trajectory)) {
            RCLCPP_ERROR(this->get_logger(), "笛卡尔规划失败");
            return false;
        }
        planning_interface_->smoothTrajectory(trajectory);
        return manipulation_interface_->executeTrajectory(trajectory);
    }
    
    std::shared_ptr<coppelia_control::PlanningInterface> planning_interface_;
    std::shared_ptr<coppelia_control::ManipulationInterface> manipulation_interface_;
    std::thread demo_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaControlNode>();
    
    // 关键：在 shared_ptr 创建后初始化接口
    node->initialize_interfaces();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}