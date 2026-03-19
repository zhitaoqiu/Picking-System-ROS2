#include "cs_hardware_interface/cs_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cs_hardware_interface
{

CsSystem::~CsSystem()
{
  // 析构函数：即使程序崩溃退出，也要尝试释放资源
  if (zmq_socket_) {
    zmq_close(zmq_socket_);
    zmq_socket_ = nullptr;
  }
  if (zmq_context_) {
    zmq_ctx_destroy(zmq_context_);
    zmq_context_ = nullptr;
  }
}

hardware_interface::CallbackReturn CsSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 1. 分配内存
  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  
  // 打印关节顺序，用于调试！这非常关键！
  RCLCPP_INFO(rclcpp::get_logger("CsSystem"), "=== Joint Order Check ===");
  for (size_t i = 0; i < info_.joints.size(); i++) {
      RCLCPP_INFO(rclcpp::get_logger("CsSystem"), "Index %ld: %s", i, info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("CsSystem"), "=========================");

  // 2. 初始化 ZMQ Context
  zmq_context_ = zmq_ctx_new();
  zmq_socket_ = zmq_socket(zmq_context_, ZMQ_REQ); // REQ 模式：一问一答

  // 3. 设置超时 (防止仿真卡死导致 ROS 死锁)
  int timeout_ms = 100; // 100ms 超时
  zmq_setsockopt(zmq_socket_, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
  zmq_setsockopt(zmq_socket_, ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));

  // 4. 连接
  int rc = zmq_connect(zmq_socket_, "tcp://localhost:5555");
  if (rc != 0) {
     RCLCPP_FATAL(rclcpp::get_logger("CsSystem"), "Failed to connect to ZMQ Server! RC=%d", rc);
     return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("CsSystem"), "ZMQ Initialized and Connected.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CsSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 如果需要重置参数可以在这里做，目前留空即可
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    
    // 如果URDF里加了Velocity，这里也需要导出，目前只用了Position
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CsSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn CsSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("CsSystem"), "System Activated!");
  
  // 可以在这里发一个 Reset 信号给 CoppeliaSim，视需求而定
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
      if (std::isnan(hw_states_[i])) {
          hw_commands_[i] = 0.0;
      } else {
          hw_commands_[i] = hw_states_[i];
      }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CsSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CsSystem"), "System Deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CsSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. 构造请求：获取状态
  std::string req = "get_state";
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  int send_rc = zmq_send(zmq_socket_, req.c_str(), req.length(), 0);
  
  if (send_rc == -1) {
      // 这里的 logger 加 throttle 防止刷屏
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("CsSystem"), clock, 2000, "ZMQ Send Error in read()");
      return hardware_interface::return_type::OK; 
  }

  // 2. 接收数据
  char buffer[2048];
  int size = zmq_recv(zmq_socket_, buffer, 2048, 0);

  if (size < 0) {
      // 超时或错误
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CsSystem"), clock, 2000, "ZMQ Recv Timeout/Error in read() - Check CoppeliaSim status");
      return hardware_interface::return_type::OK; // 容忍丢包
  }
  
  // 3. 解析数据 "q1,q2,q3..."
  buffer[size] = '\0';
  parse_feedback(std::string(buffer));

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CsSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. 构造命令字符串 "set:q1,q2,q3..."
  std::stringstream ss;
  ss << "set:";
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
      ss << hw_commands_[i];
      if (i < hw_commands_.size() - 1) ss << ",";
  }
  std::string send_str = ss.str();
  static rclcpp::Clock clock(RCL_STEADY_TIME);

  // 2. 发送命令
  zmq_send(zmq_socket_, send_str.c_str(), send_str.length(), 0);

  // 3. 接收确认 (REQ-REP 模式必须有一来一回，否则下次 send 会报错)
  char buffer[128];
  int size = zmq_recv(zmq_socket_, buffer, 128, 0);
  
  // 只要能收到东西就行，不需要太关心内容，除非是为了查错
  if (size < 0) {
       RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CsSystem"), clock, 2000, "ZMQ Recv Timeout in write()");
             zmq_close(zmq_socket_);
      zmq_socket_ = zmq_socket(zmq_context_, ZMQ_REQ);
      int timeout_ms = 100;
      zmq_setsockopt(zmq_socket_, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
      zmq_setsockopt(zmq_socket_, ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));
      zmq_connect(zmq_socket_, "tcp://localhost:5555");
  }
  return hardware_interface::return_type::OK;
}

void CsSystem::parse_feedback(const std::string & data) {
    // 简单的 CSV 解析器
    // 假设数据格式严格为: "val1,val2,val3"

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    std::stringstream ss(data);
    std::string segment;
    int i = 0;
    
    while(std::getline(ss, segment, ',') && i < (int)hw_states_.size())
    {
       try {
           hw_states_[i] = std::stod(segment);
       } catch (const std::exception& e) {
           RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("CsSystem"), clock, 5000, "Data parse error: %s", segment.c_str());
       }
       i++;
    }
}

}  // namespace cs_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cs_hardware_interface::CsSystem, hardware_interface::SystemInterface)