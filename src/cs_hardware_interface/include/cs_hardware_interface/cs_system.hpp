#ifndef CS_HARDWARE_INTERFACE__CS_SYSTEM_HPP_
#define CS_HARDWARE_INTERFACE__CS_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include <zmq.h> // 确保你安装了 libzmq3-dev

namespace cs_hardware_interface
{
class CsSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CsSystem)

  ~CsSystem(); 

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ZMQ 相关变量
  void * zmq_context_;
  void * zmq_socket_;
  
  // 存储关节数据的容器
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_; // 预留，即便URDF没配，代码里最好留着结构

  // 辅助函数
  void parse_feedback(const std::string & data);
};

}  // namespace cs_hardware_interface

#endif  // CS_HARDWARE_INTERFACE__CS_SYSTEM_HPP_