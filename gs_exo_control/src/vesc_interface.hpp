// gs_exo_control/src/vesc_interface.hpp
// VESC 通信封装：UART → VESC 电机驱动器

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gs_msgs/msg/motor_cmd.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class VescInterface {
public:
  VescInterface(rclcpp::Node::SharedPtr node);
  ~VescInterface();

  // 发送电流指令到 VESC
  void sendCurrentCommand(float left_current_a, float right_current_a);

  // 获取心跳状态
  bool isConnected() const { return connected_; }

private:
  void openUart(const std::string& port, int baudrate);
  void closeUart();

  int uart_fd_;
  bool connected_;
  rclcpp::Node::SharedPtr node_;
};
