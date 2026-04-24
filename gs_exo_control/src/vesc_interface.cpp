// gs_exo_control/src/vesc_interface.cpp
// VESC UART 通信封装

#include "vesc_interface.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>

VescInterface::VescInterface(rclcpp::Node::SharedPtr node)
  : uart_fd_(-1)
  , connected_(false)
  , node_(node) {
  node_->declare_parameter("vesc_uart_port", std::string("/dev/ttyUSB0"));
  node_->declare_parameter("vesc_uart_baudrate", 115200);

  std::string port;
  int baudrate;
  node_->get_parameter("vesc_uart_port", port);
  node_->get_parameter("vesc_uart_baudrate", baudrate);

  openUart(port, baudrate);
}

VescInterface::~VescInterface() {
  closeUart();
}

void VescInterface::openUart(const std::string& port, int baudrate) {
  uart_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (uart_fd_ < 0) {
    RCLCPP_WARN(node_->get_logger(), "无法打开 UART %s", port.c_str());
    return;
  }

  struct termios tty;
  if (tcgetattr(uart_fd_, &tty) != 0) {
    RCLCPP_ERROR(node_->get_logger(), "tcgetattr 失败");
    closeUart();
    return;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(node_->get_logger(), "tcsetattr 失败");
    closeUart();
    return;
  }

  connected_ = true;
  RCLCPP_INFO(node_->get_logger(), "VESC UART 已连接: %s", port.c_str());
}

void VescInterface::closeUart() {
  if (uart_fd_ >= 0) {
    ::close(uart_fd_);
    uart_fd_ = -1;
  }
  connected_ = false;
}

void VescInterface::sendCurrentCommand(float left_current_a, float right_current_a) {
  if (!connected_ || uart_fd_ < 0) return;

  // VESC 协议：简单打包（实际项目中应使用完整的 VESC 协议栈）
  // 这里发送原始字节流，格式：0x05 + 左电流(4字节float) + 右电流(4字节float)
  uint8_t packet[9];
  packet[0] = 0x05;  // 命令字：设置电流

  float left_val = std::clamp(left_current_a, -50.0f, 50.0f);
  float right_val = std::clamp(right_current_a, -50.0f, 50.0f);

  std::memcpy(&packet[1], &left_val, 4);
  std::memcpy(&packet[5], &right_val, 4);

  ssize_t written = write(uart_fd_, packet, sizeof(packet));
  if (written < 0) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "VESC 写入失败");
  }
}
