// gs_safety/src/heartbeat_monitor.cpp
// M5 心跳监控：监控外骨骼控制节点心跳，超时则触发 HIGH_DAMP_SAFE

#include <rclcpp/rclcpp.hpp>
#include <gs_msgs/msg/motor_cmd.hpp>
#include <gs_msgs/msg/safety_cmd.hpp>
#include "fsm_states.hpp"

class HeartbeatMonitor : public rclcpp::Node {
public:
  HeartbeatMonitor()
    : Node("heartbeat_monitor"), last_heartbeat_time_(now()), timeout_ms_(200.0) {
    declare_parameter("heartbeat_timeout_ms", 200.0);
    get_parameter("heartbeat_timeout_ms", timeout_ms_);

    motor_sub_ = create_subscription<gs_msgs::msg::MotorCmd>(
        "/motor_cmd", 10,
        [this](const gs_msgs::msg::MotorCmd::SharedPtr msg) {
          (void)msg;
          last_heartbeat_time_ = now();
        });

    safety_pub_ = create_publisher<gs_msgs::msg::SafetyCmd>("/safety_cmd_heartbeat", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
        [this]() { checkHeartbeat(); });

    RCLCPP_INFO(get_logger(), "心跳监控已启动，超时 %.1f ms", timeout_ms_);
  }

private:
  void checkHeartbeat() {
    auto elapsed = (now() - last_heartbeat_time_).seconds() * 1000.0;
    if (elapsed > timeout_ms_) {
      gs_msgs::msg::SafetyCmd cmd;
      cmd.header.stamp = now();
      cmd.state = fsm_state_to_string(FSMState::HIGH_DAMP_SAFE);
      cmd.brake = true;
      cmd.damp_level = 1.0f;
      cmd.alert_text = "外骨骼心跳丢失，进入高阻尼安全模式";
      safety_pub_->publish(cmd);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "心跳超时: %.0f ms", elapsed);
    }
  }

  rclcpp::Subscription<gs_msgs::msg::MotorCmd>::SharedPtr motor_sub_;
  rclcpp::Publisher<gs_msgs::msg::SafetyCmd>::SharedPtr safety_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_heartbeat_time_;
  double timeout_ms_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatMonitor>());
  rclcpp::shutdown();
  return 0;
}
