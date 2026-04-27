// gs_safety/src/safety_fsm_node.cpp
// M5 安全反射层主节点：5 状态 FSM，50Hz E-Core 绑定
// 输入：ToF 距离 /imu/data /state /guidance_torque
// 输出：/safety_cmd（覆盖 M4，写入 M6）

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gs_msgs/msg/guidance_torque.hpp>
#include <gs_msgs/msg/safety_cmd.hpp>
#include <memory>
#include <mutex>

#include "fsm_states.hpp"

class SafetyFSMNode : public rclcpp::Node {
public:
  SafetyFSMNode()
    : Node("safety_fsm_node"),
      current_state_(FSMState::NORMAL),
      fall_detected_(false),
      heartbeat_timeout_(false) {
    declare_parameter("tof_slow_down_m", 2.0);
    declare_parameter("tof_virtual_wall_m", 0.5);
    declare_parameter("imu_fall_pitch_deg", 30.0);
    declare_parameter("imu_fall_roll_deg", 30.0);

    get_parameter("tof_slow_down_m", tof_slow_down_m_);
    get_parameter("tof_virtual_wall_m", tof_virtual_wall_m_);
    get_parameter("imu_fall_pitch_deg", imu_fall_pitch_deg_);
    get_parameter("imu_fall_roll_deg", imu_fall_roll_deg_);

    // 订阅 ToF 距离
    tof_sub_ = create_subscription<sensor_msgs::msg::Range>(
        "/tof/distances", 10,
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_tof_range_ = msg->range;
        });

    // 订阅 IMU 数据（跌倒检测）
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 100,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          // 使用加速度估算姿态变化
          double ax = msg->linear_acceleration.x;
          double ay = msg->linear_acceleration.y;
          double az = msg->linear_acceleration.z;
          double accel_norm = std::sqrt(ax*ax + ay*ay + az*az);
          if (accel_norm < 1.0 || accel_norm > 20.0) {
            fall_detected_ = true;
          }
        });

    // 安全心跳超时订阅
    heartbeat_sub_ = create_subscription<gs_msgs::msg::SafetyCmd>(
        "/safety_cmd_heartbeat", 10,
        [this](const gs_msgs::msg::SafetyCmd::SharedPtr msg) {
          if (msg->state == "HIGH_DAMP_SAFE") {
            heartbeat_timeout_ = true;
          }
        });

    // 发布安全指令
    safety_pub_ = create_publisher<gs_msgs::msg::SafetyCmd>("/safety_cmd", 10);

    // 50Hz FSM 循环
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
        [this]() { updateFSM(); });

    RCLCPP_INFO(get_logger(), "安全 FSM 已启动 (50Hz, E-Core)");
  }

private:
  void updateFSM() {
    std::lock_guard<std::mutex> lock(mutex_);

    FSMState new_state = current_state_;

    // 跌倒检测优先级最高
    if (fall_detected_) {
      new_state = FSMState::FALL_PROTECT;
    }
    // 心跳超时
    else if (heartbeat_timeout_) {
      new_state = FSMState::HIGH_DAMP_SAFE;
    }
    // ToF 距离判断
    else if (latest_tof_range_ >= 0.0f) {
      if (latest_tof_range_ < tof_virtual_wall_m_) {
        new_state = FSMState::VIRTUAL_WALL;
      } else if (latest_tof_range_ < tof_slow_down_m_) {
        new_state = FSMState::SLOW_DOWN;
      } else {
        new_state = FSMState::NORMAL;
      }
    }
    // 无 ToF 数据恢复 NORMAL
    else {
      new_state = FSMState::NORMAL;
    }

    // 状态变化时发布
    if (new_state != current_state_) {
      current_state_ = new_state;
      publishSafetyCmd();
    }
  }

  void publishSafetyCmd() {
    gs_msgs::msg::SafetyCmd cmd;
    cmd.header.stamp = now();
    cmd.state = fsm_state_to_string(current_state_);
    cmd.brake = (current_state_ == FSMState::VIRTUAL_WALL ||
                 current_state_ == FSMState::FALL_PROTECT ||
                 current_state_ == FSMState::HIGH_DAMP_SAFE);
    cmd.damp_level = (current_state_ == FSMState::NORMAL) ? 0.0f : 1.0f;

    switch (current_state_) {
      case FSMState::NORMAL:
        cmd.alert_text = "";
        break;
      case FSMState::SLOW_DOWN:
        cmd.alert_text = "前方障碍物，请减速";
        break;
      case FSMState::VIRTUAL_WALL:
        cmd.alert_text = "停！前方危险";
        break;
      case FSMState::FALL_PROTECT:
        cmd.alert_text = "检测到跌倒，锁定关节";
        break;
      case FSMState::HIGH_DAMP_SAFE:
        cmd.alert_text = "系统异常，进入安全模式";
        break;
    }

    safety_pub_->publish(cmd);
  }

  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<gs_msgs::msg::SafetyCmd>::SharedPtr heartbeat_sub_;

  // 发布者
  rclcpp::Publisher<gs_msgs::msg::SafetyCmd>::SharedPtr safety_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;

  // FSM 状态
  FSMState current_state_;
  float latest_tof_range_ = -1.0f;
  bool fall_detected_;
  bool heartbeat_timeout_;

  // 阈值
  float tof_slow_down_m_;
  float tof_virtual_wall_m_;
  float imu_fall_pitch_deg_;
  float imu_fall_roll_deg_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyFSMNode>());
  rclcpp::shutdown();
  return 0;
}
