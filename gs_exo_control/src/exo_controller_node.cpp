// gs_exo_control/src/exo_controller_node.cpp
// M6 外骨骼控制主节点：1kHz FOC 闭环控制
// P-Core 绑定，SCHED_FIFO rt_priority=90

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gs_msgs/msg/guidance_torque.hpp>
#include <gs_msgs/msg/safety_cmd.hpp>
#include <gs_msgs/msg/gait_state.hpp>
#include <gs_msgs/msg/motor_cmd.hpp>
#include <sys/mman.h>
#include <sched.h>
#include <cstring>
#include <thread>

#include "admittance_controller.hpp"
#include "vesc_interface.hpp"

class ExoControllerNode : public rclcpp::Node {
public:
  ExoControllerNode()
    : Node("exo_controller_node") {
    declare_parameter("kp_guide", 2.5f);
    declare_parameter("kd_guide", 0.8f);
    declare_parameter("kd_damp", 1.5f);
    declare_parameter("max_torque_nm", 3.0f);

    get_parameter("kp_guide", params_.kp_guide);
    get_parameter("kd_guide", params_.kd_guide);
    get_parameter("kd_damp", params_.kd_damp);
    get_parameter("max_torque_nm", params_.max_torque_nm);

    admittance_controller_ = std::make_unique<AdmittanceController>(params_);
    vesc_interface_ = std::make_unique<VescInterface>(shared_from_this());

    rclcpp::QoS qos(1);
    qos.reliable();

    gait_sub_ = create_subscription<gs_msgs::msg::GaitState>(
        "/gait_state", qos,
        [this](const gs_msgs::msg::GaitState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(gait_mutex_);
          current_gait_phase_ = msg->phase;
          gait_confidence_ = msg->confidence;
        });

    guidance_sub_ = create_subscription<gs_msgs::msg::GuidanceTorque>(
        "/guidance_torque", qos,
        [this](const gs_msgs::msg::GuidanceTorque::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(guidance_mutex_);
          latest_guidance_ = *msg;
          has_guidance_update_ = true;
        });

    safety_sub_ = create_subscription<gs_msgs::msg::SafetyCmd>(
        "/safety_cmd", qos,
        [this](const gs_msgs::msg::SafetyCmd::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(safety_mutex_);
          safety_state_ = msg->state;
          safety_damp_level_ = msg->damp_level;
        });

    state_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/state", qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          current_velocity_ = *msg;
        });

    motor_cmd_pub_ = create_publisher<gs_msgs::msg::MotorCmd>("/motor_cmd", qos);

    if (!setupRealtime()) {
      RCLCPP_WARN(get_logger(), "实时性设置失败，继续运行");
    }

    heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { sendHeartbeat(); });

    RCLCPP_INFO(get_logger(), "ExoControllerNode 已启动 (1kHz)");
    runControlLoop();
  }

private:
  bool setupRealtime() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      RCLCPP_WARN(get_logger(), "mlockall 失败");
      return false;
    }

    std::thread([this]() {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(0, &cpuset);

      pthread_t current_thread = pthread_self();
      if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        RCLCPP_WARN(get_logger(), "CPU 亲和性设置失败");
      }

      struct sched_param param;
      param.sched_priority = 90;
      if (pthread_setschedparam(current_thread, SCHED_FIFO, &param) != 0) {
        RCLCPP_WARN(get_logger(), "SCHED_FIFO 设置失败");
        return;
      }

      RCLCPP_INFO(get_logger(), "P-Core 绑定成功，优先级 90");
    }).detach();

    return true;
  }

  void runControlLoop() {
    control_thread_ = std::thread([this]() {
      rclcpp::Rate rate(1000);

      while (rclcpp::ok()) {
        auto start = std::chrono::high_resolution_clock::now();

        computeAndSendCommand();

        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        if (elapsed < 1000) {
          std::this_thread::sleep_for(std::chrono::microseconds(1000 - elapsed));
        }

        rate.sleep();
      }
    });
  }

  void computeAndSendCommand() {
    std::string gait_phase;
    float confidence;

    {
      std::lock_guard<std::mutex> lock(gait_mutex_);
      gait_phase = current_gait_phase_;
      confidence = gait_confidence_;
    }

    if (gait_phase.empty()) {
      gait_phase = "stance";
    }

    float safety_scale = 1.0f;
    float damp_level = 1.0f;
    std::string safety_state;

    {
      std::lock_guard<std::mutex> lock(safety_mutex_);
      safety_state = safety_state_;
      damp_level = safety_damp_level_;
    }

    if (safety_state == "VIRTUAL_WALL" || safety_state == "FALL_PROTECT" ||
        safety_state == "HIGH_DAMP_SAFE") {
      safety_scale = 0.0f;
    } else if (safety_state == "SLOW_DOWN") {
      safety_scale = 0.5f;
    }

    float kd_effective = params_.kd_guide;
    if (gait_phase == "stance") {
      kd_effective = params_.kd_damp;
    }

    float p_plan_x = 0.0f;
    float p_plan_y = 0.0f;

    {
      std::lock_guard<std::mutex> lock(guidance_mutex_);
      if (has_guidance_update_) {
        p_plan_x = latest_guidance_.left_nm * 0.1f;
        p_plan_y = latest_guidance_.right_nm * 0.1f;
        has_guidance_update_ = false;
      }
    }

    float dp_curr_x = 0.0f;
    float dp_curr_y = 0.0f;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      dp_curr_x = current_velocity_.linear.x;
      dp_curr_y = current_velocity_.linear.y;
    }

    auto [left_torque, right_torque] = admittance_controller_->compute(
        gait_phase, p_plan_x, p_plan_y, 0.0f, 0.0f, dp_curr_x, dp_curr_y);

    left_torque *= safety_scale * damp_level;
    right_torque *= safety_scale * damp_level;

    float left_current = left_torque * 10.0f;
    float right_current = right_torque * 10.0f;

    vesc_interface_->sendCurrentCommand(left_current, right_current);

    gs_msgs::msg::MotorCmd motor_cmd;
    motor_cmd.header.stamp = now();
    motor_cmd.left_current_a = left_current;
    motor_cmd.right_current_a = right_current;
    motor_cmd.stiffness_mode = (gait_phase == "swing") ? "guide" : "passive";
    motor_cmd.cable_tension_n = {0.0f, 0.0f};

    motor_cmd_pub_->publish(motor_cmd);
  }

  void sendHeartbeat() {
    last_heartbeat_ = now();
  }

  rclcpp::Subscription<gs_msgs::msg::GaitState>::SharedPtr gait_sub_;
  rclcpp::Subscription<gs_msgs::msg::GuidanceTorque>::SharedPtr guidance_sub_;
  rclcpp::Subscription<gs_msgs::msg::SafetyCmd>::SharedPtr safety_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr state_sub_;

  rclcpp::Publisher<gs_msgs::msg::MotorCmd>::SharedPtr motor_cmd_pub_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::unique_ptr<AdmittanceController> admittance_controller_;
  std::unique_ptr<VescInterface> vesc_interface_;

  std::thread control_thread_;
  rclcpp::Time last_heartbeat_;

  std::mutex gait_mutex_;
  std::mutex guidance_mutex_;
  std::mutex safety_mutex_;
  std::mutex state_mutex_;

  std::string current_gait_phase_ = "stance";
  float gait_confidence_ = 0.0f;

  gs_msgs::msg::GuidanceTorque latest_guidance_;
  bool has_guidance_update_ = false;

  std::string safety_state_ = "NORMAL";
  float safety_damp_level_ = 1.0f;

  geometry_msgs::msg::Twist current_velocity_;

  AdmittanceParams params_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExoControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
