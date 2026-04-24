#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gs_msgs/msg/gait_state.hpp>
#include <gs_msgs/msg/traversability.hpp>

namespace guardian_stride {
namespace safety {

class SafetyMonitor : public rclcpp::Node {
public:
    SafetyMonitor();

private:
    void gaitCallback(const gs_msgs::msg::GaitState::SharedPtr msg);
    void traversabilityCallback(const gs_msgs::msg::Traversability::SharedPtr msg);
    void checkSafety();

    rclcpp::Subscription<gs_msgs::msg::GaitState>::SharedPtr gait_sub_;
    rclcpp::Subscription<gs_msgs::msg::Traversability>::SharedPtr traversability_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr risk_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double max_speed_;
    double risk_threshold_;
    double current_risk_;
    int unsafe_count_;
};

SafetyMonitor::SafetyMonitor()
    : Node("safety_monitor"),
      max_speed_(1.0),
      risk_threshold_(0.7),
      current_risk_(0.0),
      unsafe_count_(0)
{
    this->declare_parameter("max_speed", 1.0);
    this->declare_parameter("risk_threshold", 0.7);

    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("risk_threshold", risk_threshold_);

    gait_sub_ = this->create_subscription<gs_msgs::msg::GaitState>(
        "/gait_state", 10,
        std::bind(&SafetyMonitor::gaitCallback, this, std::placeholders::_1));

    traversability_sub_ = this->create_subscription<gs_msgs::msg::Traversability>(
        "/traversability", 10,
        std::bind(&SafetyMonitor::traversabilityCallback, this, std::placeholders::_1));

    risk_pub_ = this->create_publisher<std_msgs::msg::Float32>("/safety_risk", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SafetyMonitor::checkSafety, this));

    RCLCPP_INFO(this->get_logger(), "Safety monitor started");
}

void SafetyMonitor::gaitCallback(const gs_msgs::msg::GaitState::SharedPtr msg) {
    if (msg->phase == "FALLING" || msg->phase == "STUMBLING") {
        current_risk_ = 0.9;
    }
}

void SafetyMonitor::traversabilityCallback(const gs_msgs::msg::Traversability::SharedPtr msg) {
    if (!msg->data.empty()) {
        float min_val = msg->data[0];
        for (size_t i = 1; i < msg->data.size(); ++i) {
            if (msg->data[i] < min_val) min_val = msg->data[i];
        }
        if (min_val < 0.3f) {
            unsafe_count_++;
            if (unsafe_count_ > 5) {
                current_risk_ = 0.8;
            }
        } else {
            unsafe_count_ = 0;
            current_risk_ = std::max(0.0, current_risk_ - 0.1);
        }
    }
}

void SafetyMonitor::checkSafety() {
    std_msgs::msg::Float32 msg;
    msg.data = current_risk_;
    risk_pub_->publish(msg);
}

}  // namespace safety
}  // namespace guardian_stride

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<guardian_stride::safety::SafetyMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}