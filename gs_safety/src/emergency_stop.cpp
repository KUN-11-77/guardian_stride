#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gs_msgs/msg/safety_cmd.hpp>

namespace guardian_stride {
namespace safety {

class EmergencyStop : public rclcpp::Node {
public:
    EmergencyStop();

private:
    void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void publishEStop(bool stop);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Publisher<gs_msgs::msg::SafetyCmd>::SharedPtr cmd_pub_;
    bool estop_active_;
};

EmergencyStop::EmergencyStop()
    : Node("emergency_stop"),
      estop_active_(false)
{
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/estop_trigger", 10,
        std::bind(&EmergencyStop::estopCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<gs_msgs::msg::SafetyCmd>("/safety_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "Emergency stop node started");
}

void EmergencyStop::estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    estop_active_ = msg->data;
    publishEStop(estop_active_);
}

void EmergencyStop::publishEStop(bool stop) {
    gs_msgs::msg::SafetyCmd cmd;
    cmd.stiffness_mode = stop ? "LOCK" : "FREE";
    cmd_pub_->publish(cmd);

    RCLCPP_WARN(this->get_logger(), "Emergency stop %s", stop ? "TRIGGERED" : "CLEARED");
}

}  // namespace safety
}  // namespace guardian_stride

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<guardian_stride::safety::EmergencyStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}