#include "collision_checker.hpp"
#include <chrono>

namespace guardian_stride {
namespace safety {

CollisionChecker::CollisionChecker()
    : Node("collision_checker"),
      min_safe_distance_(0.5),
      front_angle_start_(-M_PI_4),
      front_angle_end_(M_PI_4)
{
    this->declare_parameter("min_safe_distance", 0.5);
    this->declare_parameter("front_angle_start", -M_PI_4);
    this->declare_parameter("front_angle_end", M_PI_4);

    this->get_parameter("min_safe_distance", min_safe_distance_);
    this->get_parameter("front_angle_start", front_angle_start_);
    this->get_parameter("front_angle_end", front_angle_end_);

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&CollisionChecker::laserCallback, this, std::placeholders::_1));

    collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("/collision_warning", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CollisionChecker::checkCollision, this));

    RCLCPP_INFO(this->get_logger(), "Collision checker started");
}

void CollisionChecker::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
}

void CollisionChecker::checkCollision() {
    if (!latest_scan_) return;

    bool collision = false;
    const auto& ranges = latest_scan_->ranges;
    int n = ranges.size();

    for (int i = 0; i < n; ++i) {
        float angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
        if (angle >= front_angle_start_ && angle <= front_angle_end_) {
            if (!std::isinf(ranges[i]) && !std::isnan(ranges[i]) && ranges[i] < min_safe_distance_) {
                collision = true;
                break;
            }
        }
    }

    std_msgs::msg::Bool msg;
    msg.data = collision;
    collision_pub_->publish(msg);
}

}  // namespace safety
}  // namespace guardian_stride

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<guardian_stride::safety::CollisionChecker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}