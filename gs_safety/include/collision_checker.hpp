#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

namespace guardian_stride {
namespace safety {

class CollisionChecker : public rclcpp::Node {
public:
    CollisionChecker();

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void checkCollision();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double min_safe_distance_;
    double front_angle_start_;
    double front_angle_end_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

}  // namespace safety
}  // namespace guardian_stride

#endif  // COLLISION_CHECKER_HPP