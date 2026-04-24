// gs_planning/src/guidance_torque_node.cpp
// 引导力矩计算节点：根据 Nav2 路径和语义信息计算引导力矩
// 频率：100Hz

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <gs_msgs/msg/guidance_torque.hpp>
#include <gs_msgs/msg/safety_cmd.hpp>
#include <gs_msgs/msg/traversability.hpp>
#include <geometry_msgs/msg/point.hpp>

class GuidanceTorqueNode : public rclcpp::Node {
public:
  GuidanceTorqueNode()
    : Node("guidance_torque_node") {
    declare_parameter("kp_guide", 2.5f);
    declare_parameter("kd_guide", 0.8f);
    declare_parameter("max_torque_nm", 3.0f);
    declare_parameter("path_topic", "/plan");
    declare_parameter("state_topic", "/state");
    declare_parameter("safety_cmd_topic", "/safety_cmd");

    get_parameter("kp_guide", kp_guide_);
    get_parameter("kd_guide", kd_guide_);
    get_parameter("max_torque_nm", max_torque_nm_);

    rclcpp::QoS qos(1);
    qos.reliable();

    state_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        state_topic_, qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          current_velocity_ = *msg;
        });

    safety_cmd_sub_ = create_subscription<gs_msgs::msg::SafetyCmd>(
        safety_cmd_topic_, qos,
        [this](const gs_msgs::msg::SafetyCmd::SharedPtr msg) {
          safety_state_ = msg->state;
          safety_scale_ = getSafetyScale(msg->state);
        });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
          current_path_ = *msg;
        });

    tactile_sub_ = create_subscription<geometry_msgs::msg::Point>(
        "/tactile_path_center", qos,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
          tactile_center_ = *msg;
          has_tactile_update_ = true;
        });

    traversability_sub_ = create_subscription<gs_msgs::msg::Traversability>(
        "/traversability", qos,
        [this](const gs_msgs::msg::Traversability::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(trav_mutex_);
          latest_traversability_ = msg;
        });

    torque_pub_ = create_publisher<gs_msgs::msg::GuidanceTorque>(
        "/guidance_torque", qos);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() { computeAndPublish(); });
  }

private:
  float getSafetyScale(const std::string& state) {
    if (state == "NORMAL") return 1.0f;
    if (state == "SLOW_DOWN") return 0.5f;
    return 0.0f;
  }

  void computeAndPublish() {
    if (current_path_.poses.empty()) return;

    auto nearest_point = findNearestPathPoint();
    if (!nearest_point.has_value()) return;

    float p_plan_x = nearest_point->pose.position.x;
    float p_plan_y = nearest_point->pose.position.y;

    float p_curr_x = 0.0f;
    float p_curr_y = 0.0f;

    float dx = p_plan_x - p_curr_x;
    float dy = p_plan_y - p_curr_y;
    float dist = std::sqrt(dx * dx + dy * dy);

    float target_angle = std::atan2(dy, dx);

    float vel_x = current_velocity_.linear.x;
    float vel_y = current_velocity_.linear.y;
    float current_speed = std::sqrt(vel_x * vel_x + vel_y * vel_y);

    float torque_scale = safety_scale_ * computeTraversabilityScale();

    float left_torque = kp_guide_ * dist * torque_scale - kd_guide_ * current_speed;
    float right_torque = kp_guide_ * dist * torque_scale - kd_guide_ * current_speed;

    left_torque = std::max(-max_torque_nm_, std::min(max_torque_nm_, left_torque));
    right_torque = std::max(-max_torque_nm_, std::min(max_torque_nm_, right_torque));

    gs_msgs::msg::GuidanceTorque torque_msg;
    torque_msg.header.stamp = now();
    torque_msg.left_nm = left_torque;
    torque_msg.right_nm = right_torque;
    torque_msg.mode = safety_state_ == "NORMAL" ? "guide" : "brake";
    torque_msg.waypoint_remaining = current_path_.poses.size();

    torque_pub_->publish(torque_msg);
  }

  std::optional<geometry_msgs::msg::PoseStamped> findNearestPathPoint() {
    if (current_path_.poses.empty()) return std::nullopt;

    float min_dist = std::numeric_limits<float>::max();
    geometry_msgs::msg::PoseStamped nearest;

    for (const auto& pose : current_path_.poses) {
      float dx = pose.pose.position.x;
      float dy = pose.pose.position.y;
      float dist = std::sqrt(dx * dx + dy * dy);

      if (dist < min_dist) {
        min_dist = dist;
        nearest = pose;
      }
    }

    return nearest;
  }

  float computeTraversabilityScale() {
    std::lock_guard<std::mutex> lock(trav_mutex_);
    if (!latest_traversability_) return 1.0f;

    float avg_trav = 0.0f;
    const auto& data = latest_traversability_->data;
    if (data.empty()) return 1.0f;

    for (float v : data) {
      avg_trav += v;
    }
    avg_trav /= data.size();

    return avg_trav;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr state_sub_;
  rclcpp::Subscription<gs_msgs::msg::SafetyCmd>::SharedPtr safety_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr tactile_sub_;
  rclcpp::Subscription<gs_msgs::msg::Traversability>::SharedPtr traversability_sub_;

  rclcpp::Publisher<gs_msgs::msg::GuidanceTorque>::SharedPtr torque_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist current_velocity_;
  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::Point tactile_center_;
  gs_msgs::msg::Traversability::SharedPtr latest_traversability_;

  std::mutex trav_mutex_;

  std::string safety_state_ = "NORMAL";
  float safety_scale_ = 1.0f;
  bool has_tactile_update_ = false;

  float kp_guide_;
  float kd_guide_;
  float max_torque_nm_;

  std::string state_topic_ = "/state";
  std::string path_topic_ = "/plan";
  std::string safety_cmd_topic_ = "/safety_cmd";
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidanceTorqueNode>());
  rclcpp::shutdown();
  return 0;
}
