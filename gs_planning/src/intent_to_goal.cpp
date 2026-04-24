// gs_planning/src/intent_to_goal.cpp
// Intent → Nav2 Goal 桥接节点：解析语音指令为导航目标

#include <rclcpp/rclcpp.hpp>
#include <gs_msgs/msg/intent.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class IntentToGoalNode : public rclcpp::Node {
public:
  IntentToGoalNode()
    : Node("intent_to_goal_node") {
    declare_parameter("default_frame_id", "map");

    rclcpp::QoS qos(1);
    qos.reliable();

    intent_sub_ = create_subscription<gs_msgs::msg::Intent>(
        "/intent", qos,
        [this](const gs_msgs::msg::Intent::SharedPtr msg) {
          handleIntent(msg);
        });

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", qos);

    RCLCPP_INFO(get_logger(), "IntentToGoalNode 已启动");
  }

private:
  void handleIntent(const gs_msgs::msg::Intent::SharedPtr intent) {
    if (intent->type == "navigate" && !intent->target.empty()) {
      RCLCPP_INFO(get_logger(), "收到导航意图: %s", intent->target.c_str());
      publishGoal(intent->target);
    } else if (intent->type == "query_status") {
      RCLCPP_INFO(get_logger(), "查询状态请求");
    } else if (intent->type == "mode_switch") {
      RCLCPP_INFO(get_logger(), "模式切换请求");
    } else if (intent->type == "emergency") {
      RCLCPP_WARN(get_logger(), "紧急停止指令!");
    }
  }

  void publishGoal(const std::string& target) {
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = now();
    goal_pose.header.frame_id = "map";

    if (target == "入口" || target == "entrance") {
      goal_pose.pose.position.x = 0.0;
      goal_pose.pose.position.y = 0.0;
    } else if (target == "电梯" || target == "elevator") {
      goal_pose.pose.position.x = 10.0;
      goal_pose.pose.position.y = 5.0;
    } else if (target == "楼梯" || target == "stairs") {
      goal_pose.pose.position.x = 15.0;
      goal_pose.pose.position.y = 0.0;
    } else {
      goal_pose.pose.position.x = 5.0;
      goal_pose.pose.position.y = 5.0;
    }

    goal_pose.pose.orientation.w = 1.0;

    goal_pub_->publish(goal_pose);
    RCLCPP_INFO(get_logger(), "已发布目标: (%.1f, %.1f)",
                goal_pose.pose.position.x, goal_pose.pose.position.y);
  }

  rclcpp::Subscription<gs_msgs::msg::Intent>::SharedPtr intent_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntentToGoalNode>());
  rclcpp::shutdown();
  return 0;
}
