// gs_perception/src/costmap_bridge.cpp
// 语义图 → Nav2 代价图桥接
// 订阅 SegFormer 语义分割结果，发布为 Nav2 OccupancyGrid

#include <rclcpp/rclcpp.hpp>
#include <gs_msgs/msg/semantic_map.hpp>
#include <gs_msgs/msg/traversability.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <mutex>

class CostmapBridge : public rclcpp::Node {
public:
  CostmapBridge()
    : Node("costmap_bridge"),
      resolution_(0.05), width_(100), height_(100) {
    declare_parameter("resolution", 0.05);
    declare_parameter("width", 100);
    declare_parameter("height", 100);
    declare_parameter("frame_id", "map");

    get_parameter("resolution", resolution_);
    get_parameter("width", width_);
    get_parameter("height", height_);
    frame_id_ = get_parameter("frame_id").as_string();

    semantic_sub_ = create_subscription<gs_msgs::msg::SemanticMap>(
        "/semantic_map", 10,
        [this](const gs_msgs::msg::SemanticMap::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_semantic_ = msg;
        });

    traversability_sub_ = create_subscription<gs_msgs::msg::Traversability>(
        "/traversability", 10,
        [this](const gs_msgs::msg::Traversability::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_traversability_ = msg;
        });

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(200),
        [this]() { publishCostmap(); });

    RCLCPP_INFO(get_logger(), "Costmap bridge 已启动 (5Hz)");
  }

private:
  void publishCostmap() {
    std::lock_guard<std::mutex> lock(mutex_);

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = frame_id_;
    grid.info.resolution = resolution_;
    grid.info.width = width_;
    grid.info.height = height_;
    grid.info.origin.position.x = -width_ * resolution_ / 2.0;
    grid.info.origin.position.y = -height_ * resolution_ / 2.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(width_ * height_, -1);

    if (latest_traversability_) {
      auto& trav = *latest_traversability_;
      for (uint32_t j = 0; j < std::min(trav.height, height_); j++) {
        for (uint32_t i = 0; i < std::min(trav.width, width_); i++) {
          uint32_t idx = j * trav.width + i;
          if (idx >= trav.data.size()) continue;
          float val = trav.data[idx];
          int cell = static_cast<int>(grid.info.width * (j * height_ / trav.height) +
                                      (i * width_ / trav.width));
          if (cell < static_cast<int>(grid.data.size())) {
            grid.data[cell] = (val < 0.3f) ? 100 : (val > 0.7f ? 0 : 50);
          }
        }
      }
    } else if (latest_semantic_) {
      auto& sem = *latest_semantic_;
      for (uint32_t j = 0; j < std::min(sem.height, height_); j++) {
        for (uint32_t i = 0; i < std::min(sem.width, width_); i++) {
          uint32_t idx = j * sem.width + i;
          if (idx >= sem.data.size()) continue;
          uint8_t cls = sem.data[idx];
          int cell = static_cast<int>(grid.info.width * (j * height_ / sem.height) +
                                      (i * width_ / sem.width));
          if (cell < static_cast<int>(grid.data.size())) {
            grid.data[cell] = (cls == 0 || cls == 1) ? 0 :
                              (cls == 5) ? 50 : 100;
          }
        }
      }
    }

    costmap_pub_->publish(grid);
  }

  rclcpp::Subscription<gs_msgs::msg::SemanticMap>::SharedPtr semantic_sub_;
  rclcpp::Subscription<gs_msgs::msg::Traversability>::SharedPtr traversability_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;

  gs_msgs::msg::SemanticMap::SharedPtr latest_semantic_;
  gs_msgs::msg::Traversability::SharedPtr latest_traversability_;

  double resolution_;
  uint32_t width_;
  uint32_t height_;
  std::string frame_id_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapBridge>());
  rclcpp::shutdown();
  return 0;
}
