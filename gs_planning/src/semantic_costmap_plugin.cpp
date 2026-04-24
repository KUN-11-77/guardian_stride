// gs_planning/src/semantic_costmap_plugin.cpp
// Nav2 代价图语义插件：将 SegFormer 语义分割结果映射为 Nav2 代价地图层

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gs_msgs/msg/semantic_map.hpp>
#include <gs_msgs/msg/traversability.hpp>

namespace guardian_stride {

class SemanticCostmapPlugin : public nav2_costmap_2d::Layer {
public:
  SemanticCostmapPlugin()
    : last_update_time_(0)
    , resolution_(0.05)
    , costmap_width_(0)
    , costmap_height_(0)
    , obstacle_cost_(254)
    , traversable_cost_(0)
    , unknown_cost_(255)
    , inflation_radius_(0.4)
    , enabled_(false) {}

  virtual void onInitialize() override {
    auto node = rclcpp::Node::make_shared("semantic_costmap_plugin_node");
    rclcpp::QoS qos(1);
    qos.reliable();

    semantic_sub_ = node->create_subscription<gs_msgs::msg::SemanticMap>(
        "/semantic_map", qos,
        [this](const gs_msgs::msg::SemanticMap::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_semantic_map_ = msg;
        });

    traversability_sub_ = node->create_subscription<gs_msgs::msg::Traversability>(
        "/traversability", qos,
        [this](const gs_msgs::msg::Traversability::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_traversability_ = msg;
        });

    node->declare_parameter("obstacle_cost", rclcpp::ParameterValue(254));
    node->declare_parameter("traversable_cost", rclcpp::ParameterValue(0));
    node->declare_parameter("unknown_cost", rclcpp::ParameterValue(255));
    node->declare_parameter("inflation_radius", rclcpp::ParameterValue(0.4));

    node->get_parameter("obstacle_cost", obstacle_cost_);
    node->get_parameter("traversable_cost", traversable_cost_);
    node->get_parameter("unknown_cost", unknown_cost_);
    node->get_parameter("inflation_radius", inflation_radius_);

    enabled_ = true;
  }

  virtual void updateBounds(
      double robot_x, double robot_y, double robot_yaw,
      double* min_x, double* min_y,
      double* max_x, double* max_y) override {
    if (!enabled_) return;

    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_semantic_map_) {
      *min_x = std::min(*min_x, 0.0);
      *min_y = std::min(*min_y, 0.0);
      double w = static_cast<double>(latest_semantic_map_->width) * resolution_;
      double h = static_cast<double>(latest_semantic_map_->height) * resolution_;
      *max_x = std::max(*max_x, w);
      *max_y = std::max(*max_y, h);
    }
  }

  virtual void updateCosts(
      nav2_costmap_2d::Costmap2D& master_grid,
      int min_i, int min_j,
      int max_i, int max_j) override {
    if (!enabled_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    if (latest_traversability_) {
      auto& trav = *latest_traversability_;
      uint32_t w = trav.width;
      uint32_t h = trav.height;

      for (uint32_t j = 0; j < h && (min_j + static_cast<int>(j)) < master_grid.getSizeInCellsY(); ++j) {
        for (uint32_t i = 0; i < w && (min_i + static_cast<int>(i)) < master_grid.getSizeInCellsX(); ++i) {
          uint32_t idx = j * w + i;
          if (idx >= trav.data.size()) continue;

          float trav_value = trav.data[idx];
          unsigned char cost;

          if (trav_value < 0.3f) {
            cost = static_cast<unsigned char>(obstacle_cost_);
          } else if (trav_value > 0.7f) {
            cost = static_cast<unsigned char>(traversable_cost_);
          } else {
            cost = static_cast<unsigned char>(127);
          }

          master_grid.setCost(min_i + static_cast<int>(i), min_j + static_cast<int>(j), cost);
        }
      }
    }

    if (latest_semantic_map_) {
      auto& sem_map = *latest_semantic_map_;
      uint32_t w = sem_map.width;
      uint32_t h = sem_map.height;

      for (uint32_t j = 0; j < h && (min_j + static_cast<int>(j)) < master_grid.getSizeInCellsY(); ++j) {
        for (uint32_t i = 0; i < w && (min_i + static_cast<int>(i)) < master_grid.getSizeInCellsX(); ++i) {
          uint32_t idx = j * w + i;
          if (idx >= sem_map.data.size()) continue;

          uint8_t semantic_class = sem_map.data[idx];

          if (semantic_class == 3 || semantic_class == 4) {
            master_grid.setCost(min_i + static_cast<int>(i), min_j + static_cast<int>(j),
                                static_cast<unsigned char>(obstacle_cost_));
          }
        }
      }
    }
  }

  virtual void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_semantic_map_.reset();
    latest_traversability_.reset();
  }

  virtual bool isClearable() override {
    return true;
  }

private:
  rclcpp::Subscription<gs_msgs::msg::SemanticMap>::SharedPtr semantic_sub_;
  rclcpp::Subscription<gs_msgs::msg::Traversability>::SharedPtr traversability_sub_;

  std::mutex mutex_;
  gs_msgs::msg::SemanticMap::SharedPtr latest_semantic_map_;
  gs_msgs::msg::Traversability::SharedPtr latest_traversability_;

  rclcpp::Time last_update_time_;
  double resolution_;
  int costmap_width_;
  int costmap_height_;

  int obstacle_cost_;
  int traversable_cost_;
  int unknown_cost_;
  double inflation_radius_;
  bool enabled_;
};

}  // namespace guardian_stride

PLUGINLIB_EXPORT_CLASS(guardian_stride::SemanticCostmapPlugin, nav2_costmap_2d::Layer)
