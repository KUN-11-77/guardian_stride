#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <cstring>
#include <chrono>

class OccupancyBridge : public rclcpp::Node {
public:
    OccupancyBridge() : Node("occupancy_bridge"), tf_buffer_(this->get_clock()) {
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("robot_frame", "base_link");
        this->declare_parameter("grid_resolution", 0.05);
        this->declare_parameter("grid_width", 100);
        this->declare_parameter("grid_height", 100);
        this->declare_parameter("obstacle_threshold", 0.4);

        map_frame_ = this->get_parameter("map_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        resolution_ = this->get_parameter("grid_resolution").as_double();
        grid_width_ = this->get_parameter("grid_width").as_int();
        grid_height_ = this->get_parameter("grid_height").as_int();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();

        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/vins_fusion/point_cloud",
            10,
            std::bind(&OccupancyBridge::cloudCallback, this, std::placeholders::_1)
        );

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&OccupancyBridge::timerCallback, this)
        );

        initGrid();
        RCLCPP_INFO(this->get_logger(), "Occupancy bridge started");
    }

private:
    void initGrid() {
        grid_.header.frame_id = map_frame_;
        grid_.info.resolution = resolution_;
        grid_.info.width = grid_width_;
        grid_.info.height = grid_height_;
        grid_.info.origin.position.x = -grid_width_ * resolution_ / 2.0;
        grid_.info.origin.position.y = -grid_height_ * resolution_ / 2.0;
        grid_.data.resize(grid_width_ * grid_height_, -1);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(
                map_frame_, robot_frame_, tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TF lookup failed: %s", ex.what());
            return;
        }
        processCloud(msg, transform);
    }

    void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud,
                      const geometry_msgs::msg::TransformStamped& transform) {
        std::fill(grid_.data.begin(), grid_.data.end(), -1);

        const float ox = transform.transform.translation.x;
        const float oy = transform.transform.translation.y;

        int num_points = cloud->width * cloud->height;
        const uint8_t* data = cloud->data.data();

        int step_x = cloud->point_step;
        int offset_x = cloud->fields[0].offset;
        int offset_y = cloud->fields[1].offset;
        int offset_z = cloud->fields[2].offset;

        int inflation_radius = static_cast<int>(obstacle_threshold_ / resolution_);

        for (int i = 0; i < num_points; i++) {
            float px = *reinterpret_cast<const float*>(data + i * step_x + offset_x);
            float py = *reinterpret_cast<const float*>(data + i * step_x + offset_y);
            float pz = *reinterpret_cast<const float*>(data + i * step_x + offset_z);

            float local_x = px - ox;
            float local_y = py - oy;

            int gx = static_cast<int>((local_x - grid_.info.origin.position.x) / resolution_);
            int gy = static_cast<int>((local_y - grid_.info.origin.position.y) / resolution_);

            for (int dx = -inflation_radius; dx <= inflation_radius; dx++) {
                for (int dy = -inflation_radius; dy <= inflation_radius; dy++) {
                    int nx = gx + dx;
                    int ny = gy + dy;
                    if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_) {
                        int idx = ny * grid_width_ + nx;
                        if (grid_.data[idx] < 100) {
                            grid_.data[idx] = 100;
                        }
                    }
                }
            }
        }
    }

    void timerCallback() {
        grid_.header.stamp = this->get_clock()->now();
        grid_pub_->publish(grid_);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::OccupancyGrid grid_;
    std::string map_frame_;
    std::string robot_frame_;
    double resolution_;
    int grid_width_;
    int grid_height_;
    double obstacle_threshold_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
