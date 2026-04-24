#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/opencv.hpp>

class CostmapBridge : public rclcpp::Node {
public:
    CostmapBridge() : Node("costmap_bridge") {
        this->declare_parameter("semantic_map_topic", "/semantic_map");
        this->declare_parameter("traversability_topic", "/traversability");

        semantic_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            this->get_parameter("semantic_map_topic").as_string(), 10,
            std::bind(&CostmapBridge::semanticCallback, this, std::placeholders::_1));

        traversability_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            this->get_parameter("traversability_topic").as_string(), 10,
            std::bind(&CostmapBridge::traversabilityCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Costmap bridge started");
    }

private:
    void semanticCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.empty()) return;

        int height = static_cast<int>(msg->layout.dim[0].size);
        int width = static_cast<int>(msg->layout.dim[1].size);

        cv::Mat semantic_map(height, width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()));

        cv::Mat costmap(height, width, CV_8UC1);
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                uint8_t class_id = semantic_map.at<uint8_t>(i, j);
                uint8_t cost = 0;

                switch (class_id) {
                    case 0: cost = 0; break;
                    case 1: cost = 50; break;
                    case 2: cost = 254; break;
                    case 3: cost = 254; break;
                    case 4: cost = 254; break;
                    case 5: cost = 128; break;
                    default: cost = 255; break;
                }

                costmap.at<uint8_t>(i, j) = cost;
            }
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Published costmap %dx%d", width, height);
    }

    void traversabilityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr /*msg*/) {
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr semantic_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr traversability_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
