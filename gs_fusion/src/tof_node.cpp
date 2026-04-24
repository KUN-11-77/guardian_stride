#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <gs_msgs/msg/gait_state.hpp>

#include <chrono>
#include <thread>
#include <array>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#ifndef NO_GPIOD
#include <gpiod.h>
#endif

using namespace std::chrono_literals;

constexpr int TOF_COUNT = 4;
constexpr const char* I2C_DEVICE = "/dev/i2c-0";
constexpr uint8_t TOF_ADDR = 0x29;

class ToFNode : public rclcpp::Node {
public:
    ToFNode() : Node("tof_node"), i2c_fd_(-1) {
        this->declare_parameter("i2c_bus", 0);
        this->declare_parameter("frame_id", "tof_link");
        this->declare_parameter("sampling_rate_hz", 50);

        int i2c_bus = this->get_parameter("i2c_bus").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        int sampling_rate = this->get_parameter("sampling_rate_hz").as_int();

        RCLCPP_INFO(this->get_logger(), "Initializing ToF array on I2C bus %d", i2c_bus);

        char i2c_dev[32];
        snprintf(i2c_dev, sizeof(i2c_dev), "/dev/i2c-%d", i2c_bus);
        i2c_fd_ = open(i2c_dev, O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device %s", i2c_dev);
        }

        for (int i = 0; i < TOF_COUNT; i++) {
            char topic_name[32];
            snprintf(topic_name, sizeof(topic_name), "/tof/sensor_%d", i);
            tof_pubs_[i] = this->create_publisher<sensor_msgs::msg::Range>(topic_name, 10);
        }

        tof_distances_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/tof/distances", 10);

        int interval_ms = 1000 / sampling_rate;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            std::bind(&ToFNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "ToF node started at %d Hz", sampling_rate);
    }

    ~ToFNode() {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
        }
    }

private:
    void timerCallback() {
        auto now = this->get_clock()->now();

        std::array<float, TOF_COUNT> distances;
        for (int i = 0; i < TOF_COUNT; i++) {
            distances[i] = readToFSensor(i);
        }

        for (int i = 0; i < TOF_COUNT; i++) {
            sensor_msgs::msg::Range msg;
            msg.header.stamp = now;
            msg.header.frame_id = frame_id_ + "_" + std::to_string(i);
            msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
            msg.field_of_view = 0.26;
            msg.min_range = 0.02;
            msg.max_range = 2.0;
            msg.range = distances[i];
            tof_pubs_[i]->publish(msg);
        }
    }

    float readToFSensor(int sensor_index) {
        if (i2c_fd_ < 0) return 0.0f;

        if (ioctl(i2c_fd_, I2C_SLAVE, TOF_ADDR + sensor_index) < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Failed to set I2C address for sensor %d", sensor_index);
            return 0.0f;
        }

        uint8_t data[2] = {0x00, 0x04};
        if (write(i2c_fd_, data, 2) != 2) {
            return 0.0f;
        }

        std::this_thread::sleep_for(10ms);

        uint8_t dist_buf[2];
        if (read(i2c_fd_, dist_buf, 2) == 2) {
            uint16_t dist = (dist_buf[0] << 8) | dist_buf[1];
            return dist / 1000.0f;
        }

        return 0.0f;
    }

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_pubs_[TOF_COUNT];
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_distances_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
