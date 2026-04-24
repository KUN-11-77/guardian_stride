#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

#ifdef NO_MRAA
// Stub implementations when MRAA is not available
#include <chrono>
#include <cmath>
#include <iostream>

using i2c_context = void*;
inline i2c_context mraa_i2c_init(int bus) { (void)bus; return nullptr; }
inline void mraa_i2c_stop(i2c_context ctx) { (void)ctx; }
inline int mraa_i2c_address(i2c_context ctx, uint8_t addr) { (void)ctx; (void)addr; return 0; }
inline uint8_t mraa_i2c_read_byte(i2c_context ctx, uint8_t reg) { (void)ctx; (void)reg; return 0; }
inline int mraa_i2c_read_bytes_data(i2c_context ctx, uint8_t reg, uint8_t* data, int len) { (void)ctx; (void)reg; (void)data; (void)len; return -1; }
inline int mraa_i2c_write_byte(i2c_context ctx, uint8_t reg, uint8_t val) { (void)ctx; (void)reg; (void)val; return 0; }
#define MRAA_SUCCESS 0
#else
#include <mraa_i2c.h>
#include <mraa_uart.h>
#include <mraa/gpio.h>
#endif

using namespace std::chrono_literals;

constexpr uint8_t BNO055_I2C_ADDR = 0x28;
constexpr uint8_t BNO055_CHIP_ID_ADDR = 0x00;
constexpr uint8_t BNO055_ACCEL_DATA_X_LSB = 0x08;
constexpr uint8_t BNO055_GYRO_DATA_X_LSB = 0x14;
constexpr uint8_t BNO055_MAG_DATA_X_LSB = 0x0E;
constexpr uint8_t BNO055_QUATERNION_DATA_W_LSB = 0x20;
constexpr uint8_t BNO055_CALIB_STAT_ADDR = 0x35;
constexpr uint8_t BNO055_MODE_REG = 0x3D;
constexpr uint8_t BNO055_OPR_MODE = 0x3D;
constexpr uint8_t BNO055_PWR_MODE = 0x3E;

class IMUNode : public rclcpp::Node {
public:
    IMUNode() : Node("imu_node") {
        this->declare_parameter("i2c_bus", 1);
        this->declare_parameter("i2c_address", BNO055_I2C_ADDR);
        this->declare_parameter("frame_id", "imu_link");
        this->declare_parameter("sampling_rate_hz", 1000);

        int i2c_bus = this->get_parameter("i2c_bus").as_int();
        int i2c_addr = this->get_parameter("i2c_address").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        int sampling_rate = this->get_parameter("sampling_rate_hz").as_int();

        RCLCPP_INFO(this->get_logger(), "Initializing BNO055 on I2C bus %d, address 0x%x", i2c_bus, i2c_addr);

#ifdef NO_MRAA
        RCLCPP_WARN(this->get_logger(), "MRAA not available, running in simulation mode");
        i2c_context_ = nullptr;
#else
        i2c_context_ = mraa_i2c_init(i2c_bus);
        if (i2c_context_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize I2C");
            return;
        }
        mraa_i2c_address(i2c_context_, i2c_addr);

        uint8_t chip_id = mraa_i2c_read_byte(i2c_context_, BNO055_CHIP_ID_ADDR);
        if (chip_id != 0xA0) {
            RCLCPP_WARN(this->get_logger(), "BNO055 chip ID mismatch: 0x%02x (expected 0xA0)", chip_id);
        }
        mraa_i2c_write_byte(i2c_context_, BNO055_OPR_MODE, 0x0B);
        std::this_thread::sleep_for(100ms);
#endif

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 1000);

        int interval_ms = 1000 / sampling_rate;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            std::bind(&IMUNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "IMU node started at %d Hz", sampling_rate);
    }

    ~IMUNode() {
#ifndef NO_MRAA
        if (i2c_context_ != nullptr) {
            mraa_i2c_stop(i2c_context_);
        }
#endif
    }

private:
    void timerCallback() {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = frame_id_;

#ifdef NO_MRAA
        // Simulation mode - publish zeros
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 9.81;
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
#else
        uint8_t accel_buf[6];
        if (mraa_i2c_read_bytes_data(i2c_context_, BNO055_ACCEL_DATA_X_LSB, accel_buf, 6) == MRAA_SUCCESS) {
            int16_t accel_x = (int16_t)(accel_buf[0] | (accel_buf[1] << 8));
            int16_t accel_y = (int16_t)(accel_buf[2] | (accel_buf[3] << 8));
            int16_t accel_z = (int16_t)(accel_buf[4] | (accel_buf[5] << 8));
            imu_msg.linear_acceleration.x = accel_x * 0.001 * 9.81;
            imu_msg.linear_acceleration.y = accel_y * 0.001 * 9.81;
            imu_msg.linear_acceleration.z = accel_z * 0.001 * 9.81;
        }

        uint8_t gyro_buf[6];
        if (mraa_i2c_read_bytes_data(i2c_context_, BNO055_GYRO_DATA_X_LSB, gyro_buf, 6) == MRAA_SUCCESS) {
            int16_t gyro_x = (int16_t)(gyro_buf[0] | (gyro_buf[1] << 8));
            int16_t gyro_y = (int16_t)(gyro_buf[2] | (gyro_buf[3] << 8));
            int16_t gyro_z = (int16_t)(gyro_buf[4] | (gyro_buf[5] << 8));
            imu_msg.angular_velocity.x = gyro_x * M_PI / 180.0;
            imu_msg.angular_velocity.y = gyro_y * M_PI / 180.0;
            imu_msg.angular_velocity.z = gyro_z * M_PI / 180.0;
        }
#endif

        for (int i = 0; i < 9; i++) {
            imu_msg.linear_acceleration_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
        }
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;
        imu_msg.angular_velocity_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[4] = 0.01;
        imu_msg.angular_velocity_covariance[8] = 0.01;

        imu_pub_->publish(imu_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    i2c_context i2c_context_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
