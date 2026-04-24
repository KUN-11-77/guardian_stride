#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>

#include "semantic_visualizer.hpp"
#include "traversability_calculator.hpp"

class SegFormerNode : public rclcpp::Node {
public:
    SegFormerNode() : Node("segformer_node"), last_fps_time_(this->get_clock()->now()) {
        this->declare_parameter("model_xml_path", "/home/user/ros2_ws/src/guardian_stride/gs_perception/models/segformer_b0_int8.xml");
        this->declare_parameter("inference_device", "GPU");
        this->declare_parameter("input_image_topic", "/camera/color/image_raw");
        this->declare_parameter("output_semantic_map_topic", "/semantic_map");
        this->declare_parameter("output_traversability_topic", "/traversability");
        this->declare_parameter("output_obstacle_dist_topic", "/nearest_obstacle_m");
        this->declare_parameter("output_path_center_topic", "/tactile_path_center");
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("visualization_window_name", "Guardian Stride - Perception");
        this->declare_parameter("overlay_alpha", 0.55);
        this->declare_parameter("obstacle_warn_dist_m", 2.0);
        this->declare_parameter("obstacle_stop_dist_m", 0.5);

        model_xml_path_ = this->get_parameter("model_xml_path").as_string();
        inference_device_ = this->get_parameter("inference_device").as_string();
        enable_viz_ = this->get_parameter("enable_visualization").as_bool();

        RCLCPP_INFO(this->get_logger(), "Loading SegFormer model from: %s", model_xml_path_.c_str());

        loadModel(model_xml_path_);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("input_image_topic").as_string(),
            10,
            std::bind(&SegFormerNode::imageCallback, this, std::placeholders::_1)
        );

        semantic_map_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            this->get_parameter("output_semantic_map_topic").as_string(), 10);
        traversability_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            this->get_parameter("output_traversability_topic").as_string(), 10);
        obstacle_dist_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            this->get_parameter("output_obstacle_dist_topic").as_string(), 10);
        path_center_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            this->get_parameter("output_path_center_topic").as_string(), 10);

        if (enable_viz_) {
            visualizer_ = std::make_unique<SemanticVisualizer>(
                this->get_parameter("visualization_window_name").as_string());
        }
        trav_calc_ = std::make_unique<TraversabilityCalculator>();

        RCLCPP_INFO(this->get_logger(), "SegFormer node started, device: %s", inference_device_.c_str());
    }

private:
    void loadModel(const std::string& model_path) {
        ov::Core core;
        compiled_model_ = core.compile_model(model_path, inference_device_);
        infer_request_ = compiled_model_.create_infer_request();
        RCLCPP_INFO(this->get_logger(), "Model loaded on %s", inference_device_.c_str());
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto start = this->get_clock()->now();

        cv::Mat bgr_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (bgr_frame.empty()) return;

        cv::Mat semantic_mask = runInference(bgr_frame);

        trav_calc_->compute(semantic_mask, traversability_mat_);

        publishResults(semantic_mask, msg->header.stamp);

        if (enable_viz_ && visualizer_) {
            visualizer_->render(
                bgr_frame,
                semantic_mask,
                fps_,
                trav_calc_->getNearestObstacleDistance(),
                trav_calc_->getTactileCoverage(),
                trav_calc_->getPathCenter()
            );
        }

        updateFPS(start);
    }

    cv::Mat runInference(const cv::Mat& input_bgr) {
        cv::Mat resized;
        cv::resize(input_bgr, resized, cv::Size(640, 480));

        ov::Tensor input_tensor = ov::Tensor(compiled_model_.input().get_element_type(),
                                              compiled_model_.input().get_shape(), resized.ptr());
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.infer();

        auto output_tensor = infer_request_.get_output_tensor(0);
        ov::Shape output_shape = output_tensor.get_shape();

        cv::Mat output_mat(static_cast<int>(output_shape[2]), static_cast<int>(output_shape[3]), CV_32FC1,
                           output_tensor.data<float>());

        cv::Mat semantic_mask(static_cast<int>(output_shape[2]), static_cast<int>(output_shape[3]), CV_8UC1);
        for (int i = 0; i < semantic_mask.rows; i++) {
            for (int j = 0; j < semantic_mask.cols; j++) {
                semantic_mask.at<uint8_t>(i, j) = static_cast<uint8_t>(
                    output_mat.at<float>(i, j));
            }
        }

        return semantic_mask;
    }

    void publishResults(const cv::Mat& semantic_mask, rclcpp::Time stamp) {
        {
            std_msgs::msg::UInt8MultiArray msg;
            std_msgs::msg::MultiArrayDimension dim;
            dim.label = "height";
            dim.size = static_cast<uint32_t>(semantic_mask.rows);
            dim.stride = static_cast<uint32_t>(semantic_mask.rows * semantic_mask.cols);
            msg.layout.dim.push_back(dim);
            dim.label = "width";
            dim.size = static_cast<uint32_t>(semantic_mask.cols);
            dim.stride = static_cast<uint32_t>(semantic_mask.cols);
            msg.layout.dim.push_back(dim);
            msg.data.assign(semantic_mask.datastart, semantic_mask.dataend);
            semantic_map_pub_->publish(msg);
        }

        {
            std_msgs::msg::Float32MultiArray msg;
            std_msgs::msg::MultiArrayDimension dim;
            dim.label = "height";
            dim.size = static_cast<uint32_t>(traversability_mat_.rows);
            dim.stride = static_cast<uint32_t>(traversability_mat_.rows * traversability_mat_.cols);
            msg.layout.dim.push_back(dim);
            dim.label = "width";
            dim.size = static_cast<uint32_t>(traversability_mat_.cols);
            dim.stride = static_cast<uint32_t>(traversability_mat_.cols);
            msg.layout.dim.push_back(dim);
            msg.data.assign(traversability_mat_.begin<float>(), traversability_mat_.end<float>());
            traversability_pub_->publish(msg);
        }

        {
            std_msgs::msg::Float32 msg;
            msg.data = trav_calc_->getNearestObstacleDistance();
            obstacle_dist_pub_->publish(msg);
        }

        {
            geometry_msgs::msg::Point msg;
            msg.x = trav_calc_->getPathCenter().x;
            msg.y = trav_calc_->getPathCenter().y;
            path_center_pub_->publish(msg);
        }
    }

    void updateFPS(rclcpp::Time start) {
        frame_count_++;
        auto now = this->get_clock()->now();
        double elapsed = (now - last_fps_time_).seconds();
        if (elapsed >= 1.0) {
            fps_ = frame_count_ / elapsed;
            frame_count_ = 0;
            last_fps_time_ = now;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr semantic_map_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr traversability_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr path_center_pub_;

    std::unique_ptr<SemanticVisualizer> visualizer_;
    std::unique_ptr<TraversabilityCalculator> trav_calc_;

    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;

    cv::Mat traversability_mat_;
    std::string model_xml_path_;
    std::string inference_device_;
    bool enable_viz_;

    int frame_count_{0};
    double fps_{0.0};
    rclcpp::Time last_fps_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SegFormerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
