#pragma once

#include <string>
#include <opencv2/opencv.hpp>

class SemanticVisualizer {
public:
    explicit SemanticVisualizer(const std::string& window_name = "Guardian Stride");

    void render(
        const cv::Mat& bgr_frame,
        const cv::Mat& semantic_mask,
        float fps,
        float nearest_obstacle_m,
        float tactile_coverage_pct,
        cv::Point2f path_center_normalized
    );

private:
    std::string window_name_;
    cv::Mat canvas_;
    int dynamic_obstacle_blink_;

    void drawSemanticOverlay(cv::Mat& right_panel, const cv::Mat& semantic_mask);
    void drawHUD(cv::Mat& right_panel, float fps, float obstacle_m, float tactile_pct);
    void drawPathArrow(cv::Mat& right_panel, cv::Point2f center_norm);
    void drawDynamicBlink(cv::Mat& right_panel, const cv::Mat& semantic_mask);
};
