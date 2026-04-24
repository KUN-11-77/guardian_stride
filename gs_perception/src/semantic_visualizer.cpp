#include "semantic_visualizer.hpp"

#include <opencv2/opencv.hpp>

static const cv::Vec3b SEMANTIC_COLORS[] = {
    {110, 201,   0},   // TRAVERSABLE_SIDEWALK  #00C96E
    {  0, 215, 255},   // TACTILE_PAVING        #FFD700
    {  0, 140, 255},   // STAIRS                #FF8C00
    { 48,  48, 224},   // STATIC_OBSTACLE       #E03030
    {204,   0, 204},   // DYNAMIC_OBSTACLE      #CC00CC
    { 64,  64,  64},   // BACKGROUND
};

SemanticVisualizer::SemanticVisualizer(const std::string& window_name)
    : window_name_(window_name), dynamic_obstacle_blink_(0) {
    canvas_ = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

void SemanticVisualizer::render(
    const cv::Mat& bgr_frame,
    const cv::Mat& semantic_mask,
    float fps,
    float nearest_obstacle_m,
    float tactile_coverage_pct,
    cv::Point2f path_center_normalized
) {
    canvas_.setTo(cv::Scalar(40, 40, 40));

    cv::Mat left_panel = canvas_(cv::Rect(0, 0, 640, 720));
    cv::Mat right_panel = canvas_(cv::Rect(640, 0, 640, 720));

    if (!bgr_frame.empty()) {
        cv::Mat resized;
        cv::resize(bgr_frame, resized, cv::Size(640, 480));
        resized.copyTo(left_panel(cv::Rect(0, 120, 640, 480)));
    }

    drawSemanticOverlay(right_panel, semantic_mask);
    drawHUD(right_panel, fps, nearest_obstacle_m, tactile_coverage_pct);
    drawPathArrow(right_panel, path_center_normalized);
    drawDynamicBlink(right_panel, semantic_mask);

    cv::putText(canvas_, "Guardian Stride - Perception",
                cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(255, 255, 255), 2);

    cv::imshow(window_name_, canvas_);
    cv::waitKey(1);
}

void SemanticVisualizer::drawSemanticOverlay(cv::Mat& right_panel, const cv::Mat& semantic_mask) {
    cv::Mat overlay = right_panel(cv::Rect(0, 120, 640, 480)).clone();

    cv::Mat resized_mask;
    cv::resize(semantic_mask, resized_mask, cv::Size(640, 480));

    for (int i = 0; i < resized_mask.rows; i++) {
        for (int j = 0; j < resized_mask.cols; j++) {
            uint8_t class_id = resized_mask.at<uint8_t>(i, j);
            if (class_id < 6) {
                overlay.at<cv::Vec3b>(i, j) = SEMANTIC_COLORS[class_id];
            }
        }
    }

    cv::addWeighted(overlay, 0.55, right_panel(cv::Rect(0, 120, 640, 480)), 0.45, 0,
                    right_panel(cv::Rect(0, 120, 640, 480)));
}

void SemanticVisualizer::drawHUD(cv::Mat& right_panel, float fps, float obstacle_m, float tactile_pct) {
    int y = 30;
    cv::putText(right_panel, cv::format("FPS: %.1f", fps),
                cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);

    y += 25;
    std::string obstacle_text = cv::format("Obstacle: %.2fm", obstacle_m);
    cv::Scalar obstacle_color = (obstacle_m < 2.0) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 255);
    cv::putText(right_panel, obstacle_text,
                cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, obstacle_color, 1);

    y += 25;
    cv::putText(right_panel, cv::format("Tactile: %.1f%%", tactile_pct),
                cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1);
}

void SemanticVisualizer::drawPathArrow(cv::Mat& right_panel, cv::Point2f center_norm) {
    int cx = static_cast<int>(center_norm.x * 640);
    int cy = static_cast<int>(center_norm.y * 480) + 120;

    cv::arrowedLine(right_panel, cv::Point(320, 400), cv::Point(cx, cy),
                    cv::Scalar(0, 255, 255), 3, 8, 0, 0.2);
}

void SemanticVisualizer::drawDynamicBlink(cv::Mat& right_panel, const cv::Mat& semantic_mask) {
    dynamic_obstacle_blink_ = (dynamic_obstacle_blink_ + 1) % 20;
    if (dynamic_obstacle_blink_ < 10) {
        bool has_dynamic = false;
        for (int i = 0; i < semantic_mask.rows && !has_dynamic; i++) {
            for (int j = 0; j < semantic_mask.cols && !has_dynamic; j++) {
                if (semantic_mask.at<uint8_t>(i, j) == 4) {
                    has_dynamic = true;
                }
            }
        }
        if (has_dynamic) {
            cv::rectangle(right_panel, cv::Rect(0, 120, 640, 480),
                         cv::Scalar(204, 0, 204), 3);
        }
    }
}
