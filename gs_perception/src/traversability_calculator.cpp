#include "traversability_calculator.hpp"

#include <cmath>
#include <algorithm>

TraversabilityCalculator::TraversabilityCalculator()
    : nearest_obstacle_dist_(999.0f), tactile_coverage_pct_(0.0f) {
}

void TraversabilityCalculator::compute(const cv::Mat& semantic_mask, cv::Mat& traversability) {
    traversability = cv::Mat(semantic_mask.rows, semantic_mask.cols, CV_32FC1);

    int tactile_count = 0;
    int total_pixels = 0;
    float min_obstacle_y = semantic_mask.rows;

    for (int i = 0; i < semantic_mask.rows; i++) {
        for (int j = 0; j < semantic_mask.cols; j++) {
            uint8_t class_id = semantic_mask.at<uint8_t>(i, j);
            float value = 1.0f;

            switch (class_id) {
                case 0:  // TRAVERSABLE_SIDEWALK
                case 1:  // TACTILE_PAVING
                    value = 1.0f;
                    tactile_count++;
                    break;
                case 2:  // STAIRS
                case 3:  // STATIC_OBSTACLE
                case 4:  // DYNAMIC_OBSTACLE
                    value = 0.0f;
                    if (i < min_obstacle_y) min_obstacle_y = i;
                    break;
                case 5:  // BACKGROUND
                default:
                    value = 0.3f;
                    break;
            }

            traversability.at<float>(i, j) = value;
            total_pixels++;
        }
    }

    tactile_coverage_pct_ = (total_pixels > 0) ? (tactile_count * 100.0f / total_pixels) : 0.0f;

    float focal_length = 500.0f;
    float real_height = 1.2f;
    nearest_obstacle_dist_ = (min_obstacle_y < semantic_mask.rows) ?
        (real_height * focal_length / (semantic_mask.rows - min_obstacle_y + 1)) : 10.0f;

    int search_start = semantic_mask.cols / 4;
    int search_end = 3 * semantic_mask.cols / 4;
    int mid_row = semantic_mask.rows / 2;
    int path_center_x = -1;

    for (int j = search_start; j < search_end; j++) {
        if (semantic_mask.at<uint8_t>(mid_row, j) <= 1) {
            path_center_x = j;
            break;
        }
    }

    if (path_center_x >= 0) {
        path_center_ = cv::Point2f(
            static_cast<float>(path_center_x) / semantic_mask.cols,
            static_cast<float>(mid_row) / semantic_mask.rows
        );
    } else {
        path_center_ = cv::Point2f(0.5f, 0.5f);
    }
}
