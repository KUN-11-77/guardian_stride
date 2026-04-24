#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

class TraversabilityCalculator {
public:
    TraversabilityCalculator();

    void compute(const cv::Mat& semantic_mask, cv::Mat& traversability);

    float getNearestObstacleDistance() const { return nearest_obstacle_dist_; }
    cv::Point2f getPathCenter() const { return path_center_; }
    float getTactileCoverage() const { return tactile_coverage_pct_; }

private:
    float nearest_obstacle_dist_;
    cv::Point2f path_center_;
    float tactile_coverage_pct_;
};
