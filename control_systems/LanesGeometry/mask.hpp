#ifndef MASK_HPP
#define MASK_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "lane.hpp"

class MaskProcessor {
public:
    MaskProcessor();
    ~MaskProcessor();
    void processMask(const cv::Mat& ll_mask, cv::Mat& output, std::vector<cv::Point>& medianPoints, LineCoef& left_coeffs, LineCoef& right_coeffs, LineIntersect& intersect);
    std::vector<cv::Point> linearRegression(const std::vector<cv::Point>& points, int y_top, int bottom_y, int width, LineCoef& coeffs);
    void processEdges(const cv::Mat& mask_bin, std::vector<cv::Point>& left_edge, std::vector<cv::Point>& right_edge);
    int verifyLanes(std::vector<cv::Point>& left_edge_points, std::vector<cv::Point>& right_edge_points, int& bottom_left, int& bottom_right, int& top_left, int& top_right);
    LineIntersect findIntersect(const LineCoef& left_coeffs, const LineCoef& right_coeffs, int height, int width);
    
private:
    int firstWhite(const cv::Mat& row);
    int lastWhite(const cv::Mat& row);

    std::vector<cv::Point> left_edge_points, right_edge_points;
    std::vector<cv::Point> left_line_points, right_line_points;
};

#endif