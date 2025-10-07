#include "mask.hpp"
#include "lane.hpp"

MaskProcessor::MaskProcessor() {}
MaskProcessor::~MaskProcessor() {}

std::vector<cv::Point> MaskProcessor::linearRegression(const std::vector<cv::Point>& points, int y_top, int bottom_y, int width, LineCoef& coeffs) {
    coeffs = {0.0, 0.0, false};
    std::vector<cv::Point> edge;
    if (points.size() < 3) return edge;
    double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0;
    int n = points.size();
    int y_step = 2;

    for (const auto& p : points) {
        double x = p.y;
        double y = p.x;
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumXX += x * x;
    }

    double denominator = (n * sumXX - sumX * sumX);
    if (abs(denominator) < 1e-6) return edge;

    coeffs.m = (n * sumXY - sumX * sumY) / denominator;
    coeffs.b = (sumY * sumXX - sumX * sumXY) / denominator;
    coeffs.valid = true;

    for (int y = y_top; y <= bottom_y; y += y_step) {
        int point_x = static_cast<int>(coeffs.m * y + coeffs.b);
        if (point_x >= 0 && point_x < width) edge.push_back(cv::Point(point_x, y));
    }
    return edge;
}

void MaskProcessor::processEdges(const cv::Mat& mask_bin, std::vector<cv::Point>& left_edge, std::vector<cv::Point>& right_edge) {
    int height = mask_bin.rows, width = mask_bin.cols;
    int center_x = width / 2, margin = 5, range = 20;

    left_edge.clear();
    right_edge.clear();

    int base_y = static_cast<int>(height * 0.95) - 1;

    int left_x = -1, right_x = -1;
    int left_contig = -1, right_contig = -1;
    int left_count = 0, right_count = 0;
    int left_start_limit = static_cast<int>(height * 0.60);
    int right_start_limit = static_cast<int>(height * 0.60);
    bool reset_left = false, reset_right = false;
    bool valid_left = true, valid_right = true;

    for (int y = base_y; y >= height / 2; --y) {
        const cv::Mat row = mask_bin.row(y);
        int new_right_x = -1, new_left_x = -1;

        if (left_x == -1 && y > left_start_limit && valid_left == true) {
            for (int x = center_x; x >= margin; --x) {
                if (row.at<uchar>(x) == 255 && (x >= center_x - 10 && x <= center_x)) {
                    valid_left = false;
                    break;
                }
                if (row.at<uchar>(x) == 255 && x < center_x - (center_x / 4)) {
                    left_x = x; 
                    left_contig++;
                    left_count++;
                    break;
                }
            }
        } else if (left_x != -1 && left_contig != -1 && left_contig <= 10 && valid_left == true) {
            for (int x = std::min(left_x + range, center_x + 100); x >= std::max(margin, left_x - range); --x) {
                if (row.at<uchar>(x) == 255) {
                    new_left_x = x; 
                    left_count++;
                    break; 
                }
            }
            if (new_left_x != -1) left_x = new_left_x;
            else left_contig++;
        }
        if (left_contig > 10 && left_count < 50) {
            left_x = -1;
            left_contig = -1;
            left_count = 0;
            reset_left = true;
        }
        
        if (right_x == -1 && y > right_start_limit && valid_right == true) {
            for (int x = center_x; x < width - margin; ++x) {
                 if (row.at<uchar>(x) == 255 && (x <= center_x + 10 && x >= center_x)) {
                    valid_right = false;
                    break;
                }
                if (row.at<uchar>(x) == 255 && x > center_x + (center_x / 4)) {
                    right_x = x; 
                    right_contig++;
                    right_count++;
                    break;
                }
            }
        } else if (right_x != -1 && right_contig != -1 && right_contig <= 10 && valid_right == true) {
            for (int x = std::max(center_x, right_x - range); x <= std::min(width - margin, right_x + range); ++x) {
                if (row.at<uchar>(x) == 255) {
                    new_right_x = x;
                    right_count++;
                    break;
                }
            }
            if (new_right_x != -1) right_x = new_right_x;
            else right_contig++;
        }
        if (right_contig > 10 && right_count < 50) {
            right_x = -1;
            right_contig = -1;
            right_count = 0;
            reset_right = true;
        }

        if (left_x != -1 && left_contig <= 10 && left_contig != -1) left_edge.emplace(left_edge.begin(), left_x, y);
        if (reset_left == true) left_edge.clear();

        if (right_x != -1 && right_contig <= 10 && right_contig != -1) right_edge.emplace(right_edge.begin(), right_x, y);
        if (reset_right == true) right_edge.clear();
    }
}

int MaskProcessor::verifyLanes(std::vector<cv::Point>& left_edge_points, std::vector<cv::Point>& right_edge_points, 
                              int& bottom_left, int& bottom_right, int& top_left, int& top_right) {
    size_t min_size_line = 50;
    int value = 0;

    if (left_edge_points.size() > min_size_line) {
        bottom_left = left_edge_points.back().y;
    } else if (left_edge_points.size() < min_size_line && right_edge_points.size() > min_size_line) {
        value = -1;
    }

    if (right_edge_points.size() > min_size_line) {
        bottom_right = right_edge_points.back().y;
    } else if (right_edge_points.size() < min_size_line && left_edge_points.size() > min_size_line) {
        value = -2;
    }

    if (left_edge_points.size() < min_size_line && right_edge_points.size() < min_size_line) {
        std::cerr << "[Warning Break!!!!!!!!!!!!] Edges Lost" << std::endl;
        value = -3;
    }
    return value;
}

std::ostream& operator<<(std::ostream& os, const LineCoef& coef) {
    return os << "m=" << coef.m << ", b=" << coef.b << ", valid=" << (coef.valid ? "true" : "false");
}

std::ostream& operator<<(std::ostream& os, const std::vector<cv::Point>& points) {
    os << "[";
    for (size_t i = 0; i < points.size(); ++i) {
        os << "(" << points[i].x << "," << points[i].y << ")";
        if (i < points.size() - 1) os << ",";
    }
    return os << "]";
}

void MaskProcessor::processMask(const cv::Mat& ll_mask, cv::Mat& output, std::vector<cv::Point>& medianPoints, 
                               LineCoef& left_coeffs, LineCoef& right_coeffs, LineIntersect& intersect) {
    cv::Mat mask_bin = ll_mask.clone();
    cv::threshold(ll_mask, mask_bin, 127, 255, cv::THRESH_BINARY);

    int height = 360, width = 640;
    int top_y = height / 2, bottom_y = height * 0.95;
    int bottom_left = bottom_y, bottom_right = bottom_y;
    int top_left = top_y, top_right = top_y;

    processEdges(mask_bin, left_edge_points, right_edge_points);
    int verify = verifyLanes(left_edge_points, right_edge_points, bottom_left, bottom_right, top_left, top_right);

    if (verify == 0) {
        left_line_points = linearRegression(left_edge_points, top_left, bottom_left, width, left_coeffs);
        right_line_points = linearRegression(right_edge_points, top_right, bottom_right, width, right_coeffs);
        lane_memory.pre_left_edge = left_line_points;
        lane_memory.pre_left_coeffs = left_coeffs;
        lane_memory.pre_right_edge = right_line_points;
        lane_memory.pre_right_coeffs = right_coeffs;
    }
    if (verify == -1) {
        left_edge_points = lane_memory.pre_left_edge;
        left_coeffs = lane_memory.pre_left_coeffs;
        right_line_points = linearRegression(right_edge_points, top_right, bottom_right, width, right_coeffs);
        lane_memory.pre_right_edge = right_line_points;
        lane_memory.pre_right_coeffs = right_coeffs;
    }
    if (verify == -2) {
        right_edge_points = lane_memory.pre_right_edge;
        right_coeffs = lane_memory.pre_right_coeffs;
        left_line_points = linearRegression(left_edge_points, top_left, bottom_left, width, left_coeffs);
        lane_memory.pre_left_edge = left_line_points;
        lane_memory.pre_left_coeffs = left_coeffs;
    }
    if (verify == -3){
        right_edge_points = lane_memory.pre_right_edge;
        right_coeffs = lane_memory.pre_right_coeffs;
        left_edge_points = lane_memory.pre_left_edge;
        left_coeffs = lane_memory.pre_left_coeffs;
    }

    cv::cvtColor(mask_bin, output, cv::COLOR_GRAY2BGR);
    if (!left_line_points.empty())
        cv::line(output, left_line_points.front(), left_line_points.back(), cv::Scalar(0, 0, 255), 2);
    if (!right_line_points.empty())
        cv::line(output, right_line_points.front(), right_line_points.back(), cv::Scalar(255, 0, 0), 2);

    medianPoints.clear();
    for (int y = top_y; y < height; y++) {
        float left_x = (left_coeffs.m * y + left_coeffs.b);
        float right_x = (right_coeffs.m * y + right_coeffs.b);
        float median_x = (left_x + right_x) / 2;
        if (median_x >= 0 && median_x < width) {
            medianPoints.emplace_back(median_x, y);
        }
    }

    if (!medianPoints.empty()) {
        cv::line(output, medianPoints.front(), medianPoints.back(), cv::Scalar(200, 200, 200), 2);
    }
}