#include "lane.hpp"
#include "mask.hpp"

/**
 * Preprocess for lane model: resize to 224x224, BGR->RGB, normalize per channel,
 * and return CHW float buffer (R, G, B). Chosen stats match model training.
 */
std::vector<float> preprocess_frame(const cv::Mat& frame) {
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(224, 224));
    resized.convertTo(resized, CV_32FC3, 1.0 / 255);
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    std::vector<cv::Mat> channels(3);
    cv::split(resized, channels);
    channels[0] = (channels[0] - 0.485f) / 0.229f; // R
    channels[1] = (channels[1] - 0.456f) / 0.224f; // G
    channels[2] = (channels[2] - 0.406f) / 0.225f; // B
    cv::merge(channels,  resized);

    std::vector<float> inputData;
    for (int i = 0; i < 3; ++i) {
        inputData.insert(inputData.end(), (float*)channels[i].datastart, (float*)channels[i].dataend);
    }
    return inputData;
}

/**
 * Postprocess model output (float[C,H,W]) to overlay lanes and compute pose terms.
 * Assumptions:
 * - selected_channel indexes the lane of interest per model semantics.
 * - Threshold/morphology stabilize the mask; ROI focuses on near-field evidence.
 * - Fallbacks set offset/psi to 0 when evidence is insufficient or geometry degenerates.
 */
cv::Mat postprocess(float* ll_output, cv::Mat& original_frame, std::vector<cv::Point>& medianPoints, LineIntersect& intersect) {
    const int height_mask = 224;
    const int width_mask = 224;
    const int width_win = 640;
    const int height_win = 360;

    // Select lane channel as defined by the model (e.g., 0 = center lane)
    int selected_channel = 0;
    cv::Mat ll_mask(height_mask, width_mask, CV_32FC1, ll_output + selected_channel * height_mask * width_mask);

    // Threshold chosen to reject weak activations while keeping lane signal
    cv::Mat ll_bin;
    cv::threshold(ll_mask, ll_bin, 0.1, 255, cv::THRESH_BINARY);
    ll_bin.convertTo(ll_bin, CV_8UC1);

    // Light morphology to close small gaps without over-smoothing
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(ll_bin, ll_bin, cv::MORPH_CLOSE, kernel);
    cv::dilate(ll_bin, ll_bin, kernel);

    // Scale to frame and restrict to near-field ROI for stability
    cv::Mat ll_resized;
    cv::resize(ll_bin, ll_resized, original_frame.size(), 0, 0, cv::INTER_NEAREST);

    int roi_start_y = static_cast<int>(0.50 * original_frame.rows);
    int roi_end_y = static_cast<int>(0.95 * original_frame.rows);
    ll_resized(cv::Rect(0, 0, original_frame.cols, roi_start_y)) = 0;
    ll_resized(cv::Rect(0, roi_end_y, original_frame.cols, original_frame.rows - roi_end_y)) = 0;

    // Extract lane geometry (median points, fitted lines, and intersections)
    MaskProcessor processor;
    LineCoef left_coeffs, right_coeffs;
    cv::Mat mask_output;
    processor.processMask(ll_resized, mask_output, medianPoints, left_coeffs, right_coeffs, intersect);

    // Overlay lane visualization onto original frame if available
    cv::Mat result_frame = original_frame.clone();
    if (!mask_output.empty()) {
        cv::Mat resized_mask_output;
        cv::resize(mask_output, resized_mask_output, original_frame.size(), 0, 0, cv::INTER_NEAREST);

        for (int y = 0; y < resized_mask_output.rows; ++y) {
            for (int x = 0; x < resized_mask_output.cols; ++x) {
                cv::Vec3b pix = resized_mask_output.at<cv::Vec3b>(y, x);
                if (pix != cv::Vec3b(0, 0, 0)) {
                    result_frame.at<cv::Vec3b>(y, x) = pix;
                }
            }
        }
    }

    if (medianPoints.size() >= 5) {
        intersect.xlt = (left_coeffs.m * (height_win / 2) + left_coeffs.b);
        intersect.xlb = (left_coeffs.m * height_win + left_coeffs.b);
        intersect.xrt = (right_coeffs.m * (height_win / 2) + right_coeffs.b);
        intersect.xrb = (right_coeffs.m * height_win + right_coeffs.b);

        float xmt = (intersect.xrt + intersect.xlt) / 2;
        float xmb = (intersect.xrb + intersect.xlb) / 2;

        cv::Point xmtop(xmt, height_win / 2);
        cv::Point xmbottom(xmb, height_win);
        cv::line(result_frame, xmtop, xmbottom, cv::Scalar(255, 255, 255), 2); 

        float P2_x_img_frame = (Asy * height_win + Bsy) * ( xmb -(width_win / 2) );
        float P1_x_img_frame = (Asy * height_win / 2 + Bsy) * (xmt - (width_win / 2) );
        float deltaX_car_frame = P1_y_img_frame - P2_y_img_frame;
        float deltaY_car_frame = P1_x_img_frame - P2_x_img_frame;

        if (std::abs(deltaX_car_frame) > 1e-8) {
            intersect.slope = deltaY_car_frame / deltaX_car_frame;
            intersect.offset = P2_x_img_frame - intersect.slope * P1_y_img_frame;

            intersect.psi = std::atan(intersect.slope);
        } else {
            intersect.offset = 0.0f;
            intersect.psi = 0.0f;
            std::cout << "Warning: deltaX_car_frame is zero, default values, set to intersect." << std::endl;
        }
    } else {
        intersect.offset = 0.0f;
        intersect.psi = 0.0f;
        std::cout << "Warning: medianPoints is empty, default values set to intersect." << std::endl;
    }

    return result_frame;
}


