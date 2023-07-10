#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "erl_common/eigen.hpp"
#include "erl_common/grid_map_info.hpp"

auto grid_map_info =
    std::make_shared<erl::common::GridMapInfo2D>(Eigen::Vector2d(-3, -3), Eigen::Vector2d(3, 3), Eigen::Vector2d(0.01, 0.01), Eigen::Vector2i(10, 10));
double radius = 2;
Eigen::Vector2d p_min_angle_270;
Eigen::Vector2d p_max_angle_270;
Eigen::Vector2d p_min_angle_60;
Eigen::Vector2d p_max_angle_60;
cv::Mat img_270 = cv::Mat(grid_map_info->Shape(1), grid_map_info->Shape(0), CV_8UC3, cv::Scalar(255, 255, 255));
cv::Mat img_60 = cv::Mat(grid_map_info->Shape(1), grid_map_info->Shape(0), CV_8UC3, cv::Scalar(255, 255, 255));

void
MouseCallback270(int event, int x, int y, int flags, void *userdata) {
    (void)(flags);
    (void)(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) { std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl; }
    if (event == cv::EVENT_MOUSEMOVE) {
        Eigen::Vector2d p = grid_map_info->PixelToMeterForPoints(Eigen::Vector2i(x, y));
        double theta_2 = std::atan2(p[1], p[0]);
        std::cout << "x: " << x << ", y: " << y << ", theta_2: " << theta_2 * 180.0 / M_PI << std::endl;
        double theta_1 = std::atan2(p_min_angle_270[1] - p[1], p_min_angle_270[0] - p[0]);
        double theta_3 = std::atan2(p_max_angle_270[1] - p[1], p_max_angle_270[0] - p[0]);
        auto new_image = img_270.clone();
        if (theta_2 >= 0) {
            theta_2 -= M_PI;
            double theta_min = std::max(theta_1, theta_2);
            // double theta_max = theta_3;

            cv::line(
                new_image,
                cv::Point2i(x, y),
                cv::Point2i(
                    grid_map_info->MeterToGridForValue(theta_min == theta_1 ? p_min_angle_270[0] : 0, 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(theta_min == theta_1 ? p_min_angle_270[1] : 0, 1)),
                cv::Scalar(255, 0, 0),
                1);
            cv::line(
                new_image,
                cv::Point2i(x, y),
                cv::Point2i(
                    grid_map_info->MeterToGridForValue(p_max_angle_270[0], 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(p_max_angle_270[1], 1)),
                cv::Scalar(0, 255, 0),
                1);
        } else {
            theta_2 += M_PI;
            double theta_max = std::min(theta_3, theta_2);

            cv::line(
                new_image,
                cv::Point2i(x, y),
                cv::Point2i(
                    grid_map_info->MeterToGridForValue(p_min_angle_270[0], 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(p_min_angle_270[1], 1)),
                cv::Scalar(255, 0, 0),
                1);
            cv::line(
                new_image,
                cv::Point2i(x, y),
                cv::Point2i(
                    grid_map_info->MeterToGridForValue(theta_max == theta_3 ? p_max_angle_270[0] : 0, 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(theta_max == theta_3 ? p_max_angle_270[1] : 0, 1)),
                cv::Scalar(0, 255, 0),
                1);
        }
        cv::imshow("270", new_image);
    }
}

void
MouseCallback60(int event, int x, int y, int flags, void *userdata) {
    (void)(flags);
    (void)(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) { std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl; }
    if (event == cv::EVENT_MOUSEMOVE) {
        Eigen::Vector2d p = grid_map_info->PixelToMeterForPoints(Eigen::Vector2i(x, y));
        double theta_2 = std::atan2(p[1], p[0]);
        std::cout << "x: " << x << ", y: " << y << ", theta_2: " << theta_2 * 180.0 / M_PI << std::endl;
        double theta_1 = std::atan2(p_min_angle_60[1] - p[1], p_min_angle_60[0] - p[0]);
        double theta_3 = std::atan2(p_max_angle_60[1] - p[1], p_max_angle_60[0] - p[0]);
        auto new_image = img_60.clone();
        if (theta_2 >= 0) {
            theta_2 -= M_PI;
            double theta_min = std::max(theta_1, theta_2);
            // double theta_max = theta_3;

            cv::line(
                new_image,
                cv::Point2i(x, y),
                cv::Point2i(
                    grid_map_info->MeterToGridForValue(theta_min == theta_1 ? p_min_angle_60[0] : 0, 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(theta_min == theta_1 ? p_min_angle_60[1] : 0, 1)),
                cv::Scalar(255, 0, 0),
                1);
            cv::line(
                new_image,
                cv::Point2i(x, y),
                cv::Point2i(
                    grid_map_info->MeterToGridForValue(p_max_angle_60[0], 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(p_max_angle_60[1], 1)),
                cv::Scalar(0, 255, 0),
                1);
        } else {
            theta_2 += M_PI;
            double theta_max = std::min(theta_3, theta_2);

            cv::line(
                new_image,
                cv::Point2i(x, y),
                cv::Point2i(
                    grid_map_info->MeterToGridForValue(p_min_angle_60[0], 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(p_min_angle_60[1], 1)),
                cv::Scalar(255, 0, 0),
                1);
            cv::line(
                new_image,
                cv::Point2i(x, y),
                cv::Point2i(
                    grid_map_info->MeterToGridForValue(theta_max == theta_3 ? p_max_angle_60[0] : 0, 0),
                    grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(theta_max == theta_3 ? p_max_angle_60[1] : 0, 1)),
                cv::Scalar(0, 255, 0),
                1);
        }
        cv::imshow("60", new_image);
    }
}

int
main() {

    p_min_angle_270 << radius * std::cos(-M_PI * 3 / 4), radius * std::sin(-M_PI * 3 / 4);
    p_max_angle_270 << radius * std::cos(M_PI * 3 / 4), radius * std::sin(M_PI * 3 / 4);
    p_min_angle_60 << radius * std::cos(-M_PI / 6), radius * std::sin(-M_PI / 6);
    p_max_angle_60 << radius * std::cos(M_PI / 6), radius * std::sin(M_PI / 6);

    std::vector<std::vector<cv::Point>> contours(2);
    auto &contour_270 = contours[0];

    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(270, -M_PI * 3 / 4, M_PI * 3 / 4);
    for (auto &angle: angles) {
        contour_270.emplace_back(
            grid_map_info->MeterToGridForValue(radius * std::cos(angle), 0),
            grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(radius * std::sin(angle), 1));
    }
    contour_270.emplace_back(grid_map_info->MeterToGridForValue(0, 0), grid_map_info->MeterToGridForValue(0, 1));
    cv::drawContours(img_270, contours, 0, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
    cv::imshow("270", img_270);
    cv::setMouseCallback("270", MouseCallback270, nullptr);

    auto &contour_60 = contours[1];
    angles = Eigen::VectorXd::LinSpaced(60, -M_PI / 6, M_PI / 6);
    for (auto &angle: angles) {
        contour_60.emplace_back(
            grid_map_info->MeterToGridForValue(radius * std::cos(angle), 0),
            grid_map_info->Shape(1) - grid_map_info->MeterToGridForValue(radius * std::sin(angle), 1));
    }
    contour_60.emplace_back(grid_map_info->MeterToGridForValue(0, 0), grid_map_info->MeterToGridForValue(0, 1));
    cv::drawContours(img_60, contours, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
    cv::imshow("60", img_60);
    cv::setMouseCallback("60", MouseCallback60, nullptr);
    cv::waitKey(0);

    return 0;
}
