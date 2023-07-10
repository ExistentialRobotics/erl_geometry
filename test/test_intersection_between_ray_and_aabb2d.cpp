#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "erl_common/assert.hpp"
#include "erl_common/angle_utils.hpp"
#include "erl_geometry/aabb.hpp"
#include "erl_geometry/utils.hpp"

erl::geometry::Aabb2D aabb(Eigen::Vector2d(200, 300), 100);
cv::Mat image(500, 400, CV_8UC3, cv::Scalar(255, 255, 255));
Eigen::Vector<double, 360> angles = Eigen::Vector<double, 360>::LinSpaced(360, -M_PI, M_PI * 179 / 180);
Eigen::Matrix<double, 2, 360> ray_directions;
Eigen::Matrix<double, 2, 360> ray_directions_inv;

void
MouseCallback(int event, int x, int y, int flags, void *userdata) {
    (void) flags;
    (void) userdata;

    if (event == cv::EVENT_RBUTTONDOWN) {
        std::cout << "x: " << x << ", y: " << y << std::endl;
        std::cout << "inside: " << aabb.contains(Eigen::Vector2d(x, y)) << std::endl;
    } else if (event == cv::EVENT_MOUSEMOVE) {

        Eigen::Vector<double, 360> ray_travel_distances;
        ray_travel_distances.setConstant(std::numeric_limits<double>::infinity());
        Eigen::Vector<bool, 360> intersected_flags;
        intersected_flags.setConstant(false);
        Eigen::Vector2d ray_origin(x, y);

        // erl::geometry::ComputeIntersectionBetweenRayAndAabb2D(
        //     Eigen::Vector2d(50, 200),
        //     Eigen::Vector2d(1, std::numeric_limits<double>::max()),
        //     aabb.min(),
        //     aabb.max(),
        //     ray_travel_distances[0],
        //     intersected_flags[0]);

        cv::Mat new_image = image.clone();
        cv::Point2i ray_start(x, y);
        for (int i = 0; i < 360; ++i) {
            erl::geometry::ComputeIntersectionBetweenRayAndAabb2D(
                ray_origin,
                ray_directions_inv.col(i),
                aabb.min(),
                aabb.max(),
                ray_travel_distances[i],
                intersected_flags[i]);
            if (intersected_flags[i] && ray_travel_distances[i] >= 0) {
                // ERL_ASSERTM(ray_travel_distances[i] >= 0, "ray_travel_distances[i] = %f", ray_travel_distances[i]);
                cv::Point2i ray_end(x + int(ray_travel_distances[i] * ray_directions(0, i)), y + int(ray_travel_distances[i] * ray_directions(1, i)));
                cv::line(new_image, ray_start, ray_end, cv::Scalar(0, 0, 255), 1);
            }
        }

        std::cout << "x: " << x << ", y: " << y << std::endl
                  << "angles: " << Eigen::VectorXd(angles.unaryExpr([](double x) { return erl::common::RadianToDegree(x); })) << std::endl
                  << "ray_travel_distances: " << ray_travel_distances.transpose() << std::endl
                  << "intersected_flags: " << intersected_flags.transpose() << std::endl;
        cv::imshow("image", new_image);
    }
}

int
main() {
    ray_directions.row(0) = angles.array().cos();
    ray_directions.row(1) = angles.array().sin();
    ray_directions_inv.array() = ray_directions.array().inverse();

    cv::Point2i rect_min(int(aabb.min()[0]), int(aabb.min()[1]));
    cv::Point2i rect_max(int(aabb.max()[0]), int(aabb.max()[1]));
    cv::rectangle(image, rect_min, rect_max, cv::Scalar(0, 0, 0), 1);

    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::setMouseCallback("image", MouseCallback, nullptr);

    cv::imshow("image", image);
    cv::waitKey(0);

    return 0;
}
