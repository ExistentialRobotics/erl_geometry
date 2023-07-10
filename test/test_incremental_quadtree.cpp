#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "erl_common/eigen.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_geometry/incremental_quadtree.hpp"
#include "erl_geometry/node_container_multi_types.hpp"

cv::Mat quadtree_image;
Eigen::Vector<double, 360> angles = Eigen::Vector<double, 360>::LinSpaced(360, -M_PI, M_PI);
Eigen::Matrix<double, 2, 360> ray_directions;
Eigen::Matrix<double, 2, 360> ray_directions_inv;
auto grid_map_info =
    std::make_shared<erl::common::GridMapInfo2D>(Eigen::Vector2d(0, 0), Eigen::Vector2d(7, 7), Eigen::Vector2d(0.01, 0.01), Eigen::Vector2i(10, 10));
auto quadtree_setting = std::make_shared<erl::geometry::IncrementalQuadtree::Setting>();
auto node_container_setting = std::make_shared<erl::geometry::NodeContainerMultiTypes::Setting>();
std::shared_ptr<erl::geometry::IncrementalQuadtree> quadtree = nullptr;
double hit_distance_threshold = 0.02;
const char *window_name = "quadtree ray tracing";
double total_time = 0;
double ray_tracing_cnt = 0;

void
MouseCallback(int event, int x, int y, int flags, void *userdata) {
    (void) flags;
    (void) userdata;

    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    } else if (event == cv::EVENT_MOUSEMOVE) {
        auto ray_origin = grid_map_info->GridToMeterForPoints(Eigen::Vector2i(x, grid_map_info->Shape(1) - y));
        std::vector<double> ray_travel_distances;
        std::vector<std::shared_ptr<erl::geometry::Node>> hit_nodes;
        auto t_start = std::chrono::high_resolution_clock::now();
        quadtree->RayTracing(ray_origin.replicate(1, 360), ray_directions, hit_distance_threshold, 0, ray_travel_distances, hit_nodes);
        auto t_end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        total_time += time;
        ray_tracing_cnt += 1;
        std::cout << "Ray tracing time: " << time << " ms, average time: " << total_time / ray_tracing_cnt << " ms" << std::endl;

        cv::Mat new_image = quadtree_image.clone();
        for (int i = 0; i < 360; ++i) {
            if (ray_travel_distances[i] < std::numeric_limits<double>::max()) {
                Eigen::Vector2d hit_point = ray_origin + ray_directions.col(i) * ray_travel_distances[i];
                Eigen::Vector2i hit_point_grid = grid_map_info->MeterToPixelForPoints(hit_point);
                cv::line(new_image, cv::Point2i(x, y), cv::Point2i(hit_point_grid[0], hit_point_grid[1]), cv::Scalar(255, 0, 0), 1);
            }
        }

        cv::imshow(window_name, new_image);
    }
}

int
main() {
    ray_directions.row(0) = angles.array().cos();
    ray_directions.row(1) = angles.array().sin();
    ray_directions_inv.array() = ray_directions.array().inverse();

    quadtree_setting->min_half_area_size = 0.01;
    // quadtree_setting->nodes_in_leaves_only = true;
    node_container_setting->num_node_types = 2;
    node_container_setting->node_type_capacity = {1, 1};
    node_container_setting->node_type_min_squared_distance = {1.e-4, 1.e-4};
    quadtree = erl::geometry::IncrementalQuadtree::Create(quadtree_setting, erl::geometry::Aabb2D(Eigen::Vector2d(3, 3), 0.5), [&]() {
        return erl::geometry::NodeContainerMultiTypes::Create(node_container_setting);
    });

    double radius = 1;
    Eigen::Vector2d circle_center(3.5, 3);
    Eigen::Matrix<double, 2, 360> circle_vertices;
    circle_vertices.row(0) = radius * angles.array().cos() + circle_center.x();
    circle_vertices.row(1) = radius * angles.array().sin() + circle_center.y();

    Eigen::Vector2d rect_min(1, 1);
    Eigen::Vector2d rect_max(5, 5);
    double perimeter = 2 * (rect_max.x() - rect_min.x() + rect_max.y() - rect_min.y());
    Eigen::Matrix2Xd rect_vertices(2, int(perimeter) * 100);
    int w = int(rect_max.x() - rect_min.x()) * 100;
    int h = int(rect_max.y() - rect_min.y()) * 100;
    rect_vertices.block(0, 0, 1, w) = Eigen::VectorXd::LinSpaced(w, rect_min.x(), rect_max.x()).transpose();
    rect_vertices.block(1, 0, 1, w) = Eigen::VectorXd::Constant(w, rect_min.y()).transpose();
    rect_vertices.block(0, w, 1, h) = Eigen::VectorXd::Constant(h, rect_max.x()).transpose();
    rect_vertices.block(1, w, 1, h) = Eigen::VectorXd::LinSpaced(h, rect_min.y(), rect_max.y()).transpose();
    rect_vertices.block(0, w + h, 1, w) = Eigen::VectorXd::LinSpaced(w, rect_max.x(), rect_min.x()).transpose();
    rect_vertices.block(1, w + h, 1, w) = Eigen::VectorXd::Constant(w, rect_max.y()).transpose();
    rect_vertices.block(0, w + h + w, 1, h) = Eigen::VectorXd::Constant(h, rect_min.x()).transpose();
    rect_vertices.block(1, w + h + w, 1, h) = Eigen::VectorXd::LinSpaced(h, rect_max.y(), rect_min.y()).transpose();

    std::vector<std::vector<cv::Point2i>> contours(2);
    auto &circle_contour = contours[0];
    for (int i = 0; i < circle_vertices.cols(); ++i) {
        circle_contour.emplace_back(grid_map_info->MeterToGridForValue(circle_vertices(0, i), 0), grid_map_info->MeterToGridForValue(circle_vertices(1, i), 1));
    }

    auto &rect_contour = contours[1];
    for (int i = 0; i < rect_vertices.cols(); ++i) {
        rect_contour.emplace_back(grid_map_info->MeterToGridForValue(rect_vertices(0, i), 0), grid_map_info->MeterToGridForValue(rect_vertices(1, i), 1));
    }

    // img = cv::Mat(grid_map_info->Shape(1), grid_map_info->Shape(0), CV_8UC3, cv::Scalar(255, 255, 255));
    // cv::drawContours(img, contours, 0, cv::Scalar(0, 0, 255), 1);
    // cv::drawContours(img, contours, 1, cv::Scalar(0, 255, 0), 1);
    // cv::imshow("img", img);

    int num_inserted = 0;
    for (int i = 0; i < circle_vertices.cols(); ++i) {
        auto node = std::make_shared<erl::geometry::Node>(0, circle_vertices.col(i));
        std::shared_ptr<erl::geometry::IncrementalQuadtree> cluster;
        std::shared_ptr<erl::geometry::IncrementalQuadtree> new_root;
        cluster = quadtree->Insert(node, new_root);
        if (cluster) { num_inserted++; }
        if (new_root) {
            ERL_ASSERT(quadtree->GetRoot() == new_root);
            quadtree = new_root;
        }
    }
    ERL_INFO("num_inserted: %d / %ld\n", num_inserted, circle_vertices.cols());

    num_inserted = 0;
    std::shared_ptr<erl::geometry::IncrementalQuadtree> cluster;
    std::shared_ptr<erl::geometry::IncrementalQuadtree> new_root;
    for (int i = 0; i < rect_vertices.cols(); ++i) {
        auto node = std::make_shared<erl::geometry::Node>(1, rect_vertices.col(i));
        cluster = quadtree->Insert(node, new_root);
        if (cluster) { num_inserted++; }
        if (new_root) { quadtree = new_root; }
    }
    ERL_INFO("num_inserted: %d / %ld\n", num_inserted, rect_vertices.cols());

    quadtree_image = quadtree->Plot(grid_map_info, {0, 1}, {{0, cv::Scalar(0, 0, 255)}, {1, cv::Scalar(0, 255, 0)}}, {{0, 1}, {1, 1}});
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::setMouseCallback(window_name, MouseCallback, nullptr);
    cv::imshow(window_name, quadtree_image);

    // cv::waitKey(1);
    // std::ofstream ofs("quadtree.txt");
    // quadtree->Print(ofs);
    // ofs.close();
    // Eigen::Vector2d ray_origin(3.5, 3);
    // double ray_travel_distance = std::numeric_limits<double>::infinity();
    // std::shared_ptr<erl::geometry::Node> hit_node = nullptr;
    // quadtree->RayTracing(ray_origin, Eigen::Vector2d(1, 0), hit_distance_threshold, ray_travel_distance, hit_node);
    // std::cout << "hit_node position: " << hit_node->position.transpose() << ", ray_travel_distance: " << ray_travel_distance << std::endl;

    cv::waitKey(0);

    return 0;
}
