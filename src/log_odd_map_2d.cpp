#include "erl_geometry/log_odd_map_2d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/logging.hpp"

#include <opencv2/imgproc.hpp>

#include <utility>

namespace erl::geometry {
    LogOddMap2D::LogOddMap2D(std::shared_ptr<Setting> setting, std::shared_ptr<common::GridMapInfo2D> grid_map_info)
        : m_setting_(std::move(setting)),
          m_grid_map_info_(std::move(grid_map_info)),
          m_log_map_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1), CV_64FC1, cv::Scalar{0}),  // ij-indexing, x to the bottom, y to the right
          m_possibility_map_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1), CV_64FC1, cv::Scalar{0.5}),
          m_occupancy_map_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1), CV_8UC1, cv::Scalar{CellType::kUnexplored}),
          m_kernel_(cv::getStructuringElement(m_setting_->use_cross_kernel ? cv::MORPH_CROSS : cv::MORPH_RECT, cv::Size{3, 3})),
          m_mask_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1)),
          m_cleaned_mask_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1)),
          m_num_unexplored_cells_(m_grid_map_info_->Shape(0) * m_grid_map_info_->Shape(1)) {}

    LogOddMap2D::LogOddMap2D(
        std::shared_ptr<Setting> setting,
        std::shared_ptr<common::GridMapInfo<2>> grid_map_info,
        const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices)
        : LogOddMap2D(std::move(setting), std::move(grid_map_info)) {
        m_shape_vertices_ = shape_metric_vertices;
    }

    void
    LogOddMap2D::Update(
        const Eigen::Ref<const Eigen::Vector2d> &position,
        const double theta,
        const Eigen::Ref<const Eigen::VectorXd> &angles_body,
        const Eigen::Ref<const Eigen::VectorXd> &ranges) {

        ERL_DEBUG_ASSERT(angles_body.size() == ranges.size(), "angles and ranges should be of the same shape.");
        ERL_DEBUG_ASSERT(!ranges.hasNaN(), "detect nan in ranges!");

        // generate mask of lidar scan
        constexpr bool clip_ranges = true;
        constexpr bool ray_mode = true;
        constexpr bool in_map_only = true;
        const auto mask = ComputeLidarFrameMask(position, theta, angles_body, ranges, clip_ranges, ray_mode, in_map_only, nullptr);
        if (mask->mask.rows == 0 || mask->mask.cols == 0) { return; }

        // compute parameters
        const double log_certainty = std::log(m_setting_->measurement_certainty);
        const double log_uncertainty = std::log(1.0 - m_setting_->measurement_certainty);
        const double log_odd_occupied = log_certainty - log_uncertainty;
        const double log_odd_free = log_uncertainty - log_certainty;

        // update log_odd_map, possibility_map, occupancy_map
        for (int row = 0; row < mask->mask.rows; ++row) {
            for (int col = 0; col < mask->mask.cols; ++col) {
                const auto &mask_value = mask->mask.at<uint8_t>(row, col);
                if (mask_value == kUnexplored) { continue; }

                const int x = mask->x_grid_min + row;
                const int y = mask->y_grid_min + col;
                auto &log_odd_value = m_log_map_.at<double>(x, y);
                auto &possibility_value = m_possibility_map_.at<double>(x, y);
                auto &occupancy_value = m_occupancy_map_.at<uint8_t>(x, y);
                auto &free_mask_value = m_mask_.free_mask.at<uint8_t>(x, y);
                auto &occupied_mask_value = m_mask_.occupied_mask.at<uint8_t>(x, y);
                auto &unexplored_mask_value = m_mask_.unexplored_mask.at<uint8_t>(x, y);

                if (mask_value == kOccupied) {
                    log_odd_value += log_odd_occupied;
                } else {
                    log_odd_value += log_odd_free;
                }
                if (log_odd_value < m_setting_->min_log_odd) {
                    log_odd_value = m_setting_->min_log_odd;
                } else if (log_odd_value > m_setting_->max_log_odd) {
                    log_odd_value = m_setting_->max_log_odd;
                }

                possibility_value = 1.0 / (1.0 + std::exp(-log_odd_value));

                if (possibility_value > m_setting_->threshold_occupied) {
                    if (occupancy_value == kFree) {  // kFree -> kOccupied
                        m_num_free_cells_--;
                        m_num_occupied_cells_++;
                        free_mask_value = 0;
                    } else if (occupancy_value == kUnexplored) {  // kUnexplored -> kOccupied
                        m_num_unexplored_cells_--;
                        m_num_occupied_cells_++;
                        unexplored_mask_value = 0;
                    }
                    occupancy_value = kOccupied;
                    occupied_mask_value = 1;
                } else if (possibility_value < m_setting_->threshold_free) {
                    if (occupancy_value == kOccupied) {  // kOccupied -> kFree
                        m_num_occupied_cells_--;
                        m_num_free_cells_++;
                        occupied_mask_value = 0;
                    } else if (occupancy_value == kUnexplored) {  // kUnexplored -> kFree
                        m_num_unexplored_cells_--;
                        m_num_free_cells_++;
                        unexplored_mask_value = 0;
                    }
                    occupancy_value = kFree;
                    free_mask_value = 1;
                }
            }
        }

        PostProcessMasks(position, theta);
    }

    void
    LogOddMap2D::LoadExternalPossibilityMap(
        const Eigen::Ref<const Eigen::Vector2d> &position,
        const double theta,
        const Eigen::Ref<const Eigen::MatrixXi> &possibility_map) {

        ERL_ASSERTM(
            possibility_map.rows() == m_grid_map_info_->Shape(0) && possibility_map.cols() == m_grid_map_info_->Shape(1),
            "External log odd map has wrong shape. Expected: ({}, {}), Actual: ({}, {})",
            m_grid_map_info_->Shape(0),
            m_grid_map_info_->Shape(1),
            possibility_map.rows(),
            possibility_map.cols());

        const auto n_rows = static_cast<int>(possibility_map.rows());
        const auto n_cols = static_cast<int>(possibility_map.cols());
        m_num_unexplored_cells_ = 0;
        m_num_occupied_cells_ = 0;
        m_num_free_cells_ = 0;

        for (int i = 0; i < n_rows; ++i) {
            for (int j = 0; j < n_cols; ++j) {
                auto &log_odd_value = m_log_map_.at<double>(i, j);
                auto &possibility_value = m_possibility_map_.at<double>(i, j);
                auto &occupancy_value = m_occupancy_map_.at<uint8_t>(i, j);
                auto &free_mask_value = m_mask_.free_mask.at<uint8_t>(i, j);
                auto &occupied_mask_value = m_mask_.occupied_mask.at<uint8_t>(i, j);
                auto &unexplored_mask_value = m_mask_.unexplored_mask.at<uint8_t>(i, j);

                if (possibility_map(i, j) == -1) {
                    log_odd_value = 0.;
                    possibility_value = 0.5;
                    occupancy_value = CellType::kUnexplored;
                    free_mask_value = 0;
                    occupied_mask_value = 0;
                    unexplored_mask_value = 1;
                    m_num_unexplored_cells_++;
                } else {
                    possibility_value = static_cast<double>(possibility_map(i, j)) / 100.;
                    log_odd_value = std::log(possibility_value / (1. - possibility_value));
                    if (possibility_value > m_setting_->threshold_occupied) {
                        occupancy_value = CellType::kOccupied;
                        free_mask_value = 0;
                        occupied_mask_value = 1;
                        unexplored_mask_value = 0;
                        m_num_occupied_cells_++;
                    } else if (possibility_value < m_setting_->threshold_free) {
                        occupancy_value = CellType::kFree;
                        free_mask_value = 1;
                        occupied_mask_value = 0;
                        unexplored_mask_value = 0;
                        m_num_free_cells_++;
                    } else {
                        occupancy_value = CellType::kUnexplored;
                        free_mask_value = 0;
                        occupied_mask_value = 0;
                        unexplored_mask_value = 1;
                        m_num_unexplored_cells_++;
                    }
                }
            }
        }

        PostProcessMasks(position, theta);
    }

    // void
    // LogOddMap2D::ComputeStatisticsOfLidarFrame(
    //     const Eigen::Ref<const Eigen::Vector2d> &position,
    //     double theta,
    //     const Eigen::Ref<const Eigen::VectorXd> &angles_body,
    //     const Eigen::Ref<const Eigen::VectorXd> &ranges,
    //     bool clip_ranges,
    //     int &num_occupied_cells,
    //     int &num_free_cells,
    //     int &num_unexplored_cells,
    //     int &num_out_of_map_cells) const {
    //
    //     const bool kRayMode = false;
    //     const bool kInMapOnly = false;
    //     ComputeStatisticsOfLidarFrameMask(
    //         ComputeLidarFrameMask(position, theta, angles_body, ranges, clip_ranges, kRayMode, kInMapOnly),
    //         num_occupied_cells,
    //         num_free_cells,
    //         num_unexplored_cells,
    //         num_out_of_map_cells);
    //
    //     // num_occupied_cells = 0;
    //     // num_free_cells = 0;
    //     // num_unexplored_cells = 0;
    //     // num_out_of_map_cells = 0;
    //     //
    //     // auto mask = ComputeLidarFrameMask(position, theta, angles_body, ranges, clip_ranges, kRayMode, kInMapOnly);
    //     // int n_rows = mask.mask.rows;
    //     // int n_cols = mask.mask.cols;
    //     //
    //     // int num_not_scanned_cells = 0;
    //     // for (int row = 0; row < n_rows; ++row) {
    //     //     for (int col = 0; col < n_cols; ++col) {
    //     //         if (mask.mask.at<uint8_t>(row, col) == CellType::kUnexplored) {  // unexplored <--> not scanned
    //     //             num_not_scanned_cells++;
    //     //             continue;
    //     //         }
    //     //
    //     //         int x = mask.x_grid_min + row;
    //     //         int y = mask.y_grid_min + col;
    //     //
    //     //         if (x < 0 || y < 0 || x >= m_grid_map_info_->Shape(0) || y >= m_grid_map_info_->Shape(1)) {
    //     //             num_out_of_map_cells++;
    //     //             continue;
    //     //         }
    //     //
    //     //         const auto &kOccupancyValue = m_occupancy_map_.at<uint8_t>(x, y);
    //     //         if (kOccupancyValue == CellType::kOccupied) {
    //     //             num_occupied_cells++;
    //     //         } else if (kOccupancyValue == CellType::kFree) {
    //     //             num_free_cells++;
    //     //         } else if (kOccupancyValue == CellType::kUnexplored) {
    //     //             num_unexplored_cells++;
    //     //         } else {
    //     //             throw std::runtime_error("Unexpected cell type.");
    //     //         }
    //     //     }
    //     // }
    //     //
    //     // ERL_ASSERT(num_not_scanned_cells + num_free_cells + num_occupied_cells + num_unexplored_cells + num_out_of_map_cells == n_rows * n_cols);
    // }

    std::vector<Eigen::Matrix2Xi>
    LogOddMap2D::GetFrontiers(bool clean_at_first, int approx_iters) const {
        if (m_num_free_cells_ + m_num_occupied_cells_ == 0) { return {}; }

        cv::Mat unexplored_mask;
        cv::Mat occupied_mask;
        cv::Mat free_mask;

        if (clean_at_first) {
            m_cleaned_mask_.unexplored_mask.copyTo(unexplored_mask);
            m_cleaned_mask_.occupied_mask.copyTo(occupied_mask);
            m_cleaned_mask_.free_mask.copyTo(free_mask);
        } else {
            m_mask_.unexplored_mask.copyTo(unexplored_mask);
            m_mask_.occupied_mask.copyTo(occupied_mask);
            m_mask_.free_mask.copyTo(free_mask);
        }

        cv::Mat dilated_unexplored_mask;
        cv::dilate(unexplored_mask, dilated_unexplored_mask, m_kernel_);

        // isolate the frontiers using the difference between the masks and looking for contours
        dilated_unexplored_mask |= occupied_mask;

        // convert to 32-bit signed integer due to the computing frontier_mask involves negative values
        dilated_unexplored_mask.convertTo(dilated_unexplored_mask, CV_32SC1);
        free_mask.convertTo(free_mask, CV_32SC1);
        cv::Mat frontier_mask = cv::abs(1 - dilated_unexplored_mask - free_mask);
        frontier_mask.convertTo(frontier_mask, CV_8UC1);  // convert back to unsigned byte for smaller memory footprint

        if (approx_iters > 0) {
            cv::Mat tmp_frontier_mask;
            cv::dilate(frontier_mask, tmp_frontier_mask, m_kernel_, cv::Point(-1, -1), approx_iters);
            cv::erode(tmp_frontier_mask, frontier_mask, m_kernel_, cv::Point(-1, -1), approx_iters);
        }

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        // retrieves all the contours without establishing any hierarchical relationships.
        // cv::CHAIN_APPROX_NONE: maintain all contour vertices!
        cv::findContours(frontier_mask, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        std::size_t n = contours.size();
        std::vector<Eigen::Matrix2Xi> frontiers(n);
        for (std::size_t i = 0; i < n; ++i) {
            auto &contour = contours[i];
            auto &frontier = frontiers[i];

            auto m = static_cast<long>(contour.size());
            frontier.resize(2, m);
            for (long j = 0; j < m; ++j) {
                frontier(0, j) = contour[j].y;  // OpenCV uses (y, x) for the coordinate system
                frontier(1, j) = contour[j].x;
            }
        }

        return frontiers;
    }

    std::shared_ptr<LogOddMap2D::LidarFrameMask>
    LogOddMap2D::ComputeLidarFrameMask(
        const Eigen::Ref<const Eigen::Vector2d> &position,
        const double theta,
        const Eigen::Ref<const Eigen::VectorXd> &angles_body,
        const Eigen::Ref<const Eigen::VectorXd> &ranges,
        const bool clip_ranges,
        const bool ray_mode,
        const bool in_map_only,
        const std::shared_ptr<LidarFrameMask> &old_mask) const {

        ERL_DEBUG_ASSERT(angles_body.size() > 1, "angles_body is <= 1.");
        ERL_DEBUG_ASSERT(ranges.size() > 1, "ranges is <= 1.");
        ERL_DEBUG_ASSERT(angles_body.size() == ranges.size(), "angles_body and ranges have different sizes.");

        // LidarFrameMask mask;
        auto mask = std::make_shared<LidarFrameMask>();
        Eigen::VectorXd angles = angles_body.array() + theta;  // in world frame

        // clip the ranges if necessary, check if the ray hits an obstacle
        const long num_rays = angles.size();
        mask->occupied_grids.resize(2, num_rays);
        long num_obstacle_grids = 0;
        Eigen::VectorXd clipped_ranges;
        clipped_ranges.resize(num_rays);
        for (int i = 0; i < num_rays; ++i) {
            const double &kRange = ranges[i];
            double &clipped_range = clipped_ranges[i];
            if (kRange >= m_setting_->sensor_max_range) {
                if (clip_ranges || std::isinf(kRange)) {
                    clipped_range = m_setting_->sensor_max_range;
                } else {
                    clipped_range = kRange;
                }
            } else if (kRange < m_setting_->sensor_min_range) {
                clipped_range = 0.;
            } else {  // the ray hit an obstacle
                clipped_range = kRange;
                // clang-format off
                mask->occupied_grids.col(num_obstacle_grids++) <<
                    m_grid_map_info_->MeterToGridForValue(position[0] + kRange * std::cos(angles[i]), 0),
                    m_grid_map_info_->MeterToGridForValue(position[1] + kRange * std::sin(angles[i]), 1);
                // clang-format on
            }
        }
        mask->occupied_grids.conservativeResize(2, num_obstacle_grids);

        // compute the boundary of the in-map lidar scan area
        std::vector<std::vector<cv::Point>> lidar_area_contours(1);
        auto &contour = lidar_area_contours[0];
        int start_x = m_grid_map_info_->MeterToGridForValue(position[0], 0);
        int start_y = m_grid_map_info_->MeterToGridForValue(position[1], 1);
        if (ray_mode) {  // start, end1, end2, end3, ..., endN
            contour.reserve(2 * num_rays);
        } else {  // area mode: start, end1, start, end2, start, end3, ..., start, endN
            contour.reserve(1 + num_rays);
            contour.emplace_back(start_y, start_x);
        }
        mask->UpdateGridRange(start_x, start_y);
        for (int i = 0; i < num_rays; ++i) {
            Eigen::Vector2d direction(std::cos(angles[i]), std::sin(angles[i]));
            if (ray_mode) { contour.emplace_back(start_y, start_x); }
            const double &distance = clipped_ranges[i];
            // if (distance > clipped_ranges[i]) { distance = clipped_ranges[i]; }
            int x = m_grid_map_info_->MeterToGridForValue(position[0] + direction[0] * distance, 0);
            int y = m_grid_map_info_->MeterToGridForValue(position[1] + direction[1] * distance, 1);
            contour.emplace_back(y, x);
            mask->UpdateGridRange(x, y);
        }

        if (old_mask != nullptr) {
            mask->x_grid_min = std::min(mask->x_grid_min, old_mask->x_grid_min);
            mask->x_grid_max = std::max(mask->x_grid_max, old_mask->x_grid_max);
            mask->y_grid_min = std::min(mask->y_grid_min, old_mask->y_grid_min);
            mask->y_grid_max = std::max(mask->y_grid_max, old_mask->y_grid_max);
        } else if (in_map_only) {
            if (mask->x_grid_min < 0) { mask->x_grid_min = 0; }
            if (mask->x_grid_max >= m_grid_map_info_->Shape(0)) { mask->x_grid_max = m_grid_map_info_->Shape(0) - 1; }
            if (mask->y_grid_min < 0) { mask->y_grid_min = 0; }
            if (mask->y_grid_max >= m_grid_map_info_->Shape(1)) { mask->y_grid_max = m_grid_map_info_->Shape(1) - 1; }
        }

        // draw the free grids
        int n_rows = mask->x_grid_max - mask->x_grid_min + 1;
        int n_cols = mask->y_grid_max - mask->y_grid_min + 1;
        ERL_DEBUG_ASSERT(n_rows >= 0 && n_cols >= 0, "n_rows: {}, n_cols: {}", n_rows, n_cols);
        if (n_rows == 0 || n_cols == 0) {
            if (old_mask == nullptr) { return mask; }
            return mask;
        }

        mask->mask = cv::Mat(n_rows, n_cols, CV_8UC1, cv::Scalar(CellType::kUnexplored));  // cv::Mat(rows, cols, type, value)
        if (old_mask != nullptr) {
            old_mask->mask.copyTo(mask->mask(
                cv::Rect(old_mask->y_grid_min - mask->y_grid_min, old_mask->x_grid_min - mask->x_grid_min, old_mask->mask.cols, old_mask->mask.rows)));
        }

        // vector of points (x, y), in OpenCV, (x, y) = (col, row). So, mask.x is point.y, mask.y is point.x
        for (auto &point: contour) {
            point.x -= mask->y_grid_min;
            point.y -= mask->x_grid_min;
        }
        cv::drawContours(mask->mask, lidar_area_contours, 0, CellType::kFree, cv::FILLED, cv::LINE_8);

        // if (old_mask != nullptr) {
        //     ERL_INFO("old_mask size: \n{}, {}", old_mask->mask.rows, old_mask->mask.cols);
        //     ERL_INFO("new_mask size: \n{}, {}", mask->mask.rows, mask->mask.cols);
        //     cv::Mat old_mask_image;
        //     old_mask->mask.copyTo(old_mask_image);
        //     cv::transpose(old_mask_image, old_mask_image);
        //     cv::flip(old_mask_image, old_mask_image, 0);
        //     cv::imshow("old_mask", old_mask_image);
        //     cv::Mat mask_image;
        //     mask->mask.copyTo(mask_image);
        //     cv::transpose(mask_image, mask_image);
        //     cv::flip(mask_image, mask_image, 0);
        //     cv::imshow("mask", mask_image);
        //     cv::waitKey(0);
        // }

        // draw the occupied grids
        for (int i = 0; i < num_obstacle_grids; ++i) {
            const int x = mask->occupied_grids(0, i) - mask->x_grid_min;
            if (const int y = mask->occupied_grids(1, i) - mask->y_grid_min; x >= 0 && x < n_rows && y >= 0 && y < n_cols) {
                mask->mask.at<uint8_t>(x, y) = kOccupied;
            }
        }

        return mask;
    }

    std::shared_ptr<LogOddMap2D::LidarFrameMask>
    LogOddMap2D::ComputeLidarFramesMask(
        const Eigen::Ref<const Eigen::Matrix3Xd> &lidar_poses,
        const Eigen::Ref<const Eigen::VectorXd> &lidar_angles_body,
        const std::vector<Eigen::VectorXd> &lidar_ranges,
        const bool clip_ranges,
        const std::shared_ptr<LidarFrameMask> &old_mask) const {

        ERL_DEBUG_ASSERT(lidar_angles_body.size() > 1, "angles_body is <= 1.");

        const long num_rays = lidar_angles_body.size();
        const long num_frames = lidar_poses.cols();
        auto mask = std::make_shared<LidarFrameMask>();
        mask->occupied_grids.resize(2, num_rays * num_frames);
        long num_obstacle_grids = 0;
        std::vector<Eigen::VectorXd> clipped_lidar_ranges(num_frames);
        std::vector<std::vector<cv::Point>> lidar_area_contours(num_frames);

        for (long i = 0; i < num_frames; ++i) {
            const double &lidar_x = lidar_poses(0, i);
            const double &lidar_y = lidar_poses(1, i);
            Eigen::VectorXd angles = lidar_angles_body.array() + lidar_poses(2, i);  // in world frame
            const auto &ranges = lidar_ranges[i];
            auto &clipped_ranges = clipped_lidar_ranges[i];
            clipped_ranges.resize(num_rays);

            ERL_DEBUG_ASSERT(ranges.size() > 1, "kRanges.size() <= 1.");
            ERL_DEBUG_ASSERT(
                lidar_angles_body.size() == ranges.size(),
                "angles_body and ranges have different sizes: {} vs {}.",
                lidar_angles_body.size(),
                ranges.size());

            // clip the ranges if necessary, check if the ray hits an obstacle
            for (int j = 0; j < num_rays; ++j) {
                const double &kRange = ranges[j];
                double &clipped_range = clipped_ranges[j];
                if (kRange >= m_setting_->sensor_max_range) {
                    if (clip_ranges || std::isinf(kRange)) {
                        clipped_range = m_setting_->sensor_max_range;
                    } else {
                        clipped_range = kRange;
                    }
                } else if (kRange < m_setting_->sensor_min_range) {
                    clipped_range = 0.;
                } else {  // the ray hit an obstacle
                    clipped_range = kRange;
                    // clang-format off
                const double &angle = angles[j];
                mask->occupied_grids.col(num_obstacle_grids++) <<
                    m_grid_map_info_->MeterToGridForValue(lidar_x + kRange * std::cos(angle), 0),
                    m_grid_map_info_->MeterToGridForValue(lidar_y + kRange * std::sin(angle), 1);
                    // clang-format on
                }
            }

            // compute the boundary of the lidar scan area
            auto &contour = lidar_area_contours[i];
            int start_x = m_grid_map_info_->MeterToGridForValue(lidar_x, 0);
            int start_y = m_grid_map_info_->MeterToGridForValue(lidar_y, 1);

            // if (ray_mode) {  // start, end1, end2, end3, ..., endN
            //     contour.reserve(2 * num_rays);
            // } else {  // area mode: start, end1, start, end2, start, end3, ..., start, endN
            //     contour.reserve(1 + num_rays);
            //     contour.emplace_back(start_y, start_x);
            // }

            // area mode: start, end1, start, end2, start, end3, ..., start, endN
            contour.reserve(1 + num_rays);
            contour.emplace_back(start_y, start_x);

            mask->UpdateGridRange(start_x, start_y);
            for (int j = 0; j < num_rays; ++j) {
                Eigen::Vector2d direction(std::cos(angles[j]), std::sin(angles[j]));
                // if (ray_mode) { contour.emplace_back(start_y, start_x); }
                const double distance = clipped_ranges[j];
                int x = m_grid_map_info_->MeterToGridForValue(lidar_x + direction[0] * distance, 0);
                int y = m_grid_map_info_->MeterToGridForValue(lidar_y + direction[1] * distance, 1);
                contour.emplace_back(y, x);
                mask->UpdateGridRange(x, y);
            }
        }

        mask->occupied_grids.conservativeResize(2, num_obstacle_grids);

        // if (InMapOnly) {
        //     if (mask.x_grid_min < 0) { mask.x_grid_min = 0; }
        //     if (mask.x_grid_max >= m_grid_map_info_->Shape(0)) { mask.x_grid_max = m_grid_map_info_->Shape(0) - 1; }
        //     if (mask.y_grid_min < 0) { mask.y_grid_min = 0; }
        //     if (mask.y_grid_max >= m_grid_map_info_->Shape(1)) { mask.y_grid_max = m_grid_map_info_->Shape(1) - 1; }
        // }
        if (old_mask != nullptr) {
            mask->x_grid_min = std::min(mask->x_grid_min, old_mask->x_grid_min);
            mask->x_grid_max = std::max(mask->x_grid_max, old_mask->x_grid_max);
            mask->y_grid_min = std::min(mask->y_grid_min, old_mask->y_grid_min);
            mask->y_grid_max = std::max(mask->y_grid_max, old_mask->y_grid_max);
        }

        // draw the free grids
        int n_rows = mask->x_grid_max - mask->x_grid_min + 1;
        int n_cols = mask->y_grid_max - mask->y_grid_min + 1;
        ERL_DEBUG_ASSERT(n_rows >= 0 && n_cols >= 0, "n_rows: {}, n_cols: {}", n_rows, n_cols);
        if (n_rows == 0 || n_cols == 0) {
            if (old_mask == nullptr) { return mask; }
            return mask;
        }

        mask->mask = cv::Mat(n_rows, n_cols, CV_8UC1, cv::Scalar(CellType::kUnexplored));  // cv::Mat(rows, cols, type, value)
        if (old_mask != nullptr) {
            ERL_INFO("old_mask size: {}, {}", old_mask->mask.rows, old_mask->mask.cols);
            ERL_INFO("new_mask size: {}, {}", mask->mask.rows, mask->mask.cols);
            old_mask->mask.copyTo(mask->mask(
                cv::Rect(old_mask->y_grid_min - mask->y_grid_min, old_mask->x_grid_min - mask->x_grid_min, old_mask->mask.cols, old_mask->mask.rows)));
        }

        // cv::Mat image = cv::Mat::zeros(n_rows, n_cols, CV_8UC3);
        // vector of points (x, y), in OpenCV, (x, y) = (col, row). So, mask.x is point.y, mask.y is point.x
        // int s = 255 / int(num_frames);
        // int d = 1;
        // if (s == 0) {
        //     s = 1;
        //     d = int(num_frames) / 255;
        // }
        for (long i = 0; i < num_frames; ++i) {
            auto &contour = lidar_area_contours[i];
            for (auto &point: contour) {
                point.x -= mask->y_grid_min;
                point.y -= mask->x_grid_min;
            }
            cv::drawContours(mask->mask, lidar_area_contours, static_cast<int>(i), CellType::kFree, cv::FILLED, cv::LINE_8);
            // cv::drawContours(image, lidar_area_contours, int(i), cv::Scalar(0, 255 - s * int(i / d), s * int(i / d)), cv::FILLED, cv::LINE_8);
        }
        // cv::drawContours(mask.mask, lidar_area_contours, 0, CellType::kFree, cv::FILLED, cv::LINE_8);

        // if (old_mask != nullptr) {
        //     cv::Mat old_mask_image;
        //     old_mask->mask.copyTo(old_mask_image);
        //     cv::transpose(old_mask_image, old_mask_image);
        //     cv::flip(old_mask_image, old_mask_image, 0);
        //     cv::imshow("old_mask", old_mask_image);
        //     cv::Mat mask_image;
        //     mask->mask.copyTo(mask_image);
        //     cv::transpose(mask_image, mask_image);
        //     cv::flip(mask_image, mask_image, 0);
        //     cv::imshow("mask", mask_image);
        // }
        // // cv::waitKey(0);
        // std::vector<cv::Point2i> path_points;
        // for (int i = 0; i < num_frames; ++i) { path_points.emplace_back(lidar_area_contours[i][0]); }
        // cv::polylines(image, path_points, false, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        // cv::transpose(image, image);
        // cv::flip(image, image, 0);
        // cv::imshow("image", image);
        // cv::waitKey(0);

        // draw the occupied grids
        for (int i = 0; i < num_obstacle_grids; ++i) {
            const int x = mask->occupied_grids(0, i) - mask->x_grid_min;
            if (const int y = mask->occupied_grids(1, i) - mask->y_grid_min; x >= 0 && x < n_rows && y >= 0 && y < n_cols) {
                mask->mask.at<uint8_t>(x, y) = kOccupied;
            }
        }

        return mask;
    }

    void
    LogOddMap2D::ComputeStatisticsOfLidarFrameMask(
        const std::shared_ptr<LidarFrameMask> &mask,
        int &num_occupied_cells,
        int &num_free_cells,
        int &num_unexplored_cells,
        int &num_out_of_map_cells) const {

        num_occupied_cells = 0;
        num_free_cells = 0;
        num_unexplored_cells = 0;
        num_out_of_map_cells = 0;

        const int n_rows = mask->mask.rows;
        const int n_cols = mask->mask.cols;

        int num_not_scanned_cells = 0;
        for (int row = 0; row < n_rows; ++row) {
            for (int col = 0; col < n_cols; ++col) {
                if (mask->mask.at<uint8_t>(row, col) == kUnexplored) {  // unexplored <--> not scanned
                    num_not_scanned_cells++;
                    continue;
                }

                const int x = mask->x_grid_min + row;
                const int y = mask->y_grid_min + col;

                if (x < 0 || y < 0 || x >= m_grid_map_info_->Shape(0) || y >= m_grid_map_info_->Shape(1)) {
                    num_out_of_map_cells++;
                    continue;
                }

                if (const auto &occupancy_value = m_occupancy_map_.at<uint8_t>(x, y); occupancy_value == kOccupied) {
                    num_occupied_cells++;
                } else if (occupancy_value == kFree) {
                    num_free_cells++;
                } else if (occupancy_value == kUnexplored) {
                    num_unexplored_cells++;
                } else {
                    throw std::runtime_error("Unexpected cell type.");
                }
            }
        }

        ERL_ASSERT(num_not_scanned_cells + num_free_cells + num_occupied_cells + num_unexplored_cells + num_out_of_map_cells == n_rows * n_cols);
    }

    void
    LogOddMap2D::PostProcessMasks(const Eigen::Ref<const Eigen::Vector2d> &position, const double theta) {
        // update cleaned mask
        m_mask_.free_mask.copyTo(m_cleaned_mask_.free_mask);
        // dilate then erode to remove isolated free cells
        if (m_setting_->num_iters_for_cleaned_mask > 0) {
            cv::dilate(m_cleaned_mask_.free_mask, m_cleaned_mask_.free_mask, m_kernel_, cv::Point(-1, -1), m_setting_->num_iters_for_cleaned_mask);
            cv::erode(m_cleaned_mask_.free_mask, m_cleaned_mask_.free_mask, m_kernel_, cv::Point(-1, -1), m_setting_->num_iters_for_cleaned_mask);
        }
        m_mask_.occupied_mask.copyTo(m_cleaned_mask_.occupied_mask);
        if (m_setting_->filter_obstacles_in_cleaned_mask) { cv::medianBlur(m_cleaned_mask_.occupied_mask, m_cleaned_mask_.occupied_mask, 3); }
        m_cleaned_mask_.unexplored_mask.setTo(cv::Scalar(1));
        m_cleaned_mask_.unexplored_mask -= m_cleaned_mask_.free_mask | m_cleaned_mask_.occupied_mask;
        cv::dilate(m_cleaned_mask_.occupied_mask, m_cleaned_mask_.occupied_mask, m_kernel_, cv::Point(-1, -1), 1);
        m_cleaned_mask_.free_mask -= m_cleaned_mask_.occupied_mask | m_cleaned_mask_.unexplored_mask;

        // grids occupied by the robot are free
        const long num_vertices = m_shape_vertices_.cols();
        if (num_vertices == 0) { return; }
        std::vector<std::vector<cv::Point>> contour(1);
        auto &robot_shape = contour[0];
        robot_shape.reserve(num_vertices);
        const Eigen::Matrix2d rotation_matrix = Eigen::Rotation2Dd(theta).toRotationMatrix();
        for (int i = 0; i < num_vertices; ++i) {
            Eigen::Vector2d vertex = rotation_matrix * m_shape_vertices_.col(i) + position;
            int x = m_grid_map_info_->MeterToGridForValue(vertex[0], 0);  // row
            int y = m_grid_map_info_->MeterToGridForValue(vertex[1], 1);  // col
            robot_shape.emplace_back(y, x);                               // (col, row)
        }
        cv::drawContours(m_cleaned_mask_.free_mask, contour, 0, cv::Scalar(1), cv::FILLED);
        cv::drawContours(m_cleaned_mask_.occupied_mask, contour, 0, cv::Scalar(0), cv::FILLED);
        cv::drawContours(m_cleaned_mask_.unexplored_mask, contour, 0, cv::Scalar(0), cv::FILLED);
    }
}  // namespace erl::geometry
