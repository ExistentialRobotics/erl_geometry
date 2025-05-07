#pragma once

namespace erl::geometry {

    template<typename Dtype>
    YAML::Node
    LogOddMap2D<Dtype>::Setting::YamlConvertImpl::encode(const Setting &setting) {
        YAML::Node node;
        ERL_YAML_SAVE_ATTR(node, setting, sensor_min_range);
        ERL_YAML_SAVE_ATTR(node, setting, sensor_max_range);
        ERL_YAML_SAVE_ATTR(node, setting, measurement_certainty);
        ERL_YAML_SAVE_ATTR(node, setting, max_log_odd);
        ERL_YAML_SAVE_ATTR(node, setting, min_log_odd);
        ERL_YAML_SAVE_ATTR(node, setting, threshold_occupied);
        ERL_YAML_SAVE_ATTR(node, setting, threshold_free);
        ERL_YAML_SAVE_ATTR(node, setting, use_cross_kernel);
        ERL_YAML_SAVE_ATTR(node, setting, num_iters_for_cleaned_mask);
        ERL_YAML_SAVE_ATTR(node, setting, filter_obstacles_in_cleaned_mask);
        return node;
    }

    template<typename Dtype>
    bool
    LogOddMap2D<Dtype>::Setting::YamlConvertImpl::decode(const YAML::Node &node, Setting &setting) {
        if (!node.IsMap()) { return false; }
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, sensor_min_range, Dtype);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, sensor_max_range, Dtype);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, measurement_certainty, Dtype);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, max_log_odd, Dtype);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, min_log_odd, Dtype);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, threshold_occupied, Dtype);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, threshold_free, Dtype);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, use_cross_kernel, bool);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, num_iters_for_cleaned_mask, int);
        ERL_YAML_LOAD_ATTR_TYPE(node, setting, filter_obstacles_in_cleaned_mask, bool);
        return true;
    }

    template<typename Dtype>
    LogOddMap2D<Dtype>::LogOddMap2D(
        std::shared_ptr<Setting> setting,
        std::shared_ptr<common::GridMapInfo2D<Dtype>> grid_map_info)
        : m_setting_(std::move(setting)),
          m_grid_map_info_(std::move(grid_map_info)),
          m_log_map_(
              m_grid_map_info_->Shape(0),
              m_grid_map_info_->Shape(1),
              sizeof(Dtype) == 8 ? CV_64FC1 : CV_32FC1,
              cv::Scalar{0}),
          m_possibility_map_(
              m_grid_map_info_->Shape(0),
              m_grid_map_info_->Shape(1),
              sizeof(Dtype) == 8 ? CV_64FC1 : CV_32FC1,
              cv::Scalar{0.5}),
          m_occupancy_map_(
              m_grid_map_info_->Shape(0),
              m_grid_map_info_->Shape(1),
              CV_8UC1,
              cv::Scalar{kUnexplored}),
          m_kernel_(cv::getStructuringElement(
              m_setting_->use_cross_kernel ? cv::MORPH_CROSS : cv::MORPH_RECT,
              cv::Size{3, 3})),
          m_mask_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1)),
          m_cleaned_mask_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1)),
          m_num_unexplored_cells_(m_grid_map_info_->Shape(0) * m_grid_map_info_->Shape(1)) {}

    template<typename Dtype>
    LogOddMap2D<Dtype>::LogOddMap2D(
        std::shared_ptr<Setting> setting,
        std::shared_ptr<common::GridMapInfo2D<Dtype>> grid_map_info,
        const Eigen::Ref<const Eigen::Matrix2X<Dtype>> &shape_metric_vertices)
        : LogOddMap2D(std::move(setting), std::move(grid_map_info)) {
        m_shape_vertices_ = shape_metric_vertices;
    }

    template<typename Dtype>
    void
    LogOddMap2D<Dtype>::Update(
        const Eigen::Ref<const Eigen::Vector2<Dtype>> &position,
        const Dtype theta,
        const Eigen::Ref<const Eigen::VectorX<Dtype>> &angles_body,
        const Eigen::Ref<const Eigen::VectorX<Dtype>> &ranges) {

        ERL_DEBUG_ASSERT(
            angles_body.size() == ranges.size(),
            "angles and ranges should be of the same shape.");
        ERL_DEBUG_ASSERT(!ranges.hasNaN(), "detect nan in ranges!");

        // generate mask of lidar scan
        constexpr bool clip_ranges = true;
        constexpr bool ray_mode = true;
        constexpr bool in_map_only = true;
        const auto mask = ComputeLidarFrameMask(
            position,
            theta,
            angles_body,
            ranges,
            clip_ranges,
            ray_mode,
            in_map_only,
            nullptr);
        if (mask->mask.rows == 0 || mask->mask.cols == 0) { return; }

        // compute parameters
        const Dtype log_certainty = std::log(m_setting_->measurement_certainty);
        const Dtype log_uncertainty = std::log(1.0 - m_setting_->measurement_certainty);
        const Dtype log_odd_occupied = log_certainty - log_uncertainty;
        const Dtype log_odd_free = log_uncertainty - log_certainty;

        // update log_odd_map, possibility_map, occupancy_map
        for (int row = 0; row < mask->mask.rows; ++row) {
            for (int col = 0; col < mask->mask.cols; ++col) {
                const auto &mask_value = mask->mask.template at<uint8_t>(row, col);
                if (mask_value == kUnexplored) { continue; }

                const int x = mask->x_grid_min + row;
                const int y = mask->y_grid_min + col;
                auto &log_odd_value = m_log_map_.at<Dtype>(x, y);
                auto &possibility_value = m_possibility_map_.at<Dtype>(x, y);
                auto &occupancy_value = m_occupancy_map_.at<uint8_t>(x, y);
                auto &free_mask_value = m_mask_.free_mask.template at<uint8_t>(x, y);
                auto &occupied_mask_value = m_mask_.occupied_mask.template at<uint8_t>(x, y);
                auto &unexplored_mask_value = m_mask_.unexplored_mask.template at<uint8_t>(x, y);

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

    template<typename Dtype>
    void
    LogOddMap2D<Dtype>::LoadExternalPossibilityMap(
        const Eigen::Ref<const Eigen::Vector2<Dtype>> &position,
        const Dtype theta,
        const Eigen::Ref<const Eigen::MatrixXi> &possibility_map) {

        ERL_ASSERTM(
            possibility_map.rows() == m_grid_map_info_->Shape(0) &&
                possibility_map.cols() == m_grid_map_info_->Shape(1),
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
                auto &log_odd_value = m_log_map_.at<Dtype>(i, j);
                auto &possibility_value = m_possibility_map_.at<Dtype>(i, j);
                auto &occupancy_value = m_occupancy_map_.at<uint8_t>(i, j);
                auto &free_mask_value = m_mask_.free_mask.template at<uint8_t>(i, j);
                auto &occupied_mask_value = m_mask_.occupied_mask.template at<uint8_t>(i, j);
                auto &unexplored_mask_value = m_mask_.unexplored_mask.template at<uint8_t>(i, j);

                if (possibility_map(i, j) == -1) {
                    log_odd_value = 0.;
                    possibility_value = 0.5;
                    occupancy_value = kUnexplored;
                    free_mask_value = 0;
                    occupied_mask_value = 0;
                    unexplored_mask_value = 1;
                    m_num_unexplored_cells_++;
                } else {
                    possibility_value = static_cast<Dtype>(possibility_map(i, j)) / 100.;
                    log_odd_value = std::log(possibility_value / (1. - possibility_value));
                    if (possibility_value > m_setting_->threshold_occupied) {
                        occupancy_value = kOccupied;
                        free_mask_value = 0;
                        occupied_mask_value = 1;
                        unexplored_mask_value = 0;
                        m_num_occupied_cells_++;
                    } else if (possibility_value < m_setting_->threshold_free) {
                        occupancy_value = kFree;
                        free_mask_value = 1;
                        occupied_mask_value = 0;
                        unexplored_mask_value = 0;
                        m_num_free_cells_++;
                    } else {
                        occupancy_value = kUnexplored;
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

    template<typename Dtype>
    std::shared_ptr<typename LogOddMap2D<Dtype>::LidarFrameMask>
    LogOddMap2D<Dtype>::ComputeStatisticsOfLidarFrame(
        const Eigen::Ref<const Eigen::Vector2<Dtype>> &position,
        const Dtype theta,
        const Eigen::Ref<const Eigen::VectorX<Dtype>> &angles_body,
        const Eigen::Ref<const Eigen::VectorX<Dtype>> &ranges,
        const bool clip_ranges,
        const std::shared_ptr<LidarFrameMask> &old_mask,
        int &num_occupied_cells,
        int &num_free_cells,
        int &num_unexplored_cells,
        int &num_out_of_map_cells) const {

        constexpr bool ray_mode = false;
        constexpr bool in_map_only = false;
        auto new_mask = ComputeLidarFrameMask(
            position,
            theta,
            angles_body,
            ranges,
            clip_ranges,
            ray_mode,
            in_map_only,
            old_mask);
        ComputeStatisticsOfLidarFrameMask(
            new_mask,
            num_occupied_cells,
            num_free_cells,
            num_unexplored_cells,
            num_out_of_map_cells);
        return new_mask;
    }

    template<typename Dtype>
    std::shared_ptr<typename LogOddMap2D<Dtype>::LidarFrameMask>
    LogOddMap2D<Dtype>::ComputeStatisticsOfLidarFrames(
        const Eigen::Ref<const Eigen::Matrix3X<Dtype>> &lidar_poses,
        const Eigen::Ref<const Eigen::VectorX<Dtype>> &lidar_angles_body,
        const std::vector<Eigen::VectorX<Dtype>> &lidar_ranges,
        const bool clip_ranges,
        const std::shared_ptr<LidarFrameMask> &old_mask,
        int &num_occupied_cells,
        int &num_free_cells,
        int &num_unexplored_cells,
        int &num_out_of_map_cells) const {

        // const bool kRayMode = false;
        // const bool kInMapOnly = false;
        auto new_mask = ComputeLidarFramesMask(
            lidar_poses,
            lidar_angles_body,
            lidar_ranges,
            clip_ranges,
            old_mask);
        ComputeStatisticsOfLidarFrameMask(
            new_mask,
            num_occupied_cells,
            num_free_cells,
            num_unexplored_cells,
            num_out_of_map_cells);
        return new_mask;
    }

    template<typename Dtype>
    std::shared_ptr<typename LogOddMap2D<Dtype>::Setting>
    LogOddMap2D<Dtype>::GetSetting() const {
        return m_setting_;
    }

    template<typename Dtype>
    std::shared_ptr<const common::GridMapInfo2D<Dtype>>
    LogOddMap2D<Dtype>::GetGridMapInfo() const {
        return m_grid_map_info_;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetLogMap() const {
        return m_log_map_;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetPossibilityMap() const {
        return m_possibility_map_;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetOccupancyMap() const {
        return m_occupancy_map_;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetUnexploredMask() const {
        return m_mask_.unexplored_mask;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetOccupiedMask() const {
        return m_mask_.occupied_mask;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetFreeMask() const {
        return m_mask_.free_mask;
    }

    template<typename Dtype>
    std::size_t
    LogOddMap2D<Dtype>::GetNumUnexploredCells() const {
        return m_num_unexplored_cells_;
    }

    template<typename Dtype>
    std::size_t
    LogOddMap2D<Dtype>::GetNumOccupiedCells() const {
        return m_num_occupied_cells_;
    }

    template<typename Dtype>
    std::size_t
    LogOddMap2D<Dtype>::GetNumFreeCells() const {
        return m_num_free_cells_;
    }

    template<typename Dtype>
    const typename LogOddMap2D<Dtype>::LogOddCVMask &
    LogOddMap2D<Dtype>::GetCleanedMasks() const {
        return m_cleaned_mask_;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetCleanedFreeMask() const {
        return m_cleaned_mask_.free_mask;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetCleanedOccupiedMask() const {
        return m_cleaned_mask_.occupied_mask;
    }

    template<typename Dtype>
    cv::Mat
    LogOddMap2D<Dtype>::GetCleanedUnexploredMask() const {
        return m_cleaned_mask_.unexplored_mask;
    }

    template<typename Dtype>
    std::vector<Eigen::Matrix2Xi>
    LogOddMap2D<Dtype>::GetFrontiers(bool clean_at_first, int approx_iters) const {
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

        // convert to 32-bit signed int due to the computing frontier_mask involves negative values
        dilated_unexplored_mask.convertTo(dilated_unexplored_mask, CV_32SC1);
        free_mask.convertTo(free_mask, CV_32SC1);
        cv::Mat frontier_mask = cv::abs(1 - dilated_unexplored_mask - free_mask);
        // convert back to unsigned byte for a smaller memory footprint
        frontier_mask.convertTo(frontier_mask, CV_8UC1);

        if (approx_iters > 0) {
            cv::Mat tmp_frontier_mask;
            cv::dilate(
                frontier_mask,
                tmp_frontier_mask,
                m_kernel_,
                cv::Point(-1, -1),
                approx_iters);
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

    template<typename Dtype>
    std::shared_ptr<typename LogOddMap2D<Dtype>::LidarFrameMask>
    LogOddMap2D<Dtype>::ComputeLidarFrameMask(
        const Eigen::Ref<const Eigen::Vector2<Dtype>> &position,
        const Dtype theta,
        const Eigen::Ref<const Eigen::VectorX<Dtype>> &angles_body,
        const Eigen::Ref<const Eigen::VectorX<Dtype>> &ranges,
        const bool clip_ranges,
        const bool ray_mode,
        const bool in_map_only,
        const std::shared_ptr<LidarFrameMask> &old_mask) const {

        ERL_DEBUG_ASSERT(angles_body.size() > 1, "angles_body is <= 1.");
        ERL_DEBUG_ASSERT(ranges.size() > 1, "ranges is <= 1.");
        ERL_DEBUG_ASSERT(
            angles_body.size() == ranges.size(),
            "angles_body and ranges have different sizes.");

        // LidarFrameMask mask;
        auto mask = std::make_shared<LidarFrameMask>();
        Eigen::VectorX<Dtype> angles = angles_body.array() + theta;  // in world frame

        // clip the ranges if necessary, check if the ray hits an obstacle
        const long num_rays = angles.size();
        mask->occupied_grids.resize(2, num_rays);
        long num_obstacle_grids = 0;
        Eigen::VectorX<Dtype> clipped_ranges;
        clipped_ranges.resize(num_rays);
        for (int i = 0; i < num_rays; ++i) {
            const Dtype &kRange = ranges[i];
            Dtype &clipped_range = clipped_ranges[i];
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
            Eigen::Vector2<Dtype> direction(std::cos(angles[i]), std::sin(angles[i]));
            if (ray_mode) { contour.emplace_back(start_y, start_x); }
            const Dtype &distance = clipped_ranges[i];
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
            if (mask->x_grid_max >= m_grid_map_info_->Shape(0)) {
                mask->x_grid_max = m_grid_map_info_->Shape(0) - 1;
            }
            if (mask->y_grid_min < 0) { mask->y_grid_min = 0; }
            if (mask->y_grid_max >= m_grid_map_info_->Shape(1)) {
                mask->y_grid_max = m_grid_map_info_->Shape(1) - 1;
            }
        }

        // draw the free grids
        const int n_rows = mask->x_grid_max - mask->x_grid_min + 1;
        const int n_cols = mask->y_grid_max - mask->y_grid_min + 1;
        ERL_DEBUG_ASSERT(n_rows >= 0 && n_cols >= 0, "n_rows: {}, n_cols: {}", n_rows, n_cols);
        if (n_rows == 0 || n_cols == 0) {
            if (old_mask == nullptr) { return mask; }
            return mask;
        }
        // cv::Mat(rows, cols, type, value)
        mask->mask = cv::Mat(n_rows, n_cols, CV_8UC1, cv::Scalar(kUnexplored));
        if (old_mask != nullptr) {
            old_mask->mask.copyTo(mask->mask(cv::Rect(
                old_mask->y_grid_min - mask->y_grid_min,
                old_mask->x_grid_min - mask->x_grid_min,
                old_mask->mask.cols,
                old_mask->mask.rows)));
        }

        // vector of points (x, y), in OpenCV, (x, y) = (col, row).
        // So, mask.x is point.y, mask.y is point.x
        for (auto &point: contour) {
            point.x -= mask->y_grid_min;
            point.y -= mask->x_grid_min;
        }
        cv::drawContours(mask->mask, lidar_area_contours, 0, kFree, cv::FILLED, cv::LINE_8);

        // draw the occupied grids
        for (int i = 0; i < num_obstacle_grids; ++i) {
            const int x = mask->occupied_grids(0, i) - mask->x_grid_min;
            if (const int y = mask->occupied_grids(1, i) - mask->y_grid_min;
                x >= 0 && x < n_rows && y >= 0 && y < n_cols) {
                mask->mask.template at<uint8_t>(x, y) = kOccupied;
            }
        }

        return mask;
    }

    template<typename Dtype>
    std::shared_ptr<typename LogOddMap2D<Dtype>::LidarFrameMask>
    LogOddMap2D<Dtype>::ComputeLidarFramesMask(
        const Eigen::Ref<const Eigen::Matrix3Xd> &lidar_poses,
        const Eigen::Ref<const Eigen::VectorX<Dtype>> &lidar_angles_body,
        const std::vector<Eigen::VectorX<Dtype>> &lidar_ranges,
        const bool clip_ranges,
        const std::shared_ptr<LidarFrameMask> &old_mask) const {

        ERL_DEBUG_ASSERT(lidar_angles_body.size() > 1, "angles_body is <= 1.");

        const long num_rays = lidar_angles_body.size();
        const long num_frames = lidar_poses.cols();
        auto mask = std::make_shared<LidarFrameMask>();
        mask->occupied_grids.resize(2, num_rays * num_frames);
        long num_obstacle_grids = 0;
        std::vector<Eigen::VectorX<Dtype>> clipped_lidar_ranges(num_frames);
        std::vector<std::vector<cv::Point>> lidar_area_contours(num_frames);

        for (long i = 0; i < num_frames; ++i) {
            const Dtype &lidar_x = lidar_poses(0, i);
            const Dtype &lidar_y = lidar_poses(1, i);
            // in world frame
            Eigen::VectorX<Dtype> angles = lidar_angles_body.array() + lidar_poses(2, i);
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
                const Dtype &kRange = ranges[j];
                Dtype &clipped_range = clipped_ranges[j];
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
                    const Dtype &angle = angles[j];
                    // clang-format off
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

            // area mode: start, end1, start, end2, start, end3, ..., start, endN
            contour.reserve(1 + num_rays);
            contour.emplace_back(start_y, start_x);

            mask->UpdateGridRange(start_x, start_y);
            for (int j = 0; j < num_rays; ++j) {
                Eigen::Vector2<Dtype> direction(std::cos(angles[j]), std::sin(angles[j]));
                const Dtype distance = clipped_ranges[j];
                int x = m_grid_map_info_->MeterToGridForValue(lidar_x + direction[0] * distance, 0);
                int y = m_grid_map_info_->MeterToGridForValue(lidar_y + direction[1] * distance, 1);
                contour.emplace_back(y, x);
                mask->UpdateGridRange(x, y);
            }
        }

        mask->occupied_grids.conservativeResize(2, num_obstacle_grids);

        if (old_mask != nullptr) {
            mask->x_grid_min = std::min(mask->x_grid_min, old_mask->x_grid_min);
            mask->x_grid_max = std::max(mask->x_grid_max, old_mask->x_grid_max);
            mask->y_grid_min = std::min(mask->y_grid_min, old_mask->y_grid_min);
            mask->y_grid_max = std::max(mask->y_grid_max, old_mask->y_grid_max);
        }

        // draw the free grids
        const int n_rows = mask->x_grid_max - mask->x_grid_min + 1;
        const int n_cols = mask->y_grid_max - mask->y_grid_min + 1;
        ERL_DEBUG_ASSERT(n_rows >= 0 && n_cols >= 0, "n_rows: {}, n_cols: {}", n_rows, n_cols);
        if (n_rows == 0 || n_cols == 0) {
            if (old_mask == nullptr) { return mask; }
            return mask;
        }
        // cv::Mat(rows, cols, type, value)
        mask->mask = cv::Mat(n_rows, n_cols, CV_8UC1, cv::Scalar(kUnexplored));
        if (old_mask != nullptr) {
            ERL_INFO("old_mask size: {}, {}", old_mask->mask.rows, old_mask->mask.cols);
            ERL_INFO("new_mask size: {}, {}", mask->mask.rows, mask->mask.cols);
            old_mask->mask.copyTo(mask->mask(cv::Rect(
                old_mask->y_grid_min - mask->y_grid_min,
                old_mask->x_grid_min - mask->x_grid_min,
                old_mask->mask.cols,
                old_mask->mask.rows)));
        }

        for (long i = 0; i < num_frames; ++i) {
            auto &contour = lidar_area_contours[i];
            for (auto &point: contour) {
                point.x -= mask->y_grid_min;
                point.y -= mask->x_grid_min;
            }
            cv::drawContours(
                mask->mask,
                lidar_area_contours,
                static_cast<int>(i),
                kFree,
                cv::FILLED,
                cv::LINE_8);
        }

        // draw the occupied grids
        for (int i = 0; i < num_obstacle_grids; ++i) {
            const int x = mask->occupied_grids(0, i) - mask->x_grid_min;
            if (const int y = mask->occupied_grids(1, i) - mask->y_grid_min;
                x >= 0 && x < n_rows && y >= 0 && y < n_cols) {
                mask->mask.template at<uint8_t>(x, y) = kOccupied;
            }
        }

        return mask;
    }

    template<typename Dtype>
    void
    LogOddMap2D<Dtype>::ComputeStatisticsOfLidarFrameMask(
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
                if (mask->mask.template at<uint8_t>(row, col) == kUnexplored) {
                    // unexplored <--> not scanned
                    num_not_scanned_cells++;
                    continue;
                }

                const int x = mask->x_grid_min + row;
                const int y = mask->y_grid_min + col;

                if (x < 0 || y < 0 || x >= m_grid_map_info_->Shape(0) ||
                    y >= m_grid_map_info_->Shape(1)) {
                    num_out_of_map_cells++;
                    continue;
                }

                if (const auto &occupancy_value = m_occupancy_map_.at<uint8_t>(x, y);
                    occupancy_value == kOccupied) {
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

        ERL_ASSERT(
            num_not_scanned_cells + num_free_cells + num_occupied_cells + num_unexplored_cells +
                num_out_of_map_cells ==
            n_rows * n_cols);
    }

    template<typename Dtype>
    void
    LogOddMap2D<Dtype>::PostProcessMasks(
        const Eigen::Ref<const Eigen::Vector2<Dtype>> &position,
        const Dtype theta) {
        // update cleaned mask
        m_mask_.free_mask.copyTo(m_cleaned_mask_.free_mask);
        // dilate then erode to remove isolated free cells
        if (m_setting_->num_iters_for_cleaned_mask > 0) {
            cv::dilate(
                m_cleaned_mask_.free_mask,
                m_cleaned_mask_.free_mask,
                m_kernel_,
                cv::Point(-1, -1),
                m_setting_->num_iters_for_cleaned_mask);
            cv::erode(
                m_cleaned_mask_.free_mask,
                m_cleaned_mask_.free_mask,
                m_kernel_,
                cv::Point(-1, -1),
                m_setting_->num_iters_for_cleaned_mask);
        }
        m_mask_.occupied_mask.copyTo(m_cleaned_mask_.occupied_mask);
        if (m_setting_->filter_obstacles_in_cleaned_mask) {
            cv::medianBlur(m_cleaned_mask_.occupied_mask, m_cleaned_mask_.occupied_mask, 3);
        }
        m_cleaned_mask_.unexplored_mask.setTo(cv::Scalar(1));
        m_cleaned_mask_.unexplored_mask -=
            m_cleaned_mask_.free_mask | m_cleaned_mask_.occupied_mask;
        cv::dilate(
            m_cleaned_mask_.occupied_mask,
            m_cleaned_mask_.occupied_mask,
            m_kernel_,
            cv::Point(-1, -1),
            1);
        m_cleaned_mask_.free_mask -=
            m_cleaned_mask_.occupied_mask | m_cleaned_mask_.unexplored_mask;

        // grids occupied by the robot are free
        const long num_vertices = m_shape_vertices_.cols();
        if (num_vertices == 0) { return; }
        std::vector<std::vector<cv::Point>> contour(1);
        auto &robot_shape = contour[0];
        robot_shape.reserve(num_vertices);
        const Eigen::Matrix2<Dtype> rotation_matrix =
            Eigen::Rotation2D<Dtype>(theta).toRotationMatrix();
        for (int i = 0; i < num_vertices; ++i) {
            Vector2 vertex = rotation_matrix * m_shape_vertices_.col(i) + position;
            int x = m_grid_map_info_->MeterToGridForValue(vertex[0], 0);  // row
            int y = m_grid_map_info_->MeterToGridForValue(vertex[1], 1);  // col
            robot_shape.emplace_back(y, x);                               // (col, row)
        }
        cv::drawContours(m_cleaned_mask_.free_mask, contour, 0, cv::Scalar(1), cv::FILLED);
        cv::drawContours(m_cleaned_mask_.occupied_mask, contour, 0, cv::Scalar(0), cv::FILLED);
        cv::drawContours(m_cleaned_mask_.unexplored_mask, contour, 0, cv::Scalar(0), cv::FILLED);
    }
}  // namespace erl::geometry
