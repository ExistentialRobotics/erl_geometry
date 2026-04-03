#pragma once

#include "aabb.hpp"
#include "log_odd_map.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/yaml.hpp"

#include <utility>

namespace erl::geometry {

    template<typename Dtype>
    class LogOddMap2D : public LogOddMap {

    public:
        using Vector2 = Eigen::Vector2<Dtype>;
        using VectorX = Eigen::VectorX<Dtype>;
        using Matrix2X = Eigen::Matrix2X<Dtype>;
        using Matrix3X = Eigen::Matrix3X<Dtype>;
        using GridMapInfo = common::GridMapInfo2D<Dtype>;

        struct LogOddCVMask {
            cv::Mat unexplored_mask;
            cv::Mat free_mask;
            cv::Mat occupied_mask;

            LogOddCVMask() = default;

            LogOddCVMask(const int height, const int width)
                : unexplored_mask(cv::Mat(height, width, CV_8UC1, cv::Scalar(1))),
                  free_mask(cv::Mat(height, width, CV_8UC1, cv::Scalar(0))),
                  occupied_mask(cv::Mat(height, width, CV_8UC1, cv::Scalar(0))) {}
        };

        struct Setting : public common::Yamlable<Setting> {
            Dtype sensor_min_range = 0.01;
            Dtype sensor_max_range = 30;
            Dtype log_odd_hit = 0.9;
            Dtype log_odd_miss = -0.2;
            Dtype max_log_odd = 50;
            Dtype min_log_odd = -8;
            Dtype threshold_occupied = 0.55f;
            Dtype threshold_free = 0.49f;

            // If true, use cross kernel. Otherwise, use rect kernel.
            // For 3x3, ellipse and cross are the same.
            bool use_cross_kernel = true;
            int kernel_size = 3;

            // number of iterations of dilation and erosion to generate cleaned mask
            int num_iters_for_cleaned_mask = 2;
            bool filter_obstacles_in_cleaned_mask = false;

            ERL_REFLECT_SCHEMA(
                Setting,
                ERL_REFLECT_MEMBER(Setting, sensor_min_range),
                ERL_REFLECT_MEMBER(Setting, sensor_max_range),
                ERL_REFLECT_MEMBER(Setting, log_odd_hit),
                ERL_REFLECT_MEMBER(Setting, log_odd_miss),
                ERL_REFLECT_MEMBER(Setting, max_log_odd),
                ERL_REFLECT_MEMBER(Setting, min_log_odd),
                ERL_REFLECT_MEMBER(Setting, threshold_occupied),
                ERL_REFLECT_MEMBER(Setting, threshold_free),
                ERL_REFLECT_MEMBER(Setting, use_cross_kernel),
                ERL_REFLECT_MEMBER(Setting, kernel_size),
                ERL_REFLECT_MEMBER(Setting, num_iters_for_cleaned_mask),
                ERL_REFLECT_MEMBER(Setting, filter_obstacles_in_cleaned_mask));
        };

        struct FrameMask {
            cv::Mat mask;  // rows: x, cols: y. should use point(y, x) to draw contour
            int x_grid_min = std::numeric_limits<int>::max();
            int y_grid_min = std::numeric_limits<int>::max();
            int x_grid_max = -std::numeric_limits<int>::max();
            int y_grid_max = -std::numeric_limits<int>::max();
            Eigen::Matrix2Xi occupied_grids;

            FrameMask() = default;

            void
            UpdateGridRange(const int x, const int y) {
                if (x < x_grid_min) { x_grid_min = x; }
                if (x > x_grid_max) { x_grid_max = x; }
                if (y < y_grid_min) { y_grid_min = y; }
                if (y > y_grid_max) { y_grid_max = y; }
            }
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<GridMapInfo> m_grid_map_info_ = nullptr;
        cv::Mat m_log_map_ = {};  // ij-indexing, x to the bottom, y to the right
        cv::Mat m_possibility_map_ = {};
        cv::Mat m_occupancy_map_ = {};
        cv::Mat m_kernel_ = {};
        LogOddCVMask m_mask_ = {};
        LogOddCVMask m_cleaned_mask_ = {};
        std::size_t m_num_unexplored_cells_ = -1;
        std::size_t m_num_occupied_cells_ = 0;
        std::size_t m_num_free_cells_ = 0;
        Matrix2X m_robot_metric_contour_ = {};

    public:
        LogOddMap2D(std::shared_ptr<Setting> setting, std::shared_ptr<GridMapInfo> grid_map_info)
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
                  cv::Scalar{static_cast<int>(CellType::kUnexplored)}),
              m_kernel_(
                  cv::getStructuringElement(
                      m_setting_->use_cross_kernel ? cv::MORPH_CROSS : cv::MORPH_RECT,
                      cv::Size{m_setting_->kernel_size, m_setting_->kernel_size})),
              m_mask_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1)),
              m_cleaned_mask_(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1)),
              m_num_unexplored_cells_(m_grid_map_info_->Shape(0) * m_grid_map_info_->Shape(1)) {}

        LogOddMap2D(
            std::shared_ptr<Setting> setting,
            std::shared_ptr<GridMapInfo> grid_map_info,
            const Eigen::Ref<const Matrix2X> &robot_metric_contour)
            : LogOddMap2D(std::move(setting), std::move(grid_map_info)) {
            m_robot_metric_contour_ = robot_metric_contour;
        }

        /**
         *
         * @param position Sensor position in world frame, the unit is meters.
         * @param theta Sensor yaw angle in world frame, the unit is radians.
         * @param points Scan points in world frame, the unit is meters.
         */
        void
        Update(const Eigen::Ref<const Vector2> &position, Dtype theta, Matrix2X points) {

            // generate mask of lidar scan
            constexpr bool clip_ranges = true;
            constexpr bool ray_mode = true;
            constexpr bool in_map_only = true;
            const auto mask = ComputeFrameMask(
                position,
                std::move(points),
                clip_ranges,
                ray_mode,
                in_map_only,
                nullptr);
            if (mask->mask.rows == 0 || mask->mask.cols == 0) { return; }

            // compute parameters
            const Dtype log_odd_hit = m_setting_->log_odd_hit;
            const Dtype log_odd_miss = m_setting_->log_odd_miss;

            // update log_odd_map, possibility_map, occupancy_map
            for (int row = 0; row < mask->mask.rows; ++row) {
                for (int col = 0; col < mask->mask.cols; ++col) {
                    const auto &mask_value = mask->mask.template at<uint8_t>(row, col);
                    if (mask_value == static_cast<int>(CellType::kUnexplored)) { continue; }

                    const int x = mask->x_grid_min + row;
                    const int y = mask->y_grid_min + col;
                    auto &log_odd_value = m_log_map_.at<Dtype>(x, y);
                    auto &possibility_value = m_possibility_map_.at<Dtype>(x, y);
                    auto &occupancy_value = m_occupancy_map_.at<uint8_t>(x, y);
                    auto &free_mask_value = m_mask_.free_mask.template at<uint8_t>(x, y);
                    auto &occupied_mask_value = m_mask_.occupied_mask.template at<uint8_t>(x, y);
                    auto &unexplored_mask_value =
                        m_mask_.unexplored_mask.template at<uint8_t>(x, y);

                    if (mask_value == static_cast<int>(CellType::kOccupied)) {
                        log_odd_value += log_odd_hit;
                    } else {
                        log_odd_value += log_odd_miss;
                    }
                    if (log_odd_value < m_setting_->min_log_odd) {
                        log_odd_value = m_setting_->min_log_odd;
                    } else if (log_odd_value > m_setting_->max_log_odd) {
                        log_odd_value = m_setting_->max_log_odd;
                    }

                    possibility_value = 1.0f / (1.0f + std::exp(-log_odd_value));

                    if (possibility_value > m_setting_->threshold_occupied) {
                        if (occupancy_value == static_cast<int>(CellType::kFree)) {
                            // kFree -> kOccupied
                            --m_num_free_cells_;
                            ++m_num_occupied_cells_;
                            free_mask_value = 0;
                        } else if (occupancy_value == static_cast<int>(CellType::kUnexplored)) {
                            // kUnexplored -> kOccupied
                            --m_num_unexplored_cells_;
                            ++m_num_occupied_cells_;
                            unexplored_mask_value = 0;
                        }
                        occupancy_value = static_cast<int>(CellType::kOccupied);
                        occupied_mask_value = 1;
                    } else if (possibility_value < m_setting_->threshold_free) {
                        if (occupancy_value == static_cast<int>(CellType::kOccupied)) {
                            // kOccupied -> kFree
                            --m_num_occupied_cells_;
                            ++m_num_free_cells_;
                            occupied_mask_value = 0;
                        } else if (occupancy_value == static_cast<int>(CellType::kUnexplored)) {
                            // kUnexplored -> kFree
                            --m_num_unexplored_cells_;
                            ++m_num_free_cells_;
                            unexplored_mask_value = 0;
                        }
                        occupancy_value = static_cast<int>(CellType::kFree);
                        free_mask_value = 1;
                    }
                }
            }

            PostProcessMasks(position, theta);
        }

        /**
         * @brief Update all cells within an axis-aligned bounding box with a specified log-odd
         * value. The log-odd value is added to the existing log-odd value of each cell. The
         * possibility map, occupancy map, masks, and cell counts are updated accordingly.
         * @param aabb The axis-aligned bounding box in world frame (meters).
         * @param log_odd The log-odd value to add to each cell within the AABB.
         * @note PostProcessMasks() is not called in this function since PostProcessMasks() is
         * designed for robot footprint cleaning with sensor observation, which is not the case for
         * this function.
         */
        void
        UpdateAabb(const Aabb<Dtype, 2> &aabb, Dtype log_odd) {
            // convert AABB from metric to grid coordinates and clamp to map bounds
            int x_min = m_grid_map_info_->MeterToGridAtDim(aabb.min()[0], 0);
            int y_min = m_grid_map_info_->MeterToGridAtDim(aabb.min()[1], 1);
            int x_max = m_grid_map_info_->MeterToGridAtDim(aabb.max()[0], 0);
            int y_max = m_grid_map_info_->MeterToGridAtDim(aabb.max()[1], 1);

            x_min = std::max(x_min, 0);
            y_min = std::max(y_min, 0);
            x_max = std::min(x_max, static_cast<int>(m_grid_map_info_->Shape(0)) - 1);
            y_max = std::min(y_max, static_cast<int>(m_grid_map_info_->Shape(1)) - 1);

            if (x_min > x_max || y_min > y_max) { return; }

            for (int x = x_min; x <= x_max; ++x) {
                for (int y = y_min; y <= y_max; ++y) {
                    auto &log_odd_value = m_log_map_.at<Dtype>(x, y);
                    auto &possibility_value = m_possibility_map_.at<Dtype>(x, y);
                    auto &occupancy_value = m_occupancy_map_.at<uint8_t>(x, y);
                    auto &free_mask_value = m_mask_.free_mask.template at<uint8_t>(x, y);
                    auto &occupied_mask_value = m_mask_.occupied_mask.template at<uint8_t>(x, y);
                    auto &unexplored_mask_value =
                        m_mask_.unexplored_mask.template at<uint8_t>(x, y);

                    log_odd_value += log_odd;
                    if (log_odd_value < m_setting_->min_log_odd) {
                        log_odd_value = m_setting_->min_log_odd;
                    } else if (log_odd_value > m_setting_->max_log_odd) {
                        log_odd_value = m_setting_->max_log_odd;
                    }

                    possibility_value = 1.0f / (1.0f + std::exp(-log_odd_value));

                    if (possibility_value > m_setting_->threshold_occupied) {
                        if (occupancy_value == static_cast<int>(CellType::kFree)) {
                            --m_num_free_cells_;
                            ++m_num_occupied_cells_;
                            free_mask_value = 0;
                        } else if (occupancy_value == static_cast<int>(CellType::kUnexplored)) {
                            --m_num_unexplored_cells_;
                            ++m_num_occupied_cells_;
                            unexplored_mask_value = 0;
                        }
                        occupancy_value = static_cast<int>(CellType::kOccupied);
                        occupied_mask_value = 1;
                    } else if (possibility_value < m_setting_->threshold_free) {
                        if (occupancy_value == static_cast<int>(CellType::kOccupied)) {
                            --m_num_occupied_cells_;
                            ++m_num_free_cells_;
                            occupied_mask_value = 0;
                        } else if (occupancy_value == static_cast<int>(CellType::kUnexplored)) {
                            --m_num_unexplored_cells_;
                            ++m_num_free_cells_;
                            unexplored_mask_value = 0;
                        }
                        occupancy_value = static_cast<int>(CellType::kFree);
                        free_mask_value = 1;
                    }
                }
            }
        }

        /**
         * @brief Load the external possibility map where -1 means unexplored, 0 ~ 100 means
         * occupancy possibility, i.e., 0 means free, 100 means occupied.
         * @param position Sensor position in world frame, the unit is meters.
         * @param theta Sensor yaw angle in world frame, the unit is radians.
         * @param possibility_map The external possibility map.
         * @param grid_map_info The grid map info of the external possibility map. If nullptr, the
         * provided possibility_map should have the same shape as the internal map.
         */
        template<typename ExtDtype>
        void
        LoadExternalPossibilityMap(
            const Eigen::Ref<const Vector2> &position,
            Dtype theta,
            const Eigen::Map<const Eigen::MatrixX<ExtDtype>> &possibility_map,
            const std::shared_ptr<GridMapInfo> &grid_map_info) {

            if (grid_map_info == nullptr) {
                ERL_ASSERTM(
                    possibility_map.rows() == m_grid_map_info_->Shape(0) &&
                        possibility_map.cols() == m_grid_map_info_->Shape(1),
                    "External log odd map has wrong shape. Expected: ({}, {}), Actual: ({}, {})",
                    m_grid_map_info_->Shape(0),
                    m_grid_map_info_->Shape(1),
                    possibility_map.rows(),
                    possibility_map.cols());
            } else if (*m_grid_map_info_ != *grid_map_info) {
                // reinitialize the internal map
                *m_grid_map_info_ = *grid_map_info;
                m_log_map_ = cv::Mat(
                    m_grid_map_info_->Shape(0),
                    m_grid_map_info_->Shape(1),
                    sizeof(Dtype) == 8 ? CV_64FC1 : CV_32FC1,
                    cv::Scalar{0});
                m_possibility_map_ = cv::Mat(
                    m_grid_map_info_->Shape(0),
                    m_grid_map_info_->Shape(1),
                    sizeof(Dtype) == 8 ? CV_64FC1 : CV_32FC1,
                    cv::Scalar{0.5});
                m_occupancy_map_ = cv::Mat(
                    m_grid_map_info_->Shape(0),
                    m_grid_map_info_->Shape(1),
                    CV_8UC1,
                    cv::Scalar{static_cast<int>(CellType::kUnexplored)});
                m_mask_ = LogOddCVMask(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1));
                m_cleaned_mask_ =
                    LogOddCVMask(m_grid_map_info_->Shape(0), m_grid_map_info_->Shape(1));
                m_num_unexplored_cells_ = m_grid_map_info_->Shape(0) * m_grid_map_info_->Shape(1);
            }

            const auto n_rows = static_cast<int>(possibility_map.rows());
            const auto n_cols = static_cast<int>(possibility_map.cols());

            std::vector<std::size_t> num_unexplored_cells(n_cols, 0);
            std::vector<std::size_t> num_occupied_cells(n_cols, 0);
            std::vector<std::size_t> num_free_cells(n_cols, 0);

#pragma omp parallel for default(none) \
    shared(n_rows,                     \
               n_cols,                 \
               num_unexplored_cells,   \
               num_occupied_cells,     \
               num_free_cells,         \
               possibility_map)
            for (int j = 0; j < n_cols; ++j) {

                std::size_t &n_unexplored = num_unexplored_cells[j];
                std::size_t &n_occupied = num_occupied_cells[j];
                std::size_t &n_free = num_free_cells[j];

                auto p_col = possibility_map.col(j);

                for (int i = 0; i < n_rows; ++i) {

                    auto &log_odd_value = m_log_map_.at<Dtype>(i, j);
                    auto &possibility_value = m_possibility_map_.at<Dtype>(i, j);
                    auto &occupancy_value = m_occupancy_map_.at<uint8_t>(i, j);
                    auto &free_mask_value = m_mask_.free_mask.template at<uint8_t>(i, j);
                    auto &occupied_mask_value = m_mask_.occupied_mask.template at<uint8_t>(i, j);
                    auto &unexplored_mask_value =
                        m_mask_.unexplored_mask.template at<uint8_t>(i, j);

                    if (p_col[i] == -1) {
                        log_odd_value = 0.0f;
                        possibility_value = 0.5f;
                        occupancy_value = static_cast<int>(CellType::kUnexplored);
                        free_mask_value = 0;
                        occupied_mask_value = 0;
                        unexplored_mask_value = 1;
                        ++n_unexplored;
                    } else {
                        possibility_value = static_cast<Dtype>(p_col[i]) / 100.0f;
                        log_odd_value = std::log(possibility_value / (1.0f - possibility_value));
                        if (possibility_value > m_setting_->threshold_occupied) {
                            occupancy_value = static_cast<int>(CellType::kOccupied);
                            free_mask_value = 0;
                            occupied_mask_value = 1;
                            unexplored_mask_value = 0;
                            ++n_occupied;
                        } else if (possibility_value < m_setting_->threshold_free) {
                            occupancy_value = static_cast<int>(CellType::kFree);
                            free_mask_value = 1;
                            occupied_mask_value = 0;
                            unexplored_mask_value = 0;
                            ++n_free;
                        } else {
                            occupancy_value = static_cast<int>(CellType::kUnexplored);
                            free_mask_value = 0;
                            occupied_mask_value = 0;
                            unexplored_mask_value = 1;
                            ++n_unexplored;
                        }
                    }
                }
            }

            m_num_unexplored_cells_ = 0;
            m_num_occupied_cells_ = 0;
            m_num_free_cells_ = 0;
            for (int i = 0; i < n_cols; ++i) {
                m_num_unexplored_cells_ += num_unexplored_cells[i];
                m_num_occupied_cells_ += num_occupied_cells[i];
                m_num_free_cells_ += num_free_cells[i];
            }

            PostProcessMasks(position, theta);
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] std::shared_ptr<const GridMapInfo>
        GetGridMapInfo() const {
            return m_grid_map_info_;
        }

        [[nodiscard]] cv::Mat
        GetLogMap() const {
            return m_log_map_;
        }

        [[nodiscard]] cv::Mat
        GetPossibilityMap() const {
            return m_possibility_map_;
        }

        [[nodiscard]] cv::Mat
        GetOccupancyMap() const {
            return m_occupancy_map_;
        }

        [[nodiscard]] cv::Mat
        GetUnexploredMask() const {
            return m_mask_.unexplored_mask;
        }

        [[nodiscard]] cv::Mat
        GetOccupiedMask() const {
            return m_mask_.occupied_mask;
        }

        [[nodiscard]] cv::Mat
        GetFreeMask() const {
            return m_mask_.free_mask;
        }

        [[nodiscard]] std::size_t
        GetNumUnexploredCells() const {
            return m_num_unexplored_cells_;
        }

        [[nodiscard]] std::size_t
        GetNumOccupiedCells() const {
            return m_num_occupied_cells_;
        }

        [[nodiscard]] std::size_t
        GetNumFreeCells() const {
            return m_num_free_cells_;
        }

        [[nodiscard]] const LogOddCVMask &
        GetCleanedMasks() const {
            return m_cleaned_mask_;
        }

        [[nodiscard]] cv::Mat
        GetCleanedFreeMask() const {
            return m_cleaned_mask_.free_mask;
        }

        [[nodiscard]] cv::Mat
        GetCleanedOccupiedMask() const {
            return m_cleaned_mask_.occupied_mask;
        }

        [[nodiscard]] cv::Mat
        GetCleanedUnexploredMask() const {
            return m_cleaned_mask_.unexplored_mask;
        }

        [[nodiscard]] std::vector<Eigen::Matrix2Xi>
        GetFrontiers(bool clean_at_first = true, int approx_iters = 4) const {
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

            // convert to 32-bit signed int due to the computing frontier_mask involves negative
            // values
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
                cv::erode(
                    tmp_frontier_mask,
                    frontier_mask,
                    m_kernel_,
                    cv::Point(-1, -1),
                    approx_iters);
            }

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            // retrieves all the contours without establishing any hierarchical relationships.
            // cv::CHAIN_APPROX_NONE: maintain all contour vertices!
            cv::findContours(
                frontier_mask,
                contours,
                hierarchy,
                cv::RETR_LIST,
                cv::CHAIN_APPROX_NONE);

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

    private:
        /**
         *
         * @param position sensor position in world frame, the unit is meters.
         * @param points scan points in world frame, the unit is meters.
         * @param clip_ranges if true, clip the rays that exceed the max range to the max range.
         * @param ray_mode if true, use ray mode, otherwise use area mode.
         * @param in_map_only if true, only compute the in-map area.
         * @param old_mask if not nullptr, keep the overlapped area of old_mask and the new mask.
         * @return
         */
        [[nodiscard]] std::shared_ptr<FrameMask>
        ComputeFrameMask(
            const Eigen::Ref<const Vector2> &position,
            Eigen::Matrix2X<Dtype> points,
            bool clip_ranges,
            bool ray_mode,
            bool in_map_only,
            const std::shared_ptr<FrameMask> &old_mask) const {

            if (points.cols() == 0) { return nullptr; }

            auto mask = std::make_shared<FrameMask>();

            // clip the ranges if necessary, check if the ray hits an obstacle.
            const long num_rays = points.cols();
            mask->occupied_grids.resize(2, num_rays);
            long num_obstacle_grids = 0;
            for (int i = 0; i < num_rays; ++i) {
                auto &&p = points.col(i);
                Dtype range = (p - position).norm();
                if (std::isinf(range)) { continue; }
                if (range >= m_setting_->sensor_max_range) {
                    if (clip_ranges) {
                        p = position + (p - position) * (m_setting_->sensor_max_range / range);
                    }
                } else if (range < m_setting_->sensor_min_range) {
                    p = position;
                } else {  // the ray hit an obstacle
                    mask->occupied_grids.col(num_obstacle_grids++)
                        << m_grid_map_info_->MeterToGridAtDim(p[0], 0),
                        m_grid_map_info_->MeterToGridAtDim(p[1], 1);
                }
            }
            mask->occupied_grids.conservativeResize(2, num_obstacle_grids);

            // compute the boundary of the in-map lidar scan area.
            // vector of points (x, y), in OpenCV, (x, y) = (col, row).
            // So, mask.x is point.y, mask.y is point.x.
            std::vector<std::vector<cv::Point>> area_contours;
            int start_x = m_grid_map_info_->MeterToGridAtDim(position[0], 0);
            int start_y = m_grid_map_info_->MeterToGridAtDim(position[1], 1);
            if (ray_mode) {
                area_contours.reserve(num_rays);
                for (long i = 0; i < num_rays; ++i) {
                    auto p = points.col(i);
                    int x = m_grid_map_info_->MeterToGridAtDim(p[0], 0);
                    int y = m_grid_map_info_->MeterToGridAtDim(p[1], 1);
                    mask->UpdateGridRange(x, y);
                    area_contours.emplace_back(
                        std::vector<cv::Point>{cv::Point(start_y, start_x), cv::Point(y, x)});
                }
            } else {
                area_contours.resize(1);
                auto &contour = area_contours.back();
                contour.reserve(num_rays + 1);
                contour.emplace_back(start_y, start_x);  // (col, row)
                for (long i = 0; i < num_rays; ++i) {
                    auto p = points.col(i);
                    int x = m_grid_map_info_->MeterToGridAtDim(p[0], 0);
                    int y = m_grid_map_info_->MeterToGridAtDim(p[1], 1);
                    mask->UpdateGridRange(x, y);
                    contour.emplace_back(y, x);
                }
                contour.emplace_back(start_y, start_x);
            }

            // adjust the boundary to include the old mask if provided, or to be within the map
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

            // allocate the mask
            const int n_rows = mask->x_grid_max - mask->x_grid_min + 1;
            const int n_cols = mask->y_grid_max - mask->y_grid_min + 1;
            ERL_DEBUG_ASSERT(n_rows >= 0 && n_cols >= 0, "n_rows: {}, n_cols: {}", n_rows, n_cols);
            if (n_rows == 0 || n_cols == 0) {
                if (old_mask == nullptr) { return mask; }
                return mask;
            }
            mask->mask = cv::Mat(
                n_rows,
                n_cols,
                CV_8UC1,
                cv::Scalar(static_cast<int>(CellType::kUnexplored)));

            // copy the old mask if provided
            if (old_mask != nullptr && old_mask->mask.rows > 0 && old_mask->mask.cols > 0) {
                old_mask->mask.copyTo(mask->mask(
                    cv::Rect(
                        old_mask->y_grid_min - mask->y_grid_min,  // col
                        old_mask->x_grid_min - mask->x_grid_min,  // row
                        old_mask->mask.cols,
                        old_mask->mask.rows)));
            }

            // draw the free grids
            for (auto &contour: area_contours) {
                for (auto &point: contour) {
                    point.x -= mask->y_grid_min;
                    point.y -= mask->x_grid_min;
                }
            }
            if (ray_mode) {
                cv::polylines(
                    mask->mask,
                    area_contours,
                    false,
                    static_cast<int>(CellType::kFree),
                    1,
                    cv::LINE_8);
            } else {
                cv::drawContours(
                    mask->mask,
                    area_contours,
                    0,
                    static_cast<int>(CellType::kFree),
                    cv::FILLED,
                    cv::LINE_8);
            }

            // draw the occupied grids
            for (int i = 0; i < num_obstacle_grids; ++i) {
                auto p = mask->occupied_grids.col(i);
                const int x = p[0] - mask->x_grid_min;
                if (const int y = p[1] - mask->y_grid_min;
                    x >= 0 && x < n_rows && y >= 0 && y < n_cols) {
                    mask->mask.template at<uint8_t>(x, y) = static_cast<int>(CellType::kOccupied);
                }
            }

            return mask;
        }

        void
        PostProcessMasks(const Eigen::Ref<const Vector2> &position, Dtype theta) {
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
            const long num_vertices = m_robot_metric_contour_.cols();
            if (num_vertices == 0) { return; }
            std::vector<std::vector<cv::Point>> contour(1);
            auto &robot_shape = contour[0];
            robot_shape.reserve(num_vertices);
            const Eigen::Matrix2<Dtype> rotation_matrix =
                Eigen::Rotation2D<Dtype>(theta).toRotationMatrix();
            for (int i = 0; i < num_vertices; ++i) {
                Vector2 vertex = rotation_matrix * m_robot_metric_contour_.col(i) + position;
                int x = m_grid_map_info_->MeterToGridAtDim(vertex[0], 0);  // row
                int y = m_grid_map_info_->MeterToGridAtDim(vertex[1], 1);  // col
                robot_shape.emplace_back(y, x);                            // (col, row)
            }
            cv::drawContours(m_cleaned_mask_.free_mask, contour, 0, cv::Scalar(1), cv::FILLED);
            cv::drawContours(m_cleaned_mask_.occupied_mask, contour, 0, cv::Scalar(0), cv::FILLED);
            cv::drawContours(
                m_cleaned_mask_.unexplored_mask,
                contour,
                0,
                cv::Scalar(0),
                cv::FILLED);
        }
    };

    using LogOddMap2Dd = LogOddMap2D<double>;
    using LogOddMap2Df = LogOddMap2D<float>;

    extern template class LogOddMap2D<double>;
    extern template class LogOddMap2D<float>;

}  // namespace erl::geometry
