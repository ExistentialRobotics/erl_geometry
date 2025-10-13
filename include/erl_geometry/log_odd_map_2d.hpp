#pragma once

#include "log_odd_map.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/yaml.hpp"

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

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };
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
        LogOddMap2D(std::shared_ptr<Setting> setting, std::shared_ptr<GridMapInfo> grid_map_info);

        LogOddMap2D(
            std::shared_ptr<Setting> setting,
            std::shared_ptr<GridMapInfo> grid_map_info,
            const Eigen::Ref<const Matrix2X> &robot_metric_contour);

        /**
         *
         * @param position Sensor position in world frame, the unit is meters.
         * @param theta Sensor yaw angle in world frame, the unit is radians.
         * @param points Scan points in world frame, the unit is meters.
         */
        void
        Update(const Eigen::Ref<const Vector2> &position, Dtype theta, Matrix2X points);

        /**
         * @brief Load the external possibility map where -1 means unexplored, 0 ~ 100 means
         * occupancy possibility, i.e., 0 means free, 100 means occupied.
         * @param position
         * @param theta
         * @param possibility_map
         */
        void
        LoadExternalPossibilityMap(
            const Eigen::Ref<const Vector2> &position,
            Dtype theta,
            const Eigen::Ref<const Eigen::MatrixXi> &possibility_map);

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const;

        [[nodiscard]] std::shared_ptr<const GridMapInfo>
        GetGridMapInfo() const;

        [[nodiscard]] cv::Mat
        GetLogMap() const;

        [[nodiscard]] cv::Mat
        GetPossibilityMap() const;

        [[nodiscard]] cv::Mat
        GetOccupancyMap() const;

        [[nodiscard]] cv::Mat
        GetUnexploredMask() const;

        [[nodiscard]] cv::Mat
        GetOccupiedMask() const;

        [[nodiscard]] cv::Mat
        GetFreeMask() const;

        [[nodiscard]] std::size_t
        GetNumUnexploredCells() const;

        [[nodiscard]] std::size_t
        GetNumOccupiedCells() const;

        [[nodiscard]] std::size_t
        GetNumFreeCells() const;

        [[nodiscard]] const LogOddCVMask &
        GetCleanedMasks() const;

        [[nodiscard]] cv::Mat
        GetCleanedFreeMask() const;

        [[nodiscard]] cv::Mat
        GetCleanedOccupiedMask() const;

        [[nodiscard]] cv::Mat
        GetCleanedUnexploredMask() const;

        [[nodiscard]] std::vector<Eigen::Matrix2Xi>
        GetFrontiers(bool clean_at_first = true, int approx_iters = 4) const;

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
            const std::shared_ptr<FrameMask> &old_mask) const;

        void
        PostProcessMasks(const Eigen::Ref<const Vector2> &position, Dtype theta);
    };

    using LogOddMap2Dd = LogOddMap2D<double>;
    using LogOddMap2Df = LogOddMap2D<float>;

    extern template class LogOddMap2D<double>;
    extern template class LogOddMap2D<float>;

}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::LogOddMap2Dd::Setting>
    : erl::geometry::LogOddMap2Dd::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::LogOddMap2Df::Setting>
    : erl::geometry::LogOddMap2Df::Setting::YamlConvertImpl {};
