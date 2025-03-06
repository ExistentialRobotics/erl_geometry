#pragma once

#include "log_odd_map.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/logging.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    template<typename Dtype>
    class LogOddMap2D : public LogOddMap {

    public:
        using Vector2 = Eigen::Vector2<Dtype>;
        using VectorX = Eigen::VectorX<Dtype>;
        using Matrix3X = Eigen::Matrix3X<Dtype>;

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
            Dtype measurement_certainty = 0.9;
            Dtype max_log_odd = 50;
            Dtype min_log_odd = -8;
            Dtype threshold_occupied = 0.7;
            Dtype threshold_free = 0.3;
            bool use_cross_kernel = true;        // otherwise, use rect kernel. For 3x3, ellipse and cross are the same
            int num_iters_for_cleaned_mask = 4;  // number of iterations of dilate and erode to generate cleaned mask
            bool filter_obstacles_in_cleaned_mask = false;

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };
        };

        struct LidarFrameMask {
            cv::Mat mask;  // rows: x, cols: y. should use point(y, x) to draw contour
            int x_grid_min = std::numeric_limits<int>::max();
            int y_grid_min = std::numeric_limits<int>::max();
            int x_grid_max = -std::numeric_limits<int>::max();
            int y_grid_max = -std::numeric_limits<int>::max();
            Eigen::Matrix2Xi occupied_grids;

            LidarFrameMask() = default;

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
        std::shared_ptr<common::GridMapInfo2D<Dtype>> m_grid_map_info_ = nullptr;
        cv::Mat m_log_map_ = {};  // ij-indexing, x to the bottom, y to the right
        cv::Mat m_possibility_map_ = {};
        cv::Mat m_occupancy_map_ = {};
        cv::Mat m_kernel_ = {};
        LogOddCVMask m_mask_ = {};
        LogOddCVMask m_cleaned_mask_ = {};
        std::size_t m_num_unexplored_cells_ = -1;
        std::size_t m_num_occupied_cells_ = 0;
        std::size_t m_num_free_cells_ = 0;
        Eigen::Matrix2X<Dtype> m_shape_vertices_ = {};

    public:
        LogOddMap2D(std::shared_ptr<Setting> setting, std::shared_ptr<common::GridMapInfo2D<Dtype>> grid_map_info);

        LogOddMap2D(
            std::shared_ptr<Setting> setting,
            std::shared_ptr<common::GridMapInfo2D<Dtype>> grid_map_info,
            const Eigen::Ref<const Eigen::Matrix2X<Dtype>> &shape_metric_vertices);

        void
        Update(
            const Eigen::Ref<const Vector2> &position,     // assumed in world frame, unit is meters
            Dtype theta,                                   // assumed in world frame, unit is radian
            const Eigen::Ref<const VectorX> &angles_body,  // assumed in body frame, unit is radian
            const Eigen::Ref<const VectorX> &ranges);

        /**
         * @brief Load external possibility map where -1 means unexplored, 0 ~ 100 means occupancy possibility, i.e. 0 means free, 100 means occupied.
         * @param position
         * @param theta
         * @param possibility_map
         */
        void
        LoadExternalPossibilityMap(const Eigen::Ref<const Vector2> &position, Dtype theta, const Eigen::Ref<const Eigen::MatrixXi> &possibility_map);

        std::shared_ptr<LidarFrameMask>
        ComputeStatisticsOfLidarFrame(
            const Eigen::Ref<const Vector2> &position,
            Dtype theta,
            const Eigen::Ref<const VectorX> &angles_body,
            const Eigen::Ref<const VectorX> &ranges,
            bool clip_ranges,
            const std::shared_ptr<LidarFrameMask> &old_mask,
            int &num_occupied_cells,
            int &num_free_cells,
            int &num_unexplored_cells,
            int &num_out_of_map_cells) const;

        std::shared_ptr<LidarFrameMask>
        ComputeStatisticsOfLidarFrames(
            const Eigen::Ref<const Matrix3X> &lidar_poses,
            const Eigen::Ref<const VectorX> &lidar_angles_body,
            const std::vector<VectorX> &lidar_ranges,
            bool clip_ranges,
            const std::shared_ptr<LidarFrameMask> &old_mask,
            int &num_occupied_cells,
            int &num_free_cells,
            int &num_unexplored_cells,
            int &num_out_of_map_cells) const;

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const;

        [[nodiscard]] std::shared_ptr<const common::GridMapInfo2D<Dtype>>
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

        [[nodiscard]] auto
        GetFrontiers(bool clean_at_first = true, int approx_iters = 4) const -> std::vector<Eigen::Matrix2Xi>;

    private:
        [[nodiscard]] std::shared_ptr<LidarFrameMask>
        ComputeLidarFrameMask(
            const Eigen::Ref<const Vector2> &position,
            Dtype theta,
            const Eigen::Ref<const VectorX> &angles_body,
            const Eigen::Ref<const VectorX> &ranges,
            bool clip_ranges,
            bool ray_mode,  // true: ray mode, false: area mode
            bool in_map_only,
            const std::shared_ptr<LidarFrameMask> &old_mask) const;

        [[nodiscard]] std::shared_ptr<LidarFrameMask>
        ComputeLidarFramesMask(
            const Eigen::Ref<const Eigen::Matrix3Xd> &lidar_poses,
            const Eigen::Ref<const VectorX> &lidar_angles_body,
            const std::vector<VectorX> &lidar_ranges,
            bool clip_ranges,
            const std::shared_ptr<LidarFrameMask> &old_mask) const;

        void
        ComputeStatisticsOfLidarFrameMask(
            const std::shared_ptr<LidarFrameMask> &mask,
            int &num_occupied_cells,
            int &num_free_cells,
            int &num_unexplored_cells,
            int &num_out_of_map_cells) const;

        void
        PostProcessMasks(const Eigen::Ref<const Vector2> &position, Dtype theta);
    };

    using LogOddMap2Dd = LogOddMap2D<double>;
    using LogOddMap2Df = LogOddMap2D<float>;

}  // namespace erl::geometry

#include "log_odd_map_2d.tpp"

template<>
struct YAML::convert<erl::geometry::LogOddMap2Dd::Setting> : erl::geometry::LogOddMap2Dd::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::LogOddMap2Df::Setting> : erl::geometry::LogOddMap2Df::Setting::YamlConvertImpl {};
