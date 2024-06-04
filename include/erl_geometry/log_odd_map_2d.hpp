#pragma once
#include "erl_common/eigen.hpp"
#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace erl::geometry {

    class LogOddMap2D {

    public:
        enum CellType { kOccupied = 0, kFree = 255, kUnexplored = 128 };

        static const char *
        GetCellTypeName(const CellType type) {
            static const char *names[] = {"kOccupied", "kUnexplored", "kFree"};

            const int i = (static_cast<int>(type) + 1) / 128;
            return names[i];
        }

        static CellType
        GetCellTypeFromName(const std::string &name) {
            if (name == "kOccupied") { return kOccupied; }
            if (name == "kFree") { return kFree; }
            if (name == "kUnexplored") { return kUnexplored; }
            throw std::runtime_error("Unknown cell type: " + name);
        }

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
            double sensor_min_range = 0.01;
            double sensor_max_range = 30;
            double measurement_certainty = 0.9;
            double max_log_odd = 50;
            double min_log_odd = -8;
            double threshold_occupied = 0.7;
            double threshold_free = 0.3;
            bool use_cross_kernel = true;        // otherwise, use rect kernel. For 3x3, ellipse and cross are the same
            int num_iters_for_cleaned_mask = 4;  // number of iterations of dilate and erode to generate cleaned mask
            bool filter_obstacles_in_cleaned_mask = false;
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
        std::shared_ptr<common::GridMapInfo<2>> m_grid_map_info_ = nullptr;
        cv::Mat m_log_map_ = {};
        cv::Mat m_possibility_map_ = {};
        cv::Mat m_occupancy_map_ = {};
        cv::Mat m_kernel_ = {};
        LogOddCVMask m_mask_ = {};
        LogOddCVMask m_cleaned_mask_ = {};
        std::size_t m_num_unexplored_cells_ = -1;
        std::size_t m_num_occupied_cells_ = 0;
        std::size_t m_num_free_cells_ = 0;
        Eigen::Matrix2Xd m_shape_vertices_ = {};

    public:
        LogOddMap2D(std::shared_ptr<Setting> setting, std::shared_ptr<common::GridMapInfo<2>> grid_map_info);

        LogOddMap2D(
            std::shared_ptr<Setting> setting,
            std::shared_ptr<common::GridMapInfo2D> grid_map_info,
            const Eigen::Ref<const Eigen::Matrix2Xd> &shape_metric_vertices);

        void
        Update(
            const Eigen::Ref<const Eigen::Vector2d> &position,     // assumed in world frame, unit is meters
            double theta,                                          // assumed in world frame, unit is radian
            const Eigen::Ref<const Eigen::VectorXd> &angles_body,  // assumed in body frame, unit is radian
            const Eigen::Ref<const Eigen::VectorXd> &ranges);

        /**
         * @brief Load external possibility map where -1 means unexplored, 0 ~ 100 means occupancy possibility, i.e. 0 means free, 100 means occupied.
         * @param position
         * @param theta
         * @param possibility_map
         */
        void
        LoadExternalPossibilityMap(const Eigen::Ref<const Eigen::Vector2d> &position, double theta, const Eigen::Ref<const Eigen::MatrixXi> &possibility_map);

        std::shared_ptr<LidarFrameMask>
        ComputeStatisticsOfLidarFrame(
            const Eigen::Ref<const Eigen::Vector2d> &position,
            const double theta,
            const Eigen::Ref<const Eigen::VectorXd> &angles_body,
            const Eigen::Ref<const Eigen::VectorXd> &ranges,
            const bool clip_ranges,
            const std::shared_ptr<LidarFrameMask> &old_mask,
            int &num_occupied_cells,
            int &num_free_cells,
            int &num_unexplored_cells,
            int &num_out_of_map_cells) const {

            constexpr bool ray_mode = false;
            constexpr bool in_map_only = false;
            auto new_mask = ComputeLidarFrameMask(position, theta, angles_body, ranges, clip_ranges, ray_mode, in_map_only, old_mask);
            ComputeStatisticsOfLidarFrameMask(new_mask, num_occupied_cells, num_free_cells, num_unexplored_cells, num_out_of_map_cells);
            return new_mask;
        }

        std::shared_ptr<LidarFrameMask>
        ComputeStatisticsOfLidarFrames(
            const Eigen::Ref<const Eigen::Matrix3Xd> &lidar_poses,
            const Eigen::Ref<const Eigen::VectorXd> &lidar_angles_body,
            const std::vector<Eigen::VectorXd> &lidar_ranges,
            const bool clip_ranges,
            const std::shared_ptr<LidarFrameMask> &old_mask,
            int &num_occupied_cells,
            int &num_free_cells,
            int &num_unexplored_cells,
            int &num_out_of_map_cells) const {

            // const bool kRayMode = false;
            // const bool kInMapOnly = false;
            auto new_mask = ComputeLidarFramesMask(lidar_poses, lidar_angles_body, lidar_ranges, clip_ranges, old_mask);
            ComputeStatisticsOfLidarFrameMask(new_mask, num_occupied_cells, num_free_cells, num_unexplored_cells, num_out_of_map_cells);
            return new_mask;
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Eigen::MatrixXd
        GetLogMap() const {
            Eigen::MatrixXd log_map;
            cv::cv2eigen(m_log_map_, log_map);
            return log_map;
        }

        [[nodiscard]] Eigen::MatrixXd
        GetPossibilityMap() const {
            Eigen::MatrixXd possibility_map;
            cv::cv2eigen(m_possibility_map_, possibility_map);
            return possibility_map;
        }

        [[nodiscard]] Eigen::MatrixX8U
        GetOccupancyMap() const {
            Eigen::MatrixX8U occupancy_map;
            cv::cv2eigen(m_occupancy_map_, occupancy_map);
            return occupancy_map;
        }

        [[nodiscard]] Eigen::MatrixX8U
        GetUnexploredMask() const {
            Eigen::MatrixX8U unexplored_mask;
            cv::cv2eigen(m_mask_.unexplored_mask, unexplored_mask);
            return unexplored_mask;
        }

        [[nodiscard]] Eigen::MatrixX8U
        GetOccupiedMask() const {
            Eigen::MatrixX8U occupied_mask;
            cv::cv2eigen(m_mask_.occupied_mask, occupied_mask);
            return occupied_mask;
        }

        [[nodiscard]] Eigen::MatrixX8U
        GetFreeMask() const {
            Eigen::MatrixX8U free_mask;
            cv::cv2eigen(m_mask_.free_mask, free_mask);
            return free_mask;
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

        [[nodiscard]] Eigen::MatrixX8U
        GetCleanedFreeMask() const {
            Eigen::MatrixX8U free_mask;
            cv::cv2eigen(m_cleaned_mask_.free_mask, free_mask);
            return free_mask;
        }

        [[nodiscard]] Eigen::MatrixX8U
        GetCleanedOccupiedMask() const {
            Eigen::MatrixX8U occupied_mask;
            cv::cv2eigen(m_cleaned_mask_.occupied_mask, occupied_mask);
            return occupied_mask;
        }

        [[nodiscard]] Eigen::MatrixX8U
        GetCleanedUnexploredMask() const {
            Eigen::MatrixX8U unexplored_mask;
            cv::cv2eigen(m_cleaned_mask_.unexplored_mask, unexplored_mask);
            return unexplored_mask;
        }

        [[nodiscard]] std::vector<Eigen::Matrix2Xi>
        GetFrontiers(bool clean_at_first = true, int approx_iters = 4) const;

    private:
        [[nodiscard]] std::shared_ptr<LogOddMap2D::LidarFrameMask>
        ComputeLidarFrameMask(
            const Eigen::Ref<const Eigen::Vector2d> &position,
            double theta,
            const Eigen::Ref<const Eigen::VectorXd> &angles_body,
            const Eigen::Ref<const Eigen::VectorXd> &ranges,
            bool clip_ranges,
            bool ray_mode,  // true: ray mode, false: area mode
            bool in_map_only,
            const std::shared_ptr<LogOddMap2D::LidarFrameMask> &old_mask) const;

        [[nodiscard]] std::shared_ptr<LogOddMap2D::LidarFrameMask>
        ComputeLidarFramesMask(
            const Eigen::Ref<const Eigen::Matrix3Xd> &lidar_poses,
            const Eigen::Ref<const Eigen::VectorXd> &lidar_angles_body,
            const std::vector<Eigen::VectorXd> &lidar_ranges,
            bool clip_ranges,
            const std::shared_ptr<LidarFrameMask> &old_mask) const;

        void
        ComputeStatisticsOfLidarFrameMask(
            const std::shared_ptr<LogOddMap2D::LidarFrameMask> &mask,
            int &num_occupied_cells,
            int &num_free_cells,
            int &num_unexplored_cells,
            int &num_out_of_map_cells) const;

        void
        PostProcessMasks(const Eigen::Ref<const Eigen::Vector2d> &position, double theta);
    };

}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::LogOddMap2D::Setting> {
    static Node
    encode(const erl::geometry::LogOddMap2D::Setting &setting) {
        Node node;
        node["sensor_min_range"] = setting.sensor_min_range;
        node["sensor_max_range"] = setting.sensor_max_range;
        node["measurement_certainty"] = setting.measurement_certainty;
        node["max_log_odd"] = setting.max_log_odd;
        node["min_log_odd"] = setting.min_log_odd;
        node["threshold_occupied"] = setting.threshold_occupied;
        node["threshold_free"] = setting.threshold_free;
        node["use_cross_kernel"] = setting.use_cross_kernel;
        node["num_iters_for_cleaned_mask"] = setting.num_iters_for_cleaned_mask;
        node["filter_obstacles_in_cleaned_mask"] = setting.filter_obstacles_in_cleaned_mask;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::LogOddMap2D::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.sensor_min_range = node["sensor_min_range"].as<double>();
        setting.sensor_max_range = node["sensor_max_range"].as<double>();
        setting.measurement_certainty = node["measurement_certainty"].as<double>();
        setting.max_log_odd = node["max_log_odd"].as<double>();
        setting.min_log_odd = node["min_log_odd"].as<double>();
        setting.threshold_occupied = node["threshold_occupied"].as<double>();
        setting.threshold_free = node["threshold_free"].as<double>();
        setting.use_cross_kernel = node["use_cross_kernel"].as<bool>();
        setting.num_iters_for_cleaned_mask = node["num_iters_for_cleaned_mask"].as<int>();
        setting.filter_obstacles_in_cleaned_mask = node["filter_obstacles_in_cleaned_mask"].as<bool>();
        return true;
    }
};  // namespace YAML
