#include "erl_common/pybind11.hpp"
#include "erl_geometry/log_odd_map_2d.hpp"

template<typename Dtype>
void
BindLogOddMap2DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = LogOddMap2D<Dtype>;

    auto py_log_odd_map = py::class_<T, std::shared_ptr<T>>(m, name);

    py::class_<typename T::Setting, YamlableBase, std::shared_ptr<typename T::Setting>>(py_log_odd_map, "Setting")
        .def(py::init<>())
        .def_readwrite("sensor_min_range", &T::Setting::sensor_min_range)
        .def_readwrite("sensor_max_range", &T::Setting::sensor_max_range)
        .def_readwrite("measurement_certainty", &T::Setting::measurement_certainty)
        .def_readwrite("max_log_odd", &T::Setting::max_log_odd)
        .def_readwrite("min_log_odd", &T::Setting::min_log_odd)
        .def_readwrite("threshold_occupied", &T::Setting::threshold_occupied)
        .def_readwrite("threshold_free", &T::Setting::threshold_free)
        .def_readwrite("use_cross_kernel", &T::Setting::use_cross_kernel)
        .def_readwrite("num_iters_for_cleaned_mask", &T::Setting::num_iters_for_cleaned_mask)
        .def_readwrite("filter_obstacles_in_cleaned_mask", &T::Setting::filter_obstacles_in_cleaned_mask);

    py_log_odd_map
        .def(py::init<std::shared_ptr<typename T::Setting>, std::shared_ptr<GridMapInfo<Dtype, 2>>>(), py::arg("setting").none(false), py::arg("grid_map_info"))
        .def(
            py::init<std::shared_ptr<typename T::Setting>, std::shared_ptr<GridMapInfo<Dtype, 2>>, const Eigen::Ref<const Eigen::Matrix2X<Dtype>> &>(),
            py::arg("setting").none(false),
            py::arg("grid_map_info"),
            py::arg("shape_vertices"))
        .def_static("get_cell_type_name", &T::GetCellTypeName, py::arg("cell_type"))
        .def_static("get_cell_type_from_name", &T::GetCellTypeFromName, py::arg("cell_type_name"))
        .def("update", &T::Update, py::arg("position"), py::arg("theta"), py::arg("angles_body"), py::arg("ranges"))
        .def("load_external_possibility_map", &T::LoadExternalPossibilityMap, py::arg("position"), py::arg("theta"), py::arg("possibility_map"))
        .def(
            "compute_statistics_of_lidar_frame",
            [](const T &self,
               const Eigen::Ref<const Eigen::Vector2<Dtype>> &position,
               const Dtype theta,
               const Eigen::Ref<const Eigen::VectorX<Dtype>> &angles_body,
               const Eigen::Ref<const Eigen::VectorX<Dtype>> &ranges,
               const bool clip_ranges) {
                const std::shared_ptr<typename T::LidarFrameMask> mask = nullptr;
                int num_occupied_cells;
                int num_free_cells;
                int num_unexplored_cells;
                int num_out_of_map_cells;
                self.ComputeStatisticsOfLidarFrame(
                    position,
                    theta,
                    angles_body,
                    ranges,
                    clip_ranges,
                    mask,
                    num_occupied_cells,
                    num_free_cells,
                    num_unexplored_cells,
                    num_out_of_map_cells);
                return std::make_tuple(num_occupied_cells, num_free_cells, num_unexplored_cells, num_out_of_map_cells);
            },
            py::arg("position"),
            py::arg("theta"),
            py::arg("angles_body"),
            py::arg("ranges"),
            py::arg("clip_ranges"))
        .def_property_readonly("setting", &T::GetSetting)
        .def_property_readonly(
            "log_map",
            [](const T &self) {
                const cv::Mat map = self.GetLogMap();
                Eigen::MatrixX<Dtype> log_map;
                cv::cv2eigen(map, log_map);
                return log_map;
            })
        .def_property_readonly(
            "possibility_map",
            [](const T &self) {
                const cv::Mat map = self.GetPossibilityMap();
                Eigen::MatrixX<Dtype> possibility_map;
                cv::cv2eigen(map, possibility_map);
                return possibility_map;
            })
        .def_property_readonly(
            "occupancy_map",
            [](const T &self) {
                const cv::Mat map = self.GetOccupancyMap();
                Eigen::MatrixX8U occupancy_map;
                cv::cv2eigen(map, occupancy_map);
                return occupancy_map;
            })
        .def_property_readonly(
            "unexplored_mask",
            [](const T &self) {
                const cv::Mat mask = self.GetUnexploredMask();
                Eigen::MatrixX8U unexplored_mask;
                cv::cv2eigen(mask, unexplored_mask);
                return unexplored_mask;
            })
        .def_property_readonly(
            "occupied_mask",
            [](const T &self) {
                const cv::Mat mask = self.GetOccupiedMask();
                Eigen::MatrixX8U occupied_mask;
                cv::cv2eigen(mask, occupied_mask);
                return occupied_mask;
            })
        .def_property_readonly(
            "free_mask",
            [](const T &self) {
                const cv::Mat mask = self.GetFreeMask();
                Eigen::MatrixX8U free_mask;
                cv::cv2eigen(mask, free_mask);
                return free_mask;
            })
        .def_property_readonly("num_unexplored_cells", &T::GetNumUnexploredCells)
        .def_property_readonly("num_occupied_cells", &T::GetNumOccupiedCells)
        .def_property_readonly("num_free_cells", &T::GetNumFreeCells)
        .def_property_readonly(
            "cleaned_free_mask",
            [](const T &self) {
                const cv::Mat mask = self.GetCleanedFreeMask();
                Eigen::MatrixX8U cleaned_free_mask;
                cv::cv2eigen(mask, cleaned_free_mask);
                return cleaned_free_mask;
            })
        .def_property_readonly(
            "cleaned_occupied_mask",
            [](const T &self) {
                const cv::Mat mask = self.GetCleanedOccupiedMask();
                Eigen::MatrixX8U cleaned_occupied_mask;
                cv::cv2eigen(mask, cleaned_occupied_mask);
                return cleaned_occupied_mask;
            })
        .def_property_readonly(
            "cleaned_unexplored_mask",
            [](const T &self) {
                const cv::Mat mask = self.GetCleanedUnexploredMask();
                Eigen::MatrixX8U cleaned_unexplored_mask;
                cv::cv2eigen(mask, cleaned_unexplored_mask);
                return cleaned_unexplored_mask;
            })
        .def("get_frontiers", &T::GetFrontiers, py::arg("clean_at_first") = true, py::arg("approx_iters") = 4);
}

void
BindLogOddMap2D(const py::module &m) {
    using namespace erl::geometry;
    py::class_<LogOddMap, std::shared_ptr<LogOddMap>> py_log_odd_map(m, "LogOddMap");
    py::enum_<LogOddMap::CellType>(py_log_odd_map, "CellType", py::arithmetic(), "Type of grid cell.")
        .value(LogOddMap::GetCellTypeName(LogOddMap::CellType::kOccupied), LogOddMap::CellType::kOccupied)
        .value(LogOddMap::GetCellTypeName(LogOddMap::CellType::kUnexplored), LogOddMap::CellType::kUnexplored)
        .value(LogOddMap::GetCellTypeName(LogOddMap::CellType::kFree), LogOddMap::CellType::kFree)
        .export_values();

    BindLogOddMap2DImpl<double>(m, "LogOddMap2Dd");
    BindLogOddMap2DImpl<float>(m, "LogOddMap2Df");
}
