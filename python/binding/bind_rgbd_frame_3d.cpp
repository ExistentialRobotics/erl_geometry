#include "erl_common/pybind11.hpp"
#include "erl_geometry/rgbd_frame_3d.hpp"

template<typename Dtype>
void
BindRgbdFrame3DImpl(const py::module &m, const char *name) {
    using namespace erl::common;
    using namespace erl::geometry;
    using T = RgbdFrame3D<Dtype>;

    py::class_<T, DepthFrame3D<Dtype>, std::shared_ptr<T>>(m, name)
        .def(py::init<std::shared_ptr<typename T::Setting>>(), py::arg("setting").none(false))
        .def("update_rgbd", &T::UpdateRgbd, py::arg("rotation"), py::arg("translation"), py::arg("depth"), py::arg("rgb"), py::arg("partition_rays") = false)
        .def(
            "convert_to_point_cloud",
            [](const T &self, const bool in_world_frame) {
                std::vector<Eigen::Vector3<Dtype>> points, colors;
                self.ConvertToPointCloud(in_world_frame, points, colors);
                py::dict out;
                out["points"] = points;
                out["colors"] = colors;
                return out;
            },
            py::arg("in_world_frame"));
}

void
BindRgbdFrame3D(const py::module &m) {
    BindRgbdFrame3DImpl<double>(m, "RgbdFrame3Dd");
    BindRgbdFrame3DImpl<float>(m, "RgbdFrame3Df");
}
