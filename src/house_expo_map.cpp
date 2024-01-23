#include <open3d/core/Tensor.h>
#include <open3d/geometry/LineSet.h>
#include <open3d/t/geometry/LineSet.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/io/TriangleMeshIO.h>

#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/polygon_to_mesh.hpp"

namespace erl::geometry {

    HouseExpoMap::HouseExpoMap(const char *file)
        : m_file_(file) {
        std::cout << "Loading " << file << "..." << std::flush;
        std::ifstream ifs;
        ifs.open(file);
        if (!ifs.is_open()) { ERL_ASSERTM("Failed to open %s when current path is %s.", file, std::filesystem::current_path().c_str()); }
        nlohmann::json data = nlohmann::json::parse(ifs);
        FromJson(data, *this);
        ifs.close();
        std::cout << "Done." << std::endl;
    }

    HouseExpoMap::HouseExpoMap(const char *file, double wall_thickness)
        : HouseExpoMap(file) {
        ERL_ASSERTM(wall_thickness >= 0.1, "Wall thickness must be >= 0.1.");
        ERL_INFO("Changing wall thickness to %f ...", wall_thickness);
#if defined(NDEBUG)
        double free_threshold = (wall_thickness - 0.1) / 2;  // 0.1 is the default thickness of the wall
        if (std::abs(free_threshold) < 1.e-6) {
            std::cout << "Done." << std::endl << std::flush;
            return;
        }
        const auto &kSurface = m_meter_space_->GetSurface();
        auto &verts = kSurface->vertices;
        Eigen::Vector2d min = verts.rowwise().minCoeff();
        Eigen::Vector2d max = verts.rowwise().maxCoeff();
        Eigen::Vector2d resolution(0.01, 0.01);
        Eigen::Vector2i padding(10, 10);
        auto grid_map_info = common::GridMapInfo2D(min, max, resolution, padding);
        auto map_image = m_meter_space_->GenerateMapImage(grid_map_info);
        cv::Mat map_mat;
        cv::eigen2cv(map_image, map_mat);
        int ksize = int(free_threshold / resolution[0]) * 2 + 3;
        cv::erode(map_mat, map_mat, cv::getStructuringElement(cv::MORPH_RECT, {ksize, ksize}));
        cv::cv2eigen(map_mat, map_image);
        m_meter_space_ = std::make_shared<geometry::Space2D>(Eigen::MatrixXd(map_image.cast<double>()), grid_map_info, 0, true);  // y up, x right
        ERL_INFO("Done.");
#else
        std::cout << "Debug mode, no wall thickness change." << std::endl;
#endif
    }

    std::shared_ptr<open3d::geometry::TriangleMesh>
    HouseExpoMap::ExtrudeTo3D(double room_height) const {
        open3d::geometry::LineSet wall_line_set;
        std::vector<std::vector<Eigen::Vector2d>> polygon(1);  // only one polygon

        auto &surface_2d = m_meter_space_->GetSurface();
        long num_vertices = surface_2d->GetNumVertices();
        for (long i = 0; i < num_vertices; ++i) {
            auto vertex = surface_2d->vertices.col(i);
            wall_line_set.points_.emplace_back(vertex[0], vertex[1], 0.0);
            polygon[0].emplace_back(vertex[0], vertex[1]);
        }
        long num_lines = surface_2d->GetNumLines();
        for (long i = 0; i < num_lines; ++i) { wall_line_set.lines_.emplace_back(surface_2d->lines_to_vertices.col(i)); }

        // generate wall mesh
        auto wall_line_set_t = open3d::t::geometry::LineSet::FromLegacy(wall_line_set);
        open3d::core::Tensor z_dir(std::vector<double>{0.0, 0.0, room_height});
        auto wall_mesh = wall_line_set_t.ExtrudeLinear(z_dir).ToLegacy();
        wall_mesh.PaintUniformColor({0.5, 0.5, 0.5});  // gray
        wall_mesh.ComputeVertexNormals();

        // generate ground mesh and ceiling mesh
        auto ground_mesh = PolygonToMesh(polygon, 0.0, true);
        auto ceiling_mesh = PolygonToMesh(polygon, room_height, false);
        ground_mesh->PaintUniformColor({0.67, 0.33, 0.0});  // brown
        ceiling_mesh->PaintUniformColor({1.0, 1.0, 0.67});  // light yellow

        auto room_mesh = std::make_shared<open3d::geometry::TriangleMesh>(wall_mesh);
        *room_mesh += *ground_mesh;
        *room_mesh += *ceiling_mesh;

        return room_mesh;
    }

    void
    HouseExpoMap::ToJson(nlohmann::json &json_data, const erl::geometry::HouseExpoMap &map) {
        json_data["id"] = map.m_room_id_;
        json_data["bbox"] = nlohmann::json::object({
            {"min", Eigen::Vector2d(map.m_bbox_.row(0).transpose())},
            {"max", Eigen::Vector2d(map.m_bbox_.row(1).transpose())},
        });
        json_data["verts"] = map.m_meter_space_->GetSurface()->vertices;
    }

    void
    HouseExpoMap::FromJson(const nlohmann::json &json_data, erl::geometry::HouseExpoMap &map) {
        json_data.at("id").get_to(map.m_room_id_);
        double xmin, ymin, xmax, ymax;
        json_data.at("bbox").at("min").at(0).get_to(xmin);
        json_data.at("bbox").at("min").at(1).get_to(ymin);
        json_data.at("bbox").at("max").at(0).get_to(xmax);
        json_data.at("bbox").at("max").at(1).get_to(ymax);
        // clang-format off
            map.m_bbox_ << xmin, ymin,
                           xmax, ymax;
        // clang-format on

        auto verts = json_data.at("verts").get<Eigen::Matrix2Xd>();
        std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> ordered_vertices;
        ordered_vertices.emplace_back(verts);
        Eigen::Scalar<bool> outside_flags{false};
        map.m_meter_space_ = std::make_shared<geometry::Space2D>(ordered_vertices, outside_flags);
    }
}  // namespace erl::geometry
