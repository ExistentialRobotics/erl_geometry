#include "erl_geometry/house_expo_map.hpp"

#include "erl_common/plplot_fig.hpp"
#include "erl_geometry/polygon_to_mesh.hpp"
#include "erl_geometry/winding_number.hpp"

#include <open3d/core/Tensor.h>
#include <open3d/geometry/LineSet.h>

namespace erl::geometry {

    HouseExpoMap::HouseExpoMap(std::string file)
        : m_file_(std::move(file)) {
        ERL_INFO("Loading {}...", m_file_);
        std::ifstream ifs;
        ifs.open(m_file_);
        if (!ifs.is_open()) {
            ERL_ASSERTM(
                "Failed to open {} when current path is {}.",
                m_file_,
                std::filesystem::current_path());
        }
        nlohmann::json data = nlohmann::json::parse(ifs);
        FromJson(data, *this);
        ifs.close();
        ERL_INFO("Done.");
    }

    HouseExpoMap::HouseExpoMap(const std::string &file, double wall_thickness)
        : HouseExpoMap(file) {
        ERL_ASSERTM(wall_thickness >= 0.1f, "Wall thickness must be >= 0.1.");
        ERL_INFO("Changing wall thickness to {:f} ...", wall_thickness);
#if defined(NDEBUG)
        double free_threshold = (wall_thickness - 0.1f) / 2;
        if (std::abs(free_threshold) < 1.e-6) {
            ERL_INFO("Done.");
            return;
        }
        const auto &kSurface = m_meter_space_->GetSurface();
        auto &verts = kSurface->vertices;
        Eigen::Vector2d min = verts.rowwise().minCoeff();
        Eigen::Vector2d max = verts.rowwise().maxCoeff();
        Eigen::Vector2d resolution(0.01, 0.01);
        Eigen::Vector2i padding(10, 10);
        auto grid_map_info = common::GridMapInfo2Dd(min, max, resolution, padding);
        auto map_image = m_meter_space_->GenerateMapImage(grid_map_info);
        cv::Mat map_mat;
        cv::eigen2cv(map_image, map_mat);
        const auto ksize = static_cast<int>(free_threshold / resolution[0]) * 2 + 3;
        cv::erode(map_mat, map_mat, cv::getStructuringElement(cv::MORPH_RECT, {ksize, ksize}));
        cv::cv2eigen(map_mat, map_image);
        // y up, x right
        m_meter_space_ = std::make_shared<Space2D>(
            Eigen::MatrixXd(map_image.cast<double>()),
            grid_map_info,
            0,
            true);
        ERL_INFO("Done.");
#else
        ERL_INFO("Debug mode, no wall thickness change.");
#endif
    }

    std::shared_ptr<open3d::geometry::TriangleMesh>
    HouseExpoMap::ExtrudeTo3D(const double room_height) const {
        open3d::geometry::LineSet wall_line_set;
        std::vector<std::vector<Eigen::Vector2d>> polygon(1);  // only one polygon

        auto &surface_2d = m_meter_space_->GetSurface();
        const long num_vertices = surface_2d->GetNumVertices();
        for (long i = 0; i < num_vertices; ++i) {
            auto vertex = surface_2d->vertices.col(i);
            wall_line_set.points_.emplace_back(vertex[0], vertex[1], 0.0f);
            polygon[0].emplace_back(vertex[0], vertex[1]);
        }
        const long num_lines = surface_2d->GetNumLines();
        for (long i = 0; i < num_lines; ++i) {
            wall_line_set.lines_.emplace_back(surface_2d->lines_to_vertices.col(i));
        }

        // generate ground mesh
        const std::shared_ptr<open3d::geometry::TriangleMesh> ground_mesh =
            PolygonToMesh(polygon, 0.0, true);

        auto room_mesh = std::make_shared<open3d::geometry::TriangleMesh>(*ground_mesh);
        room_mesh->PaintUniformColor({0.67, 0.33, 0.0});  // brown
        for (auto &vertex: ground_mesh->vertices_) {
            room_mesh->vertices_.emplace_back(vertex[0], vertex[1], room_height);
            room_mesh->vertex_colors_.emplace_back(1.0, 1.0, 0.67);  // light yellow
        }

        for (long i = 0; i < num_lines; ++i) {
            const long i0 = surface_2d->lines_to_vertices(0, i);
            const long i1 = surface_2d->lines_to_vertices(1, i);
            const long j0 = i0 + num_vertices;
            const long j1 = i1 + num_vertices;

            // check if the line is clockwise relative to the polygon
            Eigen::Vector3d v01 = ground_mesh->vertices_[i1] - ground_mesh->vertices_[i0];
            Eigen::Vector2d p(v01[1], -v01[0]);
            p /= p.norm() * 10;
            p[0] += (room_mesh->vertices_[i0][0] + room_mesh->vertices_[i1][0]) / 2;
            p[1] += (room_mesh->vertices_[i0][1] + room_mesh->vertices_[i1][1]) / 2;

            if (WindingNumber<double>(p, surface_2d->vertices)) {  // inside the polygon
                // the line is clockwise relative to the polygon
                room_mesh->triangles_.emplace_back(i0, i1, j0);
                room_mesh->triangles_.emplace_back(i1, j1, j0);
            } else {
                room_mesh->triangles_.emplace_back(i0, j0, i1);
                room_mesh->triangles_.emplace_back(i1, j0, j1);
            }
        }

        for (auto &triangle: ground_mesh->triangles_) {
            room_mesh->triangles_.emplace_back(
                triangle[0] + num_vertices,
                triangle[2] + num_vertices,
                triangle[1] + num_vertices);
        }

        return room_mesh;
    }

    void
    HouseExpoMap::ToJson(nlohmann::json &json_data, const HouseExpoMap &map) {
        json_data["id"] = map.m_room_id_;
        json_data["bbox"] = nlohmann::json::object({
            {"min", Eigen::Vector2d(map.m_bbox_.row(0).transpose())},
            {"max", Eigen::Vector2d(map.m_bbox_.row(1).transpose())},
        });
        json_data["verts"] = map.m_meter_space_->GetSurface()->vertices;
    }

    void
    HouseExpoMap::FromJson(const nlohmann::json &json_data, HouseExpoMap &map) {
        json_data.at("id").get_to(map.m_room_id_);
        double x_min, y_min, x_max, y_max;
        json_data.at("bbox").at("min").at(0).get_to(x_min);
        json_data.at("bbox").at("min").at(1).get_to(y_min);
        json_data.at("bbox").at("max").at(0).get_to(x_max);
        json_data.at("bbox").at("max").at(1).get_to(y_max);
        // clang-format off
        map.m_bbox_ << x_min, y_min,
                       x_max, y_max;
        // clang-format on

        auto verts = json_data.at("verts").get<Eigen::Matrix2Xd>();
        std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> ordered_vertices;
        ordered_vertices.emplace_back(verts);
        Eigen::Scalar<bool> outside_flags{false};
        map.m_meter_space_ = std::make_shared<Space2D>(ordered_vertices, outside_flags);
    }
}  // namespace erl::geometry
