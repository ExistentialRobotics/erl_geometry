#pragma once

#include <filesystem>
#include <fstream>
#include <string>

#include "erl_common/assert.hpp"
#include "erl_common/eigen.hpp"
#include "erl_common/opencv.hpp"
#include "erl_common/json.hpp"
#include "space_2d.hpp"

namespace erl::geometry {

    class HouseExpoMap {
        std::string m_file_;
        std::string m_room_id_;
        Eigen::Matrix2d m_bbox_;
        std::shared_ptr<geometry::Space2D> m_meter_space_;

    public:
        explicit HouseExpoMap(const char *file)
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

        HouseExpoMap(const char *file, double wall_thickness)
            : HouseExpoMap(file) {
            std::cout << "Changing wall thickness to " << wall_thickness << " ... " << std::flush;
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
            std::cout << "Done." << std::endl;
#else
            std::cout << "Debug mode, no wall thickness change." << std::endl;
#endif
        }

        [[nodiscard]] std::string
        GetFile() const {
            return m_file_;
        }

        [[nodiscard]] std::string
        GetRoomId() const {
            return m_room_id_;
        }

        [[nodiscard]] const std::shared_ptr<geometry::Space2D> &
        GetMeterSpace() {
            return m_meter_space_;
        }

        inline static void
        ToJson(nlohmann::json &json_data, const HouseExpoMap &map) {
            json_data["id"] = map.m_room_id_;
            json_data["bbox"] = nlohmann::json::object({
                {"min", Eigen::Vector2d(map.m_bbox_.row(0).transpose())},
                {"max", Eigen::Vector2d(map.m_bbox_.row(1).transpose())},
            });
            json_data["verts"] = map.m_meter_space_->GetSurface()->vertices;
        }

        inline static void
        FromJson(const nlohmann::json &json_data, HouseExpoMap &map) {
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
    };
}  // namespace erl::geometry
