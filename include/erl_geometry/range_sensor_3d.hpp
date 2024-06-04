#pragma once

#include "erl_common/logging.hpp"

#include <open3d/t/geometry/RaycastingScene.h>

namespace erl::geometry {

    class RangeSensor3D {
    protected:
        std::shared_ptr<open3d::t::geometry::RaycastingScene> m_scene_ = nullptr;

    public:
        RangeSensor3D() = delete;

        RangeSensor3D(const Eigen::Ref<const Eigen::Matrix3Xd> &vertices, const Eigen::Ref<const Eigen::Matrix3Xi> &triangles)
            : m_scene_(std::make_shared<open3d::t::geometry::RaycastingScene>()) {
            long num_vertices = vertices.cols();
            const open3d::core::Tensor vertices_tensor({num_vertices, 3}, open3d::core::Dtype::Float32);
            for (long i = 0; i < num_vertices; ++i) {
                vertices_tensor[i][0] = vertices(0, i);
                vertices_tensor[i][1] = vertices(1, i);
                vertices_tensor[i][2] = vertices(2, i);
            }
            long num_triangles = triangles.cols();
            const open3d::core::Tensor triangles_tensor({num_triangles, 3}, open3d::core::Dtype::UInt32);
            for (long i = 0; i < num_triangles; ++i) {
                triangles_tensor[i][0] = triangles(0, i);
                triangles_tensor[i][1] = triangles(1, i);
                triangles_tensor[i][2] = triangles(2, i);
            }
            m_scene_->AddTriangles(vertices_tensor, triangles_tensor);
        }

        RangeSensor3D(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles)
            : m_scene_(std::make_shared<open3d::t::geometry::RaycastingScene>()) {
            auto num_vertices = static_cast<long>(vertices.size());
            const open3d::core::Tensor vertices_tensor({num_vertices, 3}, open3d::core::Dtype::Float32);
            for (long i = 0; i < num_vertices; ++i) {
                vertices_tensor[i][0] = vertices[i][0];
                vertices_tensor[i][1] = vertices[i][1];
                vertices_tensor[i][2] = vertices[i][2];
            }
            auto num_triangles = static_cast<long>(triangles.size());
            const open3d::core::Tensor triangles_tensor({num_triangles, 3}, open3d::core::Dtype::UInt32);
            for (long i = 0; i < num_triangles; ++i) {
                triangles_tensor[i][0] = triangles[i][0];
                triangles_tensor[i][1] = triangles[i][1];
                triangles_tensor[i][2] = triangles[i][2];
            }
            m_scene_->AddTriangles(vertices_tensor, triangles_tensor);
        }

        explicit RangeSensor3D(const std::shared_ptr<open3d::t::geometry::RaycastingScene> &o3d_scene)
            : m_scene_(o3d_scene) {
            ERL_ASSERTM(m_scene_ != nullptr, "scene is nullptr.");
        }

        virtual ~RangeSensor3D() = default;

        [[nodiscard]] std::shared_ptr<open3d::t::geometry::RaycastingScene>
        GetScene() const {
            return m_scene_;
        }

        [[nodiscard]] virtual Eigen::MatrixX<Eigen::Vector3d>
        GetRayDirectionsInFrame() const = 0;

        [[nodiscard]] Eigen::MatrixXd
        Scan(
            const Eigen::Ref<const Eigen::Matrix3d> &orientation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            bool add_noise = false,
            double noise_stddev = 0.03) const;
    };

}  // namespace erl::geometry
