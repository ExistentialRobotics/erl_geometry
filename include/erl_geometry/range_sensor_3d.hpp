#pragma once

#include "erl_common/logging.hpp"

#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/RaycastingScene.h>

namespace erl::geometry {

    class RangeSensor3D {
    protected:
        std::shared_ptr<open3d::t::geometry::RaycastingScene> m_scene_ = nullptr;
        Eigen::MatrixX<Eigen::Vector3d> m_normals_{};  // cache normals of the last scan

    public:
        RangeSensor3D()
            : m_scene_(std::make_shared<open3d::t::geometry::RaycastingScene>()) {}

        explicit RangeSensor3D(const std::shared_ptr<open3d::t::geometry::RaycastingScene> &o3d_scene)
            : m_scene_(o3d_scene) {
            ERL_ASSERTM(m_scene_ != nullptr, "scene is nullptr.");
        }

        virtual ~RangeSensor3D() = default;

        void
        AddMesh(const std::string &mesh_path) const {
            const auto mesh = open3d::io::CreateMeshFromFile(mesh_path);
            ERL_ASSERTM(mesh != nullptr, "Failed to load mesh from file: {}", mesh_path);
            m_scene_->AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*mesh));
        }

        void
        AddMesh(const Eigen::Ref<const Eigen::Matrix3Xd> &vertices, const Eigen::Ref<const Eigen::Matrix3Xi> &triangles) const {
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

        void
        AddMesh(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles) const {
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

        [[nodiscard]] std::shared_ptr<open3d::t::geometry::RaycastingScene>
        GetScene() const {
            return m_scene_;
        }

        [[nodiscard]] virtual Eigen::MatrixX<Eigen::Vector3d>
        GetRayDirectionsInFrame() const = 0;

        [[nodiscard]] virtual std::tuple<Eigen::Matrix3d, Eigen::Vector3d>
        GetOpticalPose(const Eigen::Ref<const Eigen::Matrix3d> &orientation, const Eigen::Ref<const Eigen::Vector3d> &translation) const = 0;

        [[nodiscard]] Eigen::MatrixXd
        Scan(
            const Eigen::Ref<const Eigen::Matrix3d> &orientation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            bool add_noise = false,
            double noise_stddev = 0.03,
            bool cache_normals = false);

        [[nodiscard]] Eigen::MatrixX<Eigen::Vector3d>
        GetCachedNormals() const {
            return m_normals_;
        }
    };

}  // namespace erl::geometry
