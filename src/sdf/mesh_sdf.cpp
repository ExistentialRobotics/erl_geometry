// original code repo: https://github.com/sxyu/sdf.git

#include "erl_geometry/sdf/mesh_sdf.hpp"

#include "erl_geometry/kdtree_eigen_adaptor.hpp"
#include "erl_geometry/RTree.h"
#include "erl_geometry/sdf/sdf_util.hpp"

#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/RaycastingScene.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <random>
#include <thread>

namespace erl::geometry {
    using namespace sdf_util;

    struct MeshSdf::PySdfImpl {
        PySdfImpl(
            const Eigen::Ref<const Points>& verts,
            const Eigen::Ref<const Triangles>& faces,
            const bool robust)
            : verts(verts),
              faces(faces),
              robust(robust),
              kd_tree(verts) {
            face_normal.resize(faces.rows(), Points::ColsAtCompileTime);
            face_area.resize(faces.rows());
            adj_faces.resize(verts.rows());
            for (int i = 0; i < faces.rows(); ++i) {
                for (int j = 0; j < Triangles::ColsAtCompileTime; ++j) {
                    adj_faces[faces(i, j)].push_back(i);
                }
            }

            Update(false);
        }

        void
        Update(const bool need_rebuild_kd_tree = true) {
            if (need_rebuild_kd_tree) kd_tree.rebuild();
            if (robust) { rtree.RemoveAll(); }
            aabb.head<3>() = verts.colwise().minCoeff();
            aabb.tail<3>() = verts.colwise().maxCoeff();

            if (robust) {
                // Generate a random rotation matrix using a unit quaternion
                // to use as a raycast frame, ref
                // https://en.wikipedia.org/wiki/Rotation_matrix#Uniform_random_rotation_matrices
                auto& rg = get_rng();
                std::normal_distribution<float> gaussian(0.0f, 1.0f);
                Eigen::Quaternionf rand_rot(gaussian(rg), gaussian(rg), gaussian(rg), gaussian(rg));
                rand_rot.normalize();
                raycast_axes.noalias() = rand_rot.toRotationMatrix();
            }
            for (int i = 0; i < faces.rows(); ++i) {
                const auto va = verts.row(faces(i, 0)), vb = verts.row(faces(i, 1)),
                           vc = verts.row(faces(i, 2));
                if (robust) {
                    Eigen::Matrix<float, 1, 3, Eigen::RowMajor> face_aabb_min = va * raycast_axes,
                                                                face_aabb_max = va * raycast_axes;
                    face_aabb_min = face_aabb_min.cwiseMin(vb * raycast_axes);
                    face_aabb_min = face_aabb_min.cwiseMin(vc * raycast_axes);
                    face_aabb_max = face_aabb_max.cwiseMax(vb * raycast_axes);
                    face_aabb_max = face_aabb_max.cwiseMax(vc * raycast_axes);
                    rtree.Insert(face_aabb_min.data(), face_aabb_max.data(), i);
                }

                face_normal.row(i).noalias() = sdf_util::calc_normal<float>(va, vb, vc);
                face_area[i] = face_normal.row(i).norm();
                face_normal.row(i) /= face_area[i];
            }
            total_area = face_area.sum();
            face_area_cum.resize(0);
        }

        Eigen::VectorXi
        NearestNeighbor(
            const Eigen::Ref<const Points>& points,
            const std::size_t n_threads = std::thread::hardware_concurrency()) const {
            Eigen::VectorXi result(points.rows());
            maybe_parallel_for(
                [&, this](const int i) {
                    size_t index;
                    float dist;
                    nanoflann::KNNResultSet<float> resultSet(1);
                    resultSet.init(&index, &dist);
                    kd_tree.index->findNeighbors(
                        resultSet,
                        points.data() + i * 3,
                        nanoflann::SearchParameters(0));
                    // the original code uses eps=10 for nanoflann::SearchParameters.
                    // this makes the SDF result inaccurate.
                    result[i] = static_cast<int>(index);
                },
                static_cast<int>(points.rows()),
                n_threads);
            return result;
        }

        Vector
        Calc(
            Eigen::Ref<const Points> points,
            bool trunc_aabb = false,
            std::size_t n_threads = std::thread::hardware_concurrency()) const {
            Vector result(points.rows());
            result.setConstant(std::numeric_limits<float>::max());

            const float DIST_EPS = robust ? 0.f : 1e-5f;

            maybe_parallel_for(
                [&](int i) {
                    size_t neighbor_index;
                    float _dist;
                    nanoflann::KNNResultSet<float> result_set(1);

                    auto point = points.row(i);

                    float sign = 0;
                    if (robust) { sign = RayCastImpl(point); }

                    float& min_dist = result[i];
                    if (trunc_aabb) {
                        // Only care about the sign being correct, so we can use AABB
                        for (int t = 0; t < 3; ++t) {
                            if (point[t] < aabb[t] || point[t] > aabb[t + 3]) {
                                // Out of mesh's bounding box
                                min_dist = -std::numeric_limits<float>::max();
                            }
                        }
                    }

                    result_set.init(&neighbor_index, &_dist);
                    kd_tree.index->findNeighbors(
                        result_set,
                        point.data(),
                        nanoflann::SearchParameters(0));
                    // the original code uses eps=10 for nanoflann::SearchParameters.
                    // this makes the SDF result inaccurate.

                    Eigen::Matrix<float, 1, 3, Eigen::RowMajor> avg_normal;
                    if (!robust) { avg_normal.setZero(); }
                    // Eigen::Matrix<float, 3, 3, Eigen::RowMajor> face_tri;
                    for (const int faceid: adj_faces[neighbor_index]) {
                        const auto face = faces.row(faceid);

                        const auto normal = face_normal.row(faceid);
                        const auto tridist = dist_point2tri<float>(
                            point,
                            verts.row(face(0)),
                            verts.row(face(1)),
                            verts.row(face(2)),
                            normal,
                            face_area[faceid]);
                        if (tridist < min_dist) {
                            min_dist = tridist;
                            if (!robust) { avg_normal.noalias() = normal; }
                        } else if (!robust && tridist < min_dist + DIST_EPS) {
                            avg_normal.noalias() += normal;
                        }
                    }
                    min_dist = std::sqrt(min_dist);
                    if (robust) {
                        min_dist *= sign;
                    } else if (
                        avg_normal.dot(point - verts.row(static_cast<long>(neighbor_index))) > 0) {
                        // Outside, by normal
                        min_dist = -min_dist;
                    }

                    min_dist = -min_dist;  // invert sign, now positive is outside.
                },
                static_cast<int>(points.rows()),
                n_threads);
            return result;
        }

        Eigen::Matrix<bool, Eigen::Dynamic, 1>
        Contains(
            Eigen::Ref<const Points> points,
            std::size_t n_threads = std::thread::hardware_concurrency()) const {
            if (robust) {
                Eigen::Matrix<bool, Eigen::Dynamic, 1> result(points.rows());
                maybe_parallel_for(
                    [&](int i) { result[i] = RayCastImpl(points.row(i)) >= 0.0f; },
                    static_cast<int>(points.rows()),
                    n_threads);
                return result;
            }
            Vector vals = Calc(points, true, n_threads);
            return vals.array() >= 0;
        }

        Points
        SampleSurface(const int num_points) const {
            if (face_area.rows() == 0) {
                std::cerr << "ERROR: No faces, can't sample surface.\n";
                return {};
            }
            auto& rg = get_rng();
            std::uniform_real_distribution<float> uniform(
                0.0f,
                1.0f - std::numeric_limits<float>::epsilon());
            // float running = 0.f;

            Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> result(num_points, 3);

            // Inverse distribution sampling:
            // pick each face with prob proportional to the area
            Eigen::VectorXf rand_area(num_points);
            for (int i = 0; i < num_points; ++i) { rand_area[i] = uniform(rg) * total_area; }
            if (face_area_cum.rows() == 0) {
                face_area_cum.resize(face_area.rows());
                face_area_cum[0] = face_area[0];
                for (int i = 1; i < face_area.rows(); ++i)
                    face_area_cum[i] = face_area_cum[i - 1] + face_area[i];
            }

            for (int j = 0; j < num_points; ++j) {
                // Triangle i
                long i = std::lower_bound(
                             face_area_cum.data(),
                             face_area_cum.data() + face_area_cum.rows(),
                             rand_area[j]) -
                         face_area_cum.data();
                i = std::max(std::min(i, faces.rows() - 1l), 0l);

                const auto face = faces.row(i);
                const auto a = verts.row(face[0]), b = verts.row(face[1]), c = verts.row(face[2]);

                // Point j <-- u.a.r. in triangle i
                const Eigen::Matrix<float, 1, 3, Eigen::RowMajor> ab = b - a, ac = c - a,
                                                                  bc = c - b;
                const Eigen::Matrix<float, 1, 3, Eigen::RowMajor> perp =
                    bc.cross(face_normal.row(i)).normalized();
                bool a_dir_of_bc = ab.dot(perp) < 0.0;

                // Random in quadrilateral
                result.row(j).noalias() =
                    a + uniform(rg) * ab + uniform(rg) * ac;  // Reflect over bc if we're over it
                const float bp_dot_perp = (result.row(j) - b).dot(perp);
                const bool p_dir_of_bc = bp_dot_perp >= 0.0;
                if (p_dir_of_bc != a_dir_of_bc) {
                    result.row(j).noalias() -= bp_dot_perp * perp * 2.f;
                }
            }
            return result;
        }

        // Input vertices
        Eigen::Ref<const Points> verts;
        // Input triangular faces
        Eigen::Ref<const Triangles> faces;
        // Whether to use 'robust' sign computation
        const bool robust;

        // Stores face normals [n_face, 3]
        Points face_normal;
        // Stores face areas [n_face]
        Vector face_area;
        // Cumulative face areas for sampling [n_face]
        mutable Vector face_area_cum;
        // Total surface area
        float total_area = 0.0f;
        // Stores adjacent faces to a point [n_points, <n_adj_faces>]
        std::vector<std::vector<int>> adj_faces;

        // Store AABB of entire mesh
        // (minx, miny, minz, maxx, maxy, maxz)
        Eigen::Matrix<float, 1, 6, Eigen::RowMajor> aabb;

    private:
        // KD tree for NN search
        nanoflann::KDTreeEigenRefAdaptor<const Points, 3, nanoflann::metric_L2_Simple> kd_tree;

        // Face R-Tree (aka AABB Tree)
        RTree<int, float, 3> rtree;

        // Random rotation matrix which transforms points from
        // global space to space used for ray casting.
        // This allows us to use the RTree to do ray casting in an arbitrary
        // direction. Only to be used in robust mode
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> raycast_axes;

        // RayCastImpl to check if a point is in or on the surface of mesh.
        // Returns 1 if in, -1 else.
        // Only to be used in robust mode
        float
        RayCastImpl(
            const Eigen::Ref<const Eigen::Matrix<float, 1, 3, Eigen::RowMajor>>& point_orig) const {
            for (int t = 0; t < 3; ++t) {
                if (point_orig[t] < aabb[t] || point_orig[t] > aabb[t + 3]) {
                    // Out of mesh's bounding box
                    return -1;
                }
            }
            Eigen::Matrix<float, 1, 3, Eigen::RowMajor> point = point_orig * raycast_axes;

            // ax_idx: axis index, either 0(x) or 2(z).
            //         note: y not supported by current code for efficiency
            // ax_inv: if true, raycast in the negative direction along axis, else the positive
            // direction.
            // return: 1 if inside, 0 else
            auto raycast = [&](const int ax_idx, const bool ax_inv) -> int {
                Eigen::Matrix<float, 1, 3, Eigen::RowMajor> aabb_min, aabb_max;
                int contained = 0;
                int ax_offs = ax_idx == 0 ? 1 : 0;

                auto check_face = [&](int faceid) -> bool {
                    const auto face = faces.row(faceid);
                    Eigen::Matrix<float, 1, 3, Eigen::RowMajor> normal =
                        face_normal.row(faceid) * raycast_axes;
                    if ((normal.dot(point - verts.row(face[0]) * raycast_axes) * normal[ax_idx] >
                         0.f) == ax_inv) {
                        const auto bary = bary2d<float>(
                            point.segment<2>(ax_offs),
                            (verts.row(face[0]) * raycast_axes).segment<2>(ax_offs),
                            (verts.row(face[1]) * raycast_axes).segment<2>(ax_offs),
                            (verts.row(face[2]) * raycast_axes).segment<2>(ax_offs));
                        if (bary[0] >= 0.f && bary[1] >= 0.f && bary[2] >= 0.f) { contained ^= 1; }
                    }
                    return true;
                };
                aabb_min.noalias() = point;
                aabb_max.noalias() = point;
                if (ax_inv) {
                    aabb_min[ax_idx] = -std::numeric_limits<float>::max();
                } else {
                    aabb_max[ax_idx] = std::numeric_limits<float>::max();
                }
                rtree.Search(aabb_min.data(), aabb_max.data(), check_face);
                return contained;
            };
            int result = raycast(2, false) + raycast(2, true);
            if (result == 1) result += raycast(0, false);  // Tiebreaker
            return result > 1 ? 1.0f : -1.0f;
        }
    };

    struct MeshSdf::Open3dImpl {

        KdTree3f kd_tree;
        open3d::t::geometry::RaycastingScene scene;
        open3d::geometry::TriangleMesh mesh;
        std::vector<std::vector<int>> adj_faces;
        Vector face_area;
        Points face_normal;
        Eigen::Matrix<float, 6, 1> aabb;
        bool flip_sign = false;

        Open3dImpl(
            const Eigen::Ref<const Eigen::Matrix3Xd>& verts,
            const Eigen::Ref<const Eigen::Matrix3Xi>& faces)
            : kd_tree(verts.cast<float>()) {
            mesh.vertices_.resize(verts.cols());
            mesh.triangles_.resize(faces.cols());
            std::memcpy(mesh.vertices_.data()->data(), verts.data(), verts.size() * sizeof(double));
            std::memcpy(mesh.triangles_.data()->data(), faces.data(), faces.size() * sizeof(int));
            mesh.ComputeTriangleNormals();
            mesh.ComputeVertexNormals();
            scene.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(mesh));

            // Open3D determines the sign by checking if the point is inside the closed surface.
            // If inside the closed surface, the returned sign is -1.
            // However, the mesh may have flipped normals. We need to check this.
            auto pcd = mesh.SamplePointsUniformly(100, true);
            Points points(static_cast<long>(pcd->points_.size()), 3);
            for (long i = 0; i < points.rows(); ++i) {
                points.row(i) =
                    (pcd->points_[i] + 0.01f * pcd->normals_[i]).cast<float>().transpose();
            }
            flip_sign = Calc(points)
                            .unaryExpr([](const float val) { return val < 0 ? -1.f : 1.f; })
                            .mean() < 0.0f;

            adj_faces.resize(verts.cols());
            for (int i = 0; i < faces.rows(); ++i) {
                for (int j = 0; j < Triangles::ColsAtCompileTime; ++j) {
                    adj_faces[faces(i, j)].push_back(i);
                }
            }

            face_area.resize(faces.cols());
#pragma omp parallel for default(none)
            for (long i = 0; i < face_area.size(); ++i) {
                face_area[i] = static_cast<float>(mesh.GetTriangleArea(i));
            }

            mesh.ComputeTriangleNormals();
            face_normal.resize(faces.cols(), Points::ColsAtCompileTime);
            for (long i = 0; i < faces.cols(); ++i) {
                face_normal.row(i) <<  //
                    static_cast<float>(mesh.triangle_normals_[static_cast<size_t>(i)][0]),
                    static_cast<float>(mesh.triangle_normals_[static_cast<size_t>(i)][1]),
                    static_cast<float>(mesh.triangle_normals_[static_cast<size_t>(i)][2]);
            }

            aabb.head<3>() = mesh.GetMinBound().cast<float>();
            aabb.tail<3>() = mesh.GetMaxBound().cast<float>();
        }

        [[nodiscard]] Vector
        Calc(
            const Eigen::Ref<const Points>& points,
            const int n_threads = static_cast<int>(std::thread::hardware_concurrency())) const {
            open3d::core::Tensor positions_tensor({points.rows(), 3}, open3d::core::Dtype::Float32);
            Eigen::Map<Points>(
                static_cast<float*>(positions_tensor.GetDataPtr()),
                points.rows(),
                3) = points;
            auto sdf_gt_tensor = const_cast<open3d::t::geometry::RaycastingScene*>(&scene)
                                     ->ComputeSignedDistance(positions_tensor, n_threads, 3);
            Vector sdf =
                Eigen::Map<Vector>(static_cast<float*>(sdf_gt_tensor.GetDataPtr()), points.rows());
            if (flip_sign) { sdf = -sdf; }
            return sdf;
        }

        [[nodiscard]] Eigen::VectorXi
        NearestNeighbor(
            const Eigen::Ref<const Points>& points,
            const std::size_t n_threads = std::thread::hardware_concurrency()) const {
            Eigen::VectorXi result(points.rows());
            maybe_parallel_for(
                [&, this](const int i) {
                    Eigen::Vector3f point = points.row(i).transpose().cast<float>();
                    long index;
                    float dist;
                    kd_tree.Nearest(point, index, dist);
                    // the original code uses eps=10 for nanoflann::SearchParameters.
                    // this makes the SDF result inaccurate.
                    result[i] = static_cast<int>(index);
                },
                static_cast<int>(points.rows()),
                n_threads);
            return result;
        }

        [[nodiscard]] Eigen::Matrix<bool, Eigen::Dynamic, 1>
        Contains(
            const Eigen::Ref<const Points>& points,
            const std::size_t n_threads = std::thread::hardware_concurrency()) const {
            Vector vals = Calc(points, static_cast<int>(n_threads));
            return vals.array() >= 0;
        }
    };

    MeshSdf::MeshSdf(
        const Eigen::Ref<const Points>& verts,
        const Eigen::Ref<const Triangles>& faces,
        const bool use_open3d,
        const bool robust,
        const bool copy)
        : use_open3d(use_open3d),
          robust(robust),
          own_data(copy || use_open3d) {
        if (use_open3d) {
            owned_verts = verts;
            owned_faces = faces;
            open3d_impl = std::make_unique<Open3dImpl>(
                verts.cast<double>().transpose(),
                faces.cast<int>().transpose());
            return;
        }

        if (copy) {
            owned_verts = verts;
            owned_faces = faces;
            pysdf_impl = std::make_unique<PySdfImpl>(owned_verts, owned_faces, robust);
        } else {
            pysdf_impl = std::make_unique<PySdfImpl>(verts, faces, robust);
        }
    }

    MeshSdf::~MeshSdf() = default;

    const std::vector<int>&
    MeshSdf::GetAdjFaces(const int point_id) const {
        if (use_open3d) { return open3d_impl->adj_faces[point_id]; }
        return pysdf_impl->adj_faces[point_id];
    }

    float
    MeshSdf::GetSurfaceArea() const {
        if (use_open3d) { return static_cast<float>(open3d_impl->mesh.GetSurfaceArea()); }
        return pysdf_impl->total_area;
    }

    const MeshSdf::Vector&
    MeshSdf::GetFaceAreas() const {
        if (use_open3d) { return open3d_impl->face_area; }
        return pysdf_impl->face_area;
    }

    const MeshSdf::Points&
    MeshSdf::GetFaceNormals() const {
        if (use_open3d) { return open3d_impl->face_normal; }
        return pysdf_impl->face_normal;
    }

    Eigen::Ref<const Eigen::Matrix<float, 6, 1>>
    MeshSdf::GetAabb() const {
        if (use_open3d) { return open3d_impl->aabb; }
        return pysdf_impl->aabb.transpose();
    }

    Eigen::Ref<const MeshSdf::Triangles>
    MeshSdf::GetFaces() const {
        if (use_open3d) { return owned_faces; }
        return pysdf_impl->faces;
    }

    Eigen::Ref<MeshSdf::Triangles>
    MeshSdf::GetFacesMutable() {
        if (!own_data) { std::cerr << "ERROR: 'faces' is non mutable, construct with copy=True\n"; }
        return owned_faces;
    }

    Eigen::Ref<const MeshSdf::Points>
    MeshSdf::GetVertices() const {
        if (use_open3d) { return owned_verts; }
        return pysdf_impl->verts;
    }

    Eigen::Ref<MeshSdf::Points>
    MeshSdf::GetVerticesMutable() {
        if (!own_data) { std::cerr << "ERROR: 'verts' is non mutable, construct with copy=True\n"; }
        return owned_verts;
    }

    MeshSdf::Vector
    MeshSdf::operator()(
        const Eigen::Ref<const Points>& points,
        const bool trunc_aabb,
        const std::size_t n_threads) const {
        if (use_open3d) { return open3d_impl->Calc(points, static_cast<int>(n_threads)); }
        return pysdf_impl->Calc(points, trunc_aabb, n_threads);
    }

    Eigen::VectorXi
    MeshSdf::NearestNeighbor(const Eigen::Ref<const Points>& points, const std::size_t n_threads)
        const {
        if (use_open3d) { return open3d_impl->NearestNeighbor(points, n_threads); }
        return pysdf_impl->NearestNeighbor(points, n_threads);
    }

    Eigen::Matrix<bool, Eigen::Dynamic, 1>
    MeshSdf::Contains(const Eigen::Ref<const Points>& points, const std::size_t n_threads) const {
        if (use_open3d) { return open3d_impl->Contains(points, n_threads); }
        return pysdf_impl->Contains(points, n_threads);
    }

    void
    MeshSdf::Update() const {
        if (use_open3d) {
            ERL_WARN(
                "Open3D backend does not support Update, rebuild the MeshSdf object to update.");
            return;
        }
        pysdf_impl->Update();
    }

    MeshSdf::Points
    MeshSdf::SampleSurface(const int num_points) const {
        if (use_open3d) {
            const auto pcd = open3d_impl->mesh.SamplePointsUniformly(num_points);
            return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(
                       pcd->points_.data()->data(),
                       num_points,
                       3)
                .cast<float>();
        }
        return pysdf_impl->SampleSurface(num_points);
    }
}  // namespace erl::geometry
