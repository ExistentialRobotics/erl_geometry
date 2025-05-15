// sdf: Triangle mesh to signed-distance function (SDF) library
// Copyright Alex Yu 2020
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "erl_common/eigen.hpp"

#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

namespace erl::geometry {

    namespace sdf_util {

        template<class T>
        // the SQUARED shortest 3D point-to-line distance
        T
        dist_point2line(
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& p,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& a,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& b) {
            Eigen::Matrix<T, 1, 3> ap = p - a, ab = b - a;
            return (ap - (ap.dot(ab) / ab.squaredNorm()) * ab).squaredNorm();
        }

        // the SQUARED shortest 3D point-to-line segment distance
        template<class T>
        T
        dist_point2lineseg(
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& p,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& a,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& b) {
            Eigen::Matrix<T, 1, 3> ap = p - a, ab = b - a;
            T t = ap.dot(ab) / ab.squaredNorm();
            t = std::max(T(0.0), std::min(T(1.0), t));
            return (ap - t * ab).squaredNorm();
        }

        template<class T>
        Eigen::Matrix<T, 1, 3, Eigen::RowMajor>
        calc_normal(
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& a,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& b,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& c) {
            return (b - a).cross(c - a);
        }

        template<class T>
        // Find barycentric coords of p in triangle (a,b,c) in 3D
        // (p does NOT have to be projected to plane beforehand)
        // normal, area_abc to be computed using util::normal,
        // where normal is a normalized vector, area is magnitude
        Eigen::Matrix<T, 1, 3, Eigen::RowMajor>
        bary(
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& p,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& a,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& b,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& c,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& normal,
            float area_abc) {
            float area_pbc = normal.dot((b - p).cross(c - p));
            float area_pca = normal.dot((c - p).cross(a - p));

            Eigen::Matrix<T, 1, 3> uvw;
            uvw.x() = area_pbc / area_abc;
            uvw.y() = area_pca / area_abc;
            uvw.z() = T(1.0) - uvw.x() - uvw.y();

            return uvw;
        }

        template<class T>
        // the SQUARED shortest 3D point-to-triangle distance
        // normal, area_abc to be computed using util::normal,
        // where normal is normalized vector, area is magnitude
        T
        dist_point2tri(
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& p,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& a,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& b,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& c,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 3, Eigen::RowMajor>>& normal,
            float area) {
            const Eigen::Matrix<T, 1, 3> uvw = bary<T>(p, a, b, c, normal, area);
            if (uvw[0] < 0) { return dist_point2lineseg<T>(p, b, c); }
            if (uvw[1] < 0) { return dist_point2lineseg<T>(p, a, c); }
            if (uvw[2] < 0) { return dist_point2lineseg<T>(p, a, b); }
            return (uvw[0] * a + uvw[1] * b + uvw[2] * c - p).squaredNorm();
        }

        template<class T>
        Eigen::Matrix<T, 1, 3, Eigen::RowMajor>
        bary2d(
            const Eigen::Ref<const Eigen::Matrix<T, 1, 2, Eigen::RowMajor>>& p,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 2, Eigen::RowMajor>>& a,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 2, Eigen::RowMajor>>& b,
            const Eigen::Ref<const Eigen::Matrix<T, 1, 2, Eigen::RowMajor>>& c) {
            Eigen::Matrix<T, 1, 2, Eigen::RowMajor> v0 = b - a, v1 = c - a, v2 = p - a;
            Eigen::Matrix<T, 1, 3> result;
            const float invden = 1.f / (v0.x() * v1.y() - v1.x() * v0.y());
            result[1] = (v2.x() * v1.y() - v1.x() * v2.y()) * invden;
            result[2] = (v0.x() * v2.y() - v2.x() * v0.y()) * invden;
            result[0] = 1.0f - result.template tail<2>().sum();
            return result;
        }
    }  // namespace sdf_util

    // Signed distance function utility for watertight meshes.
    //
    // Basic usage: SDF sdf(verts, faces); Vector sdf_vals = sdf(query_points);
    // Get nearest neighbor (verts) indices: sdf.NearestNeighbor(query_points);
    // Check containment (returns bool): sdf.Contains(query_points);
    struct MeshSdf {
        using Index = uint32_t;
        using Points = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
        using Points2D = Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>;
        using Triangles = Eigen::Matrix<Index, Eigen::Dynamic, 3, Eigen::RowMajor>;
        using Matrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using Vector = Eigen::Matrix<float, Eigen::Dynamic, 1>;

        // Construct SDF instance from triangle mesh with given vertices and faces
        // Note: Mesh is assumed to be watertight, and all vertex positions are
        // expected to be free of nan/inf
        //
        // Basic usage: SDF sdf(verts, faces); Vector sdf_vals = sdf(query_points);
        // Get nearest neighbor (verts) indices: sdf.NearestNeighbor(query_points);
        // Check containment (returns bool): sdf.Contains(query_points);
        //
        // @param verts mesh vertices. If the contents of this matrix are modified,
        // please call SDF::Update() to Update the internal representation.
        // Else the results will be incorrect.
        // @param faces mesh faces. The contents of this matrix should not be
        // modified for the lifetime of this instance.
        // @param robust whether to use robust mode. In robust mode,
        // @param copy whether to make a copy of the data instead of referencing it
        // SDF/containment computation is robust to mesh self-intersections and
        // face winding but is slower.
        MeshSdf(
            const Eigen::Ref<const Points>& verts,
            const Eigen::Ref<const Triangles>& faces,
            bool robust = true,
            bool copy = false);

        template<typename T>
        MeshSdf(
            const std::vector<Eigen::Vector3<T>>& verts,
            const std::vector<Eigen::Vector3i>& faces,
            const bool robust = true)
            : MeshSdf(
                  Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>>(
                      verts.data()->data(),
                      static_cast<long>(verts.size()),
                      3)
                      .template cast<float>(),
                  Eigen::Map<const Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>>(
                      faces.data()->data(),
                      static_cast<long>(faces.size()),
                      3)
                      .cast<uint32_t>(),
                  robust,
                  true) {}

        ~MeshSdf();

        /*** PRIMARY INTERFACE ***/
        // Fast approximate signed-distance function.
        // Points inside the mesh have positive signs and outside have negative signs.
        //
        // Method: computes minimum distance to a triangular face incident to
        // the nearest vertex for each point.
        //
        // @param points input points
        // @param trunc_aabb if true, returns -FLT_MAX for all points outside mesh's
        // bounding box
        // @return approx SDF values at input points
        //
        // WARNING: if robust=false (from constructor), this WILL FAIL if the mesh
        // has self-intersections. In particular, the signs of points inside the
        // mesh may be flipped.
        Vector
        operator()(
            const Eigen::Ref<const Points>& points,
            bool trunc_aabb = false,
            std::size_t n_threads = std::thread::hardware_concurrency()) const;

        template<typename T>
        Eigen::VectorX<T>
        operator()(
            const Eigen::Matrix3X<T>& points,
            const bool trunc_aabb = false,
            const std::size_t n_threads = std::thread::hardware_concurrency()) const {
            using PointsT = Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;
            return operator()(
                       Eigen::Map<const PointsT>(points.data(), points.cols(), 3)
                           .template cast<float>(),
                       trunc_aabb,
                       n_threads)
                .transpose()
                .template cast<T>();
        }

        // Return the exact nearest neighbor vertex index for each point (index as in
        // input verts)
        [[nodiscard]] Eigen::VectorXi
        NearestNeighbor(
            const Eigen::Ref<const Points>& points,
            std::size_t n_threads = std::thread::hardware_concurrency()) const;

        // Return 1 for each point inside/on the surface of the mesh and 0 for outside.
        //
        // @param points input points
        // @return indicator of whether each point is in OR on the surface of mesh
        //
        // WARNING: if robust=false (from constructor), this WILL FAIL if the mesh
        // has self-intersections.
        [[nodiscard]] Eigen::Matrix<bool, Eigen::Dynamic, 1>
        Contains(
            const Eigen::Ref<const Points>& points,
            std::size_t n_threads = std::thread::hardware_concurrency()) const;

        // Call if vertex positions have been updated to rebuild the KD tree
        // and update face normals+areas
        void
        Update();

        /*** MISC UTILITIES ***/
        // Sample 'num_points' points uniformly on surface, output (num_points, 3).
        // Note: this takes O(num_points * log(num_faces)) time.
        [[nodiscard]] Points
        SampleSurface(int num_points) const;

        /*** DATA ACCESSORS ***/
        // Get adjacent faces of point at verts[point_id]
        [[nodiscard]] const std::vector<int>&
        GetAdjFaces(int point_id) const;

        // Get the total surface area of mesh
        [[nodiscard]] float
        GetSurfaceArea() const;

        // Get a vector of face areas, shape (num_faces)
        [[nodiscard]] const Vector&
        GetFaceAreas() const;

        // Get matrix of face normals, shape (num_faces, 3).
        // Normal of face i (from faces passed to constructor) is in row i
        [[nodiscard]] const Points&
        GetFaceNormals() const;

        // Get AABB of entire mesh, shape (6).
        // (minx, miny, minz, maxx, maxy, maxz)
        [[nodiscard]] Eigen::Ref<const Eigen::Matrix<float, 6, 1>>
        GetAabb() const;

        // Get faces
        [[nodiscard]] Eigen::Ref<const Triangles>
        GetFaces() const;

        Eigen::Ref<Triangles>
        GetFacesMutable();

        // Get verts
        [[nodiscard]] Eigen::Ref<const Points>
        GetVertices() const;

        Eigen::Ref<Points>
        GetVerticesMutable();

        // Whether SDF is in robust mode
        const bool robust;

        // Whether we own data
        const bool own_data;

    private:
        // Optional owned data
        Points owned_verts;
        Triangles owned_faces;

        struct Impl;
        std::unique_ptr<Impl> p_impl;
    };
}  // namespace erl::geometry
