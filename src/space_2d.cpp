#include "erl_geometry/space_2d.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include "erl_geometry/surface_2d.hpp"
#include "erl_geometry/utils.hpp"

namespace erl::geometry {

    Space2D::Space2D(
        const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_vertices,
        const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_normals) {

        auto n_obj = (ssize_t) ordered_object_vertices.size();
        ERL_ASSERTM((ssize_t) ordered_object_normals.size() == n_obj, "#groups_of_normals != #objects.");

        ssize_t n_vtx = 0;
        ssize_t n_lines = 0;
        for (ssize_t i = 0; i < n_obj; ++i) {
            auto n = ordered_object_vertices[i].cols();
            ERL_ASSERTM(n >= 3, "#vertices(%ld) should >= 3 for object %ld.", n, i);

            auto m = ordered_object_normals[i].cols();
            ERL_ASSERTM(m == n, "#vertices (%ld) != #normals (%ld).", n, m);

            n_vtx += n;
            n_lines += n;
        }

        Eigen::Matrix2Xd vertices(2, n_vtx);
        Eigen::Matrix2Xd normals(2, n_vtx);
        Eigen::Matrix2Xi lines_to_vertices(2, n_lines);
        Eigen::Matrix2Xi objects_to_lines(2, n_obj);
        int vertex_idx_0 = 0;
        int line_idx_0 = 0;
        for (ssize_t i = 0; i < n_obj; ++i) {
            auto n = (int) ordered_object_vertices[i].cols();

            for (int j = 0; j < n; ++j) {
                vertices(0, vertex_idx_0 + j) = ordered_object_vertices[i](0, j);
                vertices(1, vertex_idx_0 + j) = ordered_object_vertices[i](1, j);
                normals(0, vertex_idx_0 + j) = ordered_object_normals[i](0, j);
                normals(1, vertex_idx_0 + j) = ordered_object_normals[i](1, j);
                lines_to_vertices(0, line_idx_0 + j) = vertex_idx_0 + j;
                lines_to_vertices(1, line_idx_0 + j) = lines_to_vertices(0, line_idx_0 + j) + 1;
            }
            lines_to_vertices(1, line_idx_0 + n - 1) = vertex_idx_0;
            objects_to_lines(0, i) = line_idx_0;
            objects_to_lines(1, i) = line_idx_0 + n;

            vertex_idx_0 += n;
            line_idx_0 += n;
        }

        m_surface_ =
            std::make_shared<Surface2D>(std::move(vertices), std::move(normals), std::move(lines_to_vertices), std::move(objects_to_lines), Eigen::VectorXb{});

        m_kdtree_ = std::make_unique<KdTree>(m_surface_->vertices);

        ComputeOutsideFlags();
    }

    Space2D::Space2D(
        const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_vertices,
        const Eigen::Ref<const Eigen::VectorXb> &outside_flags,
        double delta,
        bool parallel) {

        auto n_obj = (ssize_t) ordered_object_vertices.size();
        ERL_ASSERTM(outside_flags.size() == n_obj, "#outside_flags(%ld) != #objects(%ld).", outside_flags.size(), n_obj);

        ssize_t n_vtx = 0;
        ssize_t n_lines = 0;
        for (ssize_t i = 0; i < n_obj; ++i) {
            auto n = ordered_object_vertices[i].cols();
            ERL_ASSERTM(n >= 3, "#vertices(%ld) should >= 3 for object %ld.", n, i);
            n_vtx += n;
            n_lines += n;
        }

        Eigen::Matrix2Xd vertices(2, n_vtx);
        Eigen::Matrix2Xi lines_to_vertices(2, n_lines);
        Eigen::Matrix2Xi objects_to_lines(2, n_obj);
        int vertex_idx_0 = 0;
        int line_idx_0 = 0;
        for (ssize_t i = 0; i < n_obj; ++i) {
            auto n = (int) ordered_object_vertices[i].cols();

            for (int j = 0; j < n; ++j) {
                vertices(0, vertex_idx_0 + j) = ordered_object_vertices[i](0, j);
                vertices(1, vertex_idx_0 + j) = ordered_object_vertices[i](1, j);
                lines_to_vertices(0, line_idx_0 + j) = vertex_idx_0 + j;
                lines_to_vertices(1, line_idx_0 + j) = lines_to_vertices(0, line_idx_0 + j) + 1;
            }
            lines_to_vertices(1, line_idx_0 + n - 1) = vertex_idx_0;
            objects_to_lines(0, i) = line_idx_0;
            objects_to_lines(1, i) = line_idx_0 + n;

            vertex_idx_0 += n;
            line_idx_0 += n;
        }

        m_surface_ =
            std::make_shared<Surface2D>(std::move(vertices), Eigen::Matrix2Xd{}, std::move(lines_to_vertices), std::move(objects_to_lines), outside_flags);

        m_kdtree_ = std::make_unique<KdTree>(m_surface_->vertices);

        ComputeNormals(delta, false, parallel);  // don't use kd-tree because vertices are not always dense
    }

    Space2D::Space2D(
        const Eigen::Ref<const Eigen::MatrixXd> &map_image,  // y up, x right
        const common::GridMapInfo2D &grid_map_info,
        double free_threshold,
        double delta,
        bool parallel) {

        // in pixels
        Eigen::Matrix2Xd vertices;
        Eigen::Matrix2Xi lines_to_vertices;
        Eigen::Matrix2Xi objects_to_lines;
        MarchingSquare(map_image, free_threshold, vertices, lines_to_vertices, objects_to_lines);  // in pixels, (u, v) <--> (x_grid, height - y_grid)
        m_surface_ =
            std::make_shared<Surface2D>(std::move(vertices), Eigen::Matrix2Xd{}, std::move(lines_to_vertices), std::move(objects_to_lines), Eigen::VectorXb{});
        ComputeOutsideFlags(map_image, free_threshold);

        // convert to meters
        m_surface_->vertices = grid_map_info.PixelToMeterForPoints(m_surface_->vertices.array().round().cast<int>());  // in meters
        delta = grid_map_info.Resolution().minCoeff();
        m_kdtree_ = std::make_unique<KdTree>(m_surface_->vertices);
        ComputeNormals(delta, true, parallel);  // use kd-tree because vertices generated by marching square are dense
    }

    Space2D
    Space2D::AddObstacles(const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &obstacle_vertices, double delta, bool parallel) {

        // in meters
        long num_objects = m_surface_->GetNumObjects();
        auto num_obstacles = long(obstacle_vertices.size());
        std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> ordered_object_vertices;
        ordered_object_vertices.reserve(num_objects + num_obstacles);
        Eigen::VectorXb outside_flags(num_objects + num_obstacles);
        for (long i = 0; i < num_objects; ++i) {
            ordered_object_vertices.emplace_back(m_surface_->GetObjectVertices(int(i)));
            outside_flags[i] = m_surface_->outside_flags[i];
        }

        for (long i = 0; i < num_obstacles; ++i) {
            ordered_object_vertices.push_back(obstacle_vertices[i]);
            outside_flags[num_objects + i] = true;
        }

        return {ordered_object_vertices, outside_flags, delta, parallel};
    }

    Eigen::MatrixX8U
    Space2D::GenerateMapImage(const common::GridMapInfo2D &grid_map_info, bool anti_aliased) {
        cv::Mat cv_map_image = cv::Mat::zeros(grid_map_info.Height(), grid_map_info.Width(), CV_8UC1);
        auto n_obj = m_surface_->GetNumObjects();
        for (int i = 0; i < n_obj; ++i) {
            std::vector<std::vector<cv::Point>> contours(1);
            Eigen::Matrix2Xi vertices = grid_map_info.MeterToPixelForPoints(m_surface_->GetObjectVertices(i));
            auto n_vertices = vertices.cols();
            for (int j = 0; j < n_vertices; ++j) { contours[0].emplace_back(vertices(0, j), vertices(1, j)); }
            cv::drawContours(cv_map_image, contours, 0, m_surface_->outside_flags[i] ? 0 : 255, -1, anti_aliased ? cv::LINE_8 : cv::LINE_AA);
        }
        Eigen::MatrixX8U map_image(grid_map_info.Height(), grid_map_info.Width());  // map image: x-axis to right, y-axis to top
        cv::cv2eigen(cv_map_image, map_image);
        return map_image;
    }

    Eigen::MatrixXd
    Space2D::ComputeSdfImage(const common::GridMapInfo2D &grid_map_info, SignMethod sign_method, bool use_kdtree, bool parallel) const {
        int height = grid_map_info.Height();                                 // y dimension
        int width = grid_map_info.Width();                                   // x dimension
        Eigen::MatrixXd sdf(grid_map_info.Height(), grid_map_info.Width());  // y up, x right, column-major

#pragma omp parallel for if (parallel) default(none) shared(height, width, grid_map_info, sign_method, use_kdtree, sdf)
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                Eigen::Vector2d q = grid_map_info.PixelToMeterForPoints(Eigen::Vector2i(u, v));
                sdf(v, u) = use_kdtree ? ComputeSdfWithKdtree(q, sign_method) : ComputeSdfGreedily(q, sign_method);
            }
        }
        return sdf;
    }

    Eigen::VectorXd
    Space2D::ComputeSdf(const Eigen::Ref<const Eigen::Matrix2Xd> &query_points, SignMethod sign_method, bool use_kdtree, bool parallel) const {

        Eigen::VectorXd sdf(query_points.cols());

#pragma omp parallel for if (parallel) default(none) shared(m_surface_, query_points, use_kdtree, sign_method, m_kdtree_, sdf)
        for (int i = 0; i < query_points.cols(); ++i) {
            if (use_kdtree) {
                sdf[i] = ComputeSdfWithKdtree(query_points.col(i), sign_method);
            } else {
                sdf[i] = ComputeSdfGreedily(query_points.col(i), sign_method);
            }
        }

        return sdf;
    }

    /**
     * use kd-tree to find the nearest vertex to speed up the computation: this works well ONLY when the surface points are dense enough.
     * @param q
     * @param sign_method
     * @return
     */
    double
    Space2D::ComputeSdfWithKdtree(const Eigen::Vector2d &q, SignMethod sign_method) const {
        double sdf;
        int idx_vertex_0 = 0, idx_vertex_1 = 0;

        m_kdtree_->Knn(1, q, idx_vertex_0, sdf);
        sdf = std::sqrt(sdf);

        const auto &kV0 = m_surface_->vertices.col(idx_vertex_0);
        auto idx_vertex_12 = m_surface_->GetVertexNeighbors(idx_vertex_0);

        const auto &kV1 = m_surface_->vertices.col(idx_vertex_12.first);
        const auto &kV2 = m_surface_->vertices.col(idx_vertex_12.second);

        // compute the distances to the two nearby line segments
        double dist_1 = ComputeNearestDistanceFromPointToLineSegment2D(q.x(), q.y(), kV0.x(), kV0.y(), kV1.x(), kV1.y());
        double dist_2 = ComputeNearestDistanceFromPointToLineSegment2D(q.x(), q.y(), kV0.x(), kV0.y(), kV2.x(), kV2.y());

        if (dist_1 < sdf) {
            sdf = dist_1;
            idx_vertex_1 = idx_vertex_12.first;
        }
        if (dist_2 < sdf) {
            sdf = dist_2;
            idx_vertex_1 = idx_vertex_12.second;
        }

        if (IsNegativeSdf(sign_method, q, idx_vertex_0, idx_vertex_1)) { sdf = -sdf; }

        return sdf;
    }

    /**
     * iterate over all line segments, this is slower but accurate
     * @param x
     * @param y
     * @param sign_method
     * @return
     */
    double
    Space2D::ComputeSdfGreedily(const Eigen::Ref<const Eigen::Vector2d> &q, SignMethod sign_method) const {
        double sdf = std::numeric_limits<double>::infinity();
        int idx_vertex_0 = 0, idx_vertex_1 = 0;

        for (int j = 1; j < m_surface_->GetNumLines(); ++j) {
            const auto &kL = m_surface_->lines_to_vertices.col(j);
            const auto &kV0 = m_surface_->vertices.col(kL.x());
            const auto &kV1 = m_surface_->vertices.col(kL.y());
            double dist = ComputeNearestDistanceFromPointToLineSegment2D(q.x(), q.y(), kV0.x(), kV0.y(), kV1.x(), kV1.y());
            if (dist < sdf) {
                sdf = dist;
                idx_vertex_0 = kL.x();
                idx_vertex_1 = kL.y();
            }
        }

        if (IsNegativeSdf(sign_method, q, idx_vertex_0, idx_vertex_1)) { sdf = -sdf; }

        return sdf;
    }

    Eigen::MatrixX<Eigen::VectorXd>
    Space2D::ComputeDdf(const common::GridMapInfo2D &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel) const {

        long n_directions = query_directions.cols();
        int height = grid_map_info.Height();
        int width = grid_map_info.Width();
        Eigen::MatrixX<Eigen::VectorXd> out(grid_map_info.Height(), grid_map_info.Width());

#pragma omp parallel for if (parallel) default(none) shared(n_directions, height, width, grid_map_info, query_directions, out)
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                Eigen::Vector2d point = grid_map_info.PixelToMeterForPoints(Eigen::Vector2i(u, v));
                out(v, u) = ComputeDdf(point.replicate(1, n_directions), query_directions, false);
            }
        }

        return out;
    }

    Eigen::VectorXd
    Space2D::ComputeDdf(const Eigen::Ref<const Eigen::Matrix2Xd> &query_points, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel)
        const {

        ERL_ASSERTM(
            query_points.cols() == query_directions.cols(),
            "#query_points(%ld) != #query_directions(%ld).",
            query_points.cols(),
            query_directions.cols());

        // find the line segment that intersects the ray
        // auto filter_func = [](const Eigen::VectorXd &lams, const Eigen::VectorXd &ts) -> double {
        //     double ddf = std::numeric_limits<double>::infinity();
        //     for (int i = 0; i < lams.size(); ++i) {
        //         if (lams[i] <= 1. && lams[i] >= 0. && ts[i] >= 0. && ts[i] < ddf) {  // positive ddf only
        //             ddf = ts[i];
        //         }
        //     }
        //
        //     return ddf;
        // };

        Eigen::VectorXd ddf(query_points.cols());

#pragma omp parallel for if (parallel) default(none) shared(m_surface_, query_points, query_directions, ddf)
        for (int i = 0; i < query_points.cols(); ++i) {
            Eigen::Vector2d d = query_directions.col(i);
            d.normalize();
            auto n_lines = m_surface_->GetNumLines();
            // Eigen::VectorXd lams(n_lines);
            // Eigen::VectorXd ts(n_lines);
            double lam, t;
            ddf[i] = std::numeric_limits<double>::infinity();
            for (int j = 0; j < n_lines; ++j) {
                ComputeIntersectionBetweenRayAndSegment2D(
                    query_points.col(i),
                    d,
                    m_surface_->vertices.col(m_surface_->lines_to_vertices(0, j)),
                    m_surface_->vertices.col(m_surface_->lines_to_vertices(1, j)),
                    lam,
                    t);
                // lams[j],
                // ts[j]);

                if (lam <= 1. && lam >= 0. && t >= 0. && t < ddf[i]) { ddf[i] = t; }  // positive ddf only
            }
            // ddf[i] = filter_func(lams, ts);
        }

        return ddf;
    }

    Eigen::MatrixX<Eigen::VectorXd>
    Space2D::ComputeSddfV1(const common::GridMapInfo2D &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel) const {

        long n_directions = query_directions.cols();
        int height = grid_map_info.Height();
        int width = grid_map_info.Width();
        Eigen::MatrixX<Eigen::VectorXd> out(grid_map_info.Height(), grid_map_info.Width());

#pragma omp parallel for if (parallel) default(none) shared(n_directions, height, width, grid_map_info, query_directions, out)
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                Eigen::Vector2d point = grid_map_info.PixelToMeterForPoints(Eigen::Vector2i(u, v));
                out(v, u) = ComputeSddfV1(point.replicate(1, n_directions), query_directions, false);
            }
        }

        return out;
    }

    Eigen::VectorXd
    Space2D::ComputeSddfV1(const Eigen::Ref<const Eigen::Matrix2Xd> &query_points, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel)
        const {

        ERL_ASSERTM(
            query_points.cols() == query_directions.cols(),
            "#query_points(%ld) != #query_directions(%ld).",
            query_points.cols(),
            query_directions.cols());

        // auto filter_func = [](const Eigen::VectorXd &lams, const Eigen::VectorXd &ts) -> double {
        //     double ddf = std::numeric_limits<double>::infinity();
        //     for (int i = 0; i < lams.size(); ++i) {
        //         if (lams[i] <= 1. && lams[i] >= 0. && ts[i] < ddf) {  // min_t(p + t * v == surface_point)
        //             ddf = ts[i];
        //         }
        //     }
        //
        //     return ddf;
        // };

        Eigen::VectorXd ddf(query_points.cols());

#pragma omp parallel for if (parallel) default(none) shared(m_surface_, query_points, query_directions, ddf)
        for (int i = 0; i < query_points.cols(); ++i) {
            Eigen::Vector2d d = query_directions.col(i);
            d.normalize();
            auto n_lines = m_surface_->GetNumLines();
            // Eigen::VectorXd lams(n_lines);
            // Eigen::VectorXd ts(n_lines);
            double lam, t;
            ddf[i] = std::numeric_limits<double>::infinity();
            for (int j = 0; j < n_lines; ++j) {
                auto l = m_surface_->lines_to_vertices.col(j);
                ComputeIntersectionBetweenRayAndSegment2D(query_points.col(i), d, m_surface_->vertices.col(l.x()), m_surface_->vertices.col(l.y()), lam, t);
                // lams[j],
                // ts[j]);

                if (lam <= 1. && lam >= 0. && t < ddf[i]) { ddf[i] = t; }  // min_t(p + t * v == surface_point)
            }
            // ddf[i] = filter_func(lams, ts);
        }

        return ddf;
    }

    Eigen::MatrixX<Eigen::VectorXd>
    Space2D::ComputeSddfV2(
        const common::GridMapInfo2D &grid_map_info,
        const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
        SignMethod sign_method,
        bool parallel) const {

        long n_directions = query_directions.cols();
        int height = grid_map_info.Height();
        int width = grid_map_info.Width();
        Eigen::MatrixX<Eigen::VectorXd> out(grid_map_info.Height(), grid_map_info.Width());

#pragma omp parallel for if (parallel) default(none) shared(n_directions, height, width, grid_map_info, query_directions, sign_method, out)
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                Eigen::Vector2d point = grid_map_info.PixelToMeterForPoints(Eigen::Vector2i(u, v));
                out(v, u) = ComputeSddfV2(point.replicate(1, n_directions), query_directions, sign_method, false);
            }
        }

        return out;
    }

    Eigen::VectorXd
    Space2D::ComputeSddfV2(
        const Eigen::Ref<const Eigen::Matrix2Xd> &query_points,
        const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
        SignMethod sign_method,
        bool parallel) const {

        ERL_ASSERTM(
            query_points.cols() == query_directions.cols(),
            "#query_points(%ld) != #query_directions(%ld).",
            query_points.cols(),
            query_directions.cols());

        auto filter_func = [](const Eigen::VectorXd &lams, const Eigen::VectorXd &ts, bool is_negative) -> double {
            std::optional<double> sddf;
            for (int i = 0; i < lams.size(); ++i) {
                auto &kLam = lams[i];
                auto &kT = ts[i];

                if (kLam <= 1. && kLam >= 0.) {
                    if (is_negative) {
                        if (kT <= 0.) {
                            if (!sddf.has_value() || -kT < std::abs(sddf.value())) { sddf = kT; }
                        }
                    } else {
                        if (kT >= 0.) {
                            if (!sddf.has_value() || kT < sddf.value()) { sddf = kT; }
                        }
                    }
                }
            }
            if (!sddf.has_value()) { sddf = std::numeric_limits<double>::infinity(); }

            return sddf.value();
        };

        Eigen::VectorXd sddf(query_points.cols());
        auto n_lines = m_surface_->GetNumLines();

#pragma omp parallel for if (parallel) default(none) shared(m_surface_, query_points, query_directions, sign_method, filter_func, sddf, n_lines)
        for (int i = 0; i < query_points.cols(); ++i) {
            Eigen::Vector2d d = query_directions.col(i);
            d.normalize();

            Eigen::Vector2d q = query_points.col(i);

            Eigen::VectorXd lams(n_lines);
            Eigen::VectorXd ts(n_lines);
            bool is_negative = false;
            double min_dist = std::numeric_limits<double>::infinity();
            for (int j = 0; j < n_lines; ++j) {
                Eigen::Vector2i l = m_surface_->lines_to_vertices.col(j);
                const auto &v_1 = m_surface_->vertices.col(l.x());
                const auto &v_2 = m_surface_->vertices.col(l.y());
                ComputeIntersectionBetweenRayAndSegment2D(q, d, v_1, v_2, lams[j], ts[j]);

                Eigen::Vector2d qv_1 = q - v_1;
                auto dist_1 = qv_1.norm();

                Eigen::Vector2d qv_2 = q - v_2;
                auto dist_2 = qv_2.norm();

                if (dist_2 < dist_1) {
                    std::swap(dist_1, dist_2);
                    std::swap(l.x(), l.y());
                }

                if (dist_1 < min_dist) {
                    min_dist = dist_1;
                    is_negative = IsNegativeSdf(sign_method, q, l.x(), l.y());
                }
            }
            sddf[i] = filter_func(lams, ts, is_negative);
        }

        return sddf;
    }

    void
    Space2D::ComputeNormals(double delta, bool use_kdtree, bool parallel) {
        auto n_vtx = m_surface_->GetNumVertices();

        Eigen::Matrix2Xd query_points(2, 4 * n_vtx);  // px-, py-, px+, py+
        Eigen::Matrix24d deltas;
        // clang-format off
            deltas << -delta, 0, +delta, 0,
                      0, -delta, 0, +delta;
        // clang-format on
        for (int i = 0; i < (int) n_vtx; ++i) { query_points.block<2, 4>(0, i * 4) << (deltas.colwise() + m_surface_->vertices.col(i)); }
        Eigen::Matrix4Xd sdf = ComputeSdf(query_points, SignMethod::kPolygon, use_kdtree, parallel).reshaped(4, n_vtx);
        m_surface_->normals = ((sdf.block(2, 0, 2, n_vtx) - sdf.block(0, 0, 2, n_vtx)) / (delta * 2.));
        m_surface_->normals.colwise().normalize();
    }

    void
    Space2D::ComputeOutsideFlags(const Eigen::Ref<const Eigen::MatrixXd> &map_image, double free_threshold) {
        auto n_obj = m_surface_->GetNumObjects();
        m_surface_->outside_flags.resize(n_obj);
        for (int i = 0; i < n_obj; ++i) {
            auto vertices = m_surface_->GetObjectVertices(i);
            Eigen::Vector2d p = vertices.rowwise().mean();
            ssize_t u = std::lround(p[0]);
            ssize_t v = std::lround(p[1]);
            bool inside = WindingNumber(p, vertices);
            m_surface_->outside_flags[i] = (inside == (map_image(v, u) <= free_threshold));
        }
    }

    void
    Space2D::ComputeOutsideFlags() {
        auto n_obj = m_surface_->GetNumObjects();
        m_surface_->outside_flags.resize(n_obj);
        for (int i = 0; i < n_obj; ++i) {
            auto vertices = m_surface_->GetObjectVertices(i);
            Eigen::Vector2d p = vertices.rowwise().mean();
            Eigen::Matrix2Xd v = p.replicate(1, vertices.cols()).array() - vertices.array();
            bool is_negative_sdf = (m_surface_->GetObjectNormals(i).array() * v.array()).mean() < 0.;
            bool inside = WindingNumber(p, vertices);
            m_surface_->outside_flags[i] = (inside == is_negative_sdf);
        }
    }
}  // namespace erl::geometry
