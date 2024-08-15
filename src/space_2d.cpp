#include "erl_geometry/space_2d.hpp"

#include "erl_geometry/intersection.hpp"
#include "erl_geometry/surface_2d.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace erl::geometry {

    Space2D::Space2D(
        const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_vertices,
        const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_normals) {

        const auto n_obj = static_cast<ssize_t>(ordered_object_vertices.size());
        ERL_ASSERTM(static_cast<ssize_t>(ordered_object_normals.size()) == n_obj, "#groups_of_normals != #objects.");

        ssize_t n_vtx = 0;
        ssize_t n_lines = 0;
        for (ssize_t i = 0; i < n_obj; ++i) {
            auto n = ordered_object_vertices[i].cols();
            ERL_ASSERTM(n >= 3, "#vertices({}) should >= 3 for object {}.", n, i);

            auto m = ordered_object_normals[i].cols();
            ERL_ASSERTM(m == n, "#vertices ({}) != #normals ({}).", n, m);

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
            const auto n = static_cast<int>(ordered_object_vertices[i].cols());

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

        m_surface_ = std::make_shared<Surface2D>(  //
            std::move(vertices),
            std::move(normals),
            std::move(lines_to_vertices),
            std::move(objects_to_lines),
            Eigen::VectorXb{});

        m_kdtree_ = std::make_unique<KdTree>(2, m_surface_->vertices);

        ComputeOutsideFlags();
    }

    Space2D::Space2D(
        const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_vertices,
        const Eigen::Ref<const Eigen::VectorXb> &outside_flags,
        const double delta,
        const bool parallel) {

        auto n_obj = static_cast<ssize_t>(ordered_object_vertices.size());
        ERL_ASSERTM(outside_flags.size() == n_obj, "#outside_flags({}) != #objects({}).", outside_flags.size(), n_obj);

        ssize_t n_vtx = 0;
        ssize_t n_lines = 0;
        for (ssize_t i = 0; i < n_obj; ++i) {
            auto n = ordered_object_vertices[i].cols();
            ERL_ASSERTM(n >= 3, "#vertices({}) should >= 3 for object {}.", n, i);
            n_vtx += n;
            n_lines += n;
        }

        Eigen::Matrix2Xd vertices(2, n_vtx);
        Eigen::Matrix2Xi lines_to_vertices(2, n_lines);
        Eigen::Matrix2Xi objects_to_lines(2, n_obj);
        int vertex_idx_0 = 0;
        int line_idx_0 = 0;
        for (ssize_t i = 0; i < n_obj; ++i) {
            const auto n = static_cast<int>(ordered_object_vertices[i].cols());

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

        m_kdtree_ = std::make_unique<KdTree>(2, m_surface_->vertices);

        ComputeNormals(delta, false, parallel);  // don't use kd-tree because vertices are not always dense
    }

    Space2D::Space2D(
        const Eigen::Ref<const Eigen::MatrixXd> &map_image,  // y up, x right
        const common::GridMapInfo2D &grid_map_info,
        const double free_threshold,
        double delta,
        const bool parallel) {

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
        m_kdtree_ = std::make_unique<KdTree>(2, m_surface_->vertices);
        ComputeNormals(delta, true, parallel);  // use kd-tree because vertices generated by marching square are dense
    }

    Space2D
    Space2D::AddObstacles(const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &obstacle_vertices, double delta, bool parallel) const {

        // in meters
        const long num_objects = m_surface_->GetNumObjects();
        const auto num_obstacles = static_cast<long>(obstacle_vertices.size());
        std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> ordered_object_vertices;
        ordered_object_vertices.reserve(num_objects + num_obstacles);
        Eigen::VectorXb outside_flags(num_objects + num_obstacles);
        for (long i = 0; i < num_objects; ++i) {
            ordered_object_vertices.emplace_back(m_surface_->GetObjectVertices(static_cast<int>(i)));
            outside_flags[i] = m_surface_->outside_flags[i];
        }

        for (long i = 0; i < num_obstacles; ++i) {
            ordered_object_vertices.push_back(obstacle_vertices[i]);
            outside_flags[num_objects + i] = true;
        }

        return {ordered_object_vertices, outside_flags, delta, parallel};
    }

    Eigen::MatrixX8U
    Space2D::GenerateMapImage(const common::GridMapInfo2D &grid_map_info, const bool anti_aliased) const {
        cv::Mat cv_map_image = cv::Mat::zeros(grid_map_info.Height(), grid_map_info.Width(), CV_8UC1);
        const long n_obj = m_surface_->GetNumObjects();
        for (int i = 0; i < n_obj; ++i) {
            std::vector<std::vector<cv::Point>> contours(1);
            Eigen::Matrix2Xi vertices = grid_map_info.MeterToPixelForPoints(m_surface_->GetObjectVertices(i));
            const long n_vertices = vertices.cols();
            for (int j = 0; j < n_vertices; ++j) { contours[0].emplace_back(vertices(0, j), vertices(1, j)); }
            cv::drawContours(cv_map_image, contours, 0, m_surface_->outside_flags[i] ? 0 : 255, -1, anti_aliased ? cv::LINE_8 : cv::LINE_AA);
        }
        Eigen::MatrixX8U map_image(grid_map_info.Height(), grid_map_info.Width());  // map image: x-axis to right, y-axis to top
        cv::cv2eigen(cv_map_image, map_image);
        return map_image;
    }

    Eigen::MatrixXd
    Space2D::ComputeSdfImage(const common::GridMapInfo2D &grid_map_info, const SignMethod sign_method, const bool use_kdtree, const bool parallel) const {
        Eigen::MatrixXd sdf(grid_map_info.Height(), grid_map_info.Width());  // y up, x right, column-major

#pragma omp parallel for if (parallel) default(none) shared(grid_map_info, sign_method, use_kdtree, sdf)
        for (int v = 0; v < grid_map_info.Height(); ++v) {
            for (int u = 0; u < grid_map_info.Width(); ++u) {
                Eigen::Vector2d q = grid_map_info.PixelToMeterForPoints(Eigen::Vector2i(u, v));
                sdf(v, u) = use_kdtree ? ComputeSdfWithKdtree(q, sign_method) : ComputeSdfGreedily(q, sign_method);
            }
        }
        return sdf;
    }

    Eigen::VectorXd
    Space2D::ComputeSdf(const Eigen::Ref<const Eigen::Matrix2Xd> &query_points, const SignMethod sign_method, const bool use_kdtree, const bool parallel)
        const {

        Eigen::VectorXd sdf(query_points.cols());

#pragma omp parallel for if (parallel) default(none) shared(m_surface_, query_points, use_kdtree, sign_method, m_kdtree_, sdf, Eigen::Dynamic)
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
    Space2D::ComputeSdfWithKdtree(const Eigen::Vector2d &q, const SignMethod sign_method) const {
        double sdf;
        long idx_vertex_0 = 0, idx_vertex_1 = 0;
        m_kdtree_->query(q.data(), 1, &idx_vertex_0, &sdf);
        sdf = std::sqrt(sdf);

        const Eigen::Vector2d v0 = m_surface_->vertices.col(idx_vertex_0);
        const auto [idx_neighbor1, idx_neighbor2] = m_surface_->GetVertexNeighbors(static_cast<int>(idx_vertex_0));

        const auto &v1 = m_surface_->vertices.col(idx_neighbor1);
        const auto &v2 = m_surface_->vertices.col(idx_neighbor2);

        // compute the distances to the two nearby line segments
        const double dist_1 = ComputeNearestDistanceFromPointToLineSegment2D(q.x(), q.y(), v0.x(), v0.y(), v1.x(), v1.y());
        const double dist_2 = ComputeNearestDistanceFromPointToLineSegment2D(q.x(), q.y(), v0.x(), v0.y(), v2.x(), v2.y());

        if (dist_1 < sdf) {
            sdf = dist_1;
            idx_vertex_1 = idx_neighbor1;
        }
        if (dist_2 < sdf) {
            sdf = dist_2;
            idx_vertex_1 = idx_neighbor2;
        }

        if (IsNegativeSdf(sign_method, q, static_cast<int>(idx_vertex_0), static_cast<int>(idx_vertex_1))) { sdf = -sdf; }

        return sdf;
    }

    /**
     * iterate over all line segments, this is slower but accurate
     * @param q
     * @param sign_method
     * @return
     */
    double
    Space2D::ComputeSdfGreedily(const Eigen::Ref<const Eigen::Vector2d> &q, const SignMethod sign_method) const {
        double sdf = std::numeric_limits<double>::infinity();
        int idx_vertex_0 = 0, idx_vertex_1 = 0;

        for (int j = 1; j < m_surface_->GetNumLines(); ++j) {
            const auto &line = m_surface_->lines_to_vertices.col(j);
            const auto &v0 = m_surface_->vertices.col(line.x());
            const auto &v1 = m_surface_->vertices.col(line.y());
            if (const double dist = ComputeNearestDistanceFromPointToLineSegment2D(q.x(), q.y(), v0.x(), v0.y(), v1.x(), v1.y()); dist < sdf) {
                sdf = dist;
                idx_vertex_0 = line.x();
                idx_vertex_1 = line.y();
            }
        }

        if (IsNegativeSdf(sign_method, q, idx_vertex_0, idx_vertex_1)) { sdf = -sdf; }

        return sdf;
    }

    Eigen::MatrixX<Eigen::VectorXd>
    Space2D::ComputeDdf(const common::GridMapInfo2D &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, const bool parallel) const {
        Eigen::MatrixX<Eigen::VectorXd> out(grid_map_info.Height(), grid_map_info.Width());

#pragma omp parallel for if (parallel) default(none) shared(grid_map_info, query_directions, out, Eigen::Dynamic)
        for (int v = 0; v < grid_map_info.Height(); ++v) {
            for (int u = 0; u < grid_map_info.Width(); ++u) {
                Eigen::Vector2d point = grid_map_info.PixelToMeterForPoints(Eigen::Vector2i(u, v));
                out(v, u) = ComputeDdf(point.replicate(1, query_directions.cols()), query_directions, false);
            }
        }

        return out;
    }

    Eigen::VectorXd
    Space2D::ComputeDdf(const Eigen::Ref<const Eigen::Matrix2Xd> &query_points, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, const bool parallel)
        const {

        ERL_ASSERTM(
            query_points.cols() == query_directions.cols(),
            "#query_points({}) != #query_directions({}).",
            query_points.cols(),
            query_directions.cols());

        Eigen::VectorXd ddf(query_points.cols());

#pragma omp parallel for if (parallel) default(none) shared(m_surface_, query_points, query_directions, ddf)
        for (int i = 0; i < query_points.cols(); ++i) {
            Eigen::Vector2d d = query_directions.col(i);
            d.normalize();
            const long n_lines = m_surface_->GetNumLines();
            double lam, t;
            bool intersected;
            ddf[i] = std::numeric_limits<double>::infinity();
            for (int j = 0; j < n_lines; ++j) {
                ComputeIntersectionBetweenRayAndLine2D(
                    query_points.col(i),
                    d,
                    m_surface_->vertices.col(m_surface_->lines_to_vertices(0, j)),
                    m_surface_->vertices.col(m_surface_->lines_to_vertices(1, j)),
                    lam,
                    t,
                    intersected);
                if (intersected && lam <= 1. && lam >= 0. && t >= 0. && t < ddf[i]) { ddf[i] = t; }  // positive ddf only
            }
        }

        return ddf;
    }

    Eigen::MatrixX<Eigen::VectorXd>
    Space2D::ComputeSddfV1(const common::GridMapInfo2D &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, const bool parallel) const {

        const long n_directions = query_directions.cols();
        const int height = grid_map_info.Height();
        const int width = grid_map_info.Width();
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
            "#query_points({}) != #query_directions({}).",
            query_points.cols(),
            query_directions.cols());

        Eigen::VectorXd ddf(query_points.cols());

#pragma omp parallel for if (parallel) default(none) shared(m_surface_, query_points, query_directions, ddf)
        for (int i = 0; i < query_points.cols(); ++i) {
            Eigen::Vector2d d = query_directions.col(i);
            d.normalize();
            const long n_lines = m_surface_->GetNumLines();
            double lam, t;
            bool intersected;
            ddf[i] = std::numeric_limits<double>::infinity();
            for (int j = 0; j < n_lines; ++j) {
                auto l = m_surface_->lines_to_vertices.col(j);
                ComputeIntersectionBetweenRayAndLine2D(
                    query_points.col(i),
                    d,
                    m_surface_->vertices.col(l.x()),
                    m_surface_->vertices.col(l.y()),
                    lam,
                    t,
                    intersected);
                if (intersected && lam <= 1. && lam >= 0. && t < ddf[i]) { ddf[i] = t; }  // min_t(p + t * v == surface_point)
            }
        }

        return ddf;
    }

    Eigen::MatrixX<Eigen::VectorXd>
    Space2D::ComputeSddfV2(
        const common::GridMapInfo2D &grid_map_info,
        const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
        const SignMethod sign_method,
        const bool parallel) const {

        const long n_directions = query_directions.cols();
        const int height = grid_map_info.Height();
        const int width = grid_map_info.Width();
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
        const SignMethod sign_method,
        const bool parallel) const {

        ERL_ASSERTM(
            query_points.cols() == query_directions.cols(),
            "#query_points({}) != #query_directions({}).",
            query_points.cols(),
            query_directions.cols());
        Eigen::VectorXd sddf(query_points.cols());

#pragma omp parallel for if (parallel) default(none) shared(m_surface_, query_points, query_directions, sign_method, sddf, Eigen::Dynamic)
        for (int i = 0; i < query_points.cols(); ++i) {
            Eigen::Vector2d d = query_directions.col(i);
            d.normalize();

            Eigen::Vector2d q = query_points.col(i);

            const long n_lines = m_surface_->GetNumLines();
            Eigen::VectorXd lams(n_lines);
            Eigen::VectorXd ts(n_lines);
            bool is_negative = false;
            double min_dist = std::numeric_limits<double>::infinity();
            for (int j = 0; j < n_lines; ++j) {
                Eigen::Vector2i l = m_surface_->lines_to_vertices.col(j);
                const auto &v_1 = m_surface_->vertices.col(l.x());
                const auto &v_2 = m_surface_->vertices.col(l.y());
                bool intersected;
                ComputeIntersectionBetweenRayAndLine2D(q, d, v_1, v_2, lams[j], ts[j], intersected);

                Eigen::Vector2d qv_1 = q - v_1;
                auto dist_1 = qv_1.norm();
                Eigen::Vector2d qv_2 = q - v_2;
                if (double dist_2 = qv_2.norm(); dist_2 < dist_1) {
                    std::swap(dist_1, dist_2);
                    std::swap(l.x(), l.y());
                }

                if (dist_1 < min_dist) {
                    min_dist = dist_1;
                    is_negative = IsNegativeSdf(sign_method, q, l.x(), l.y());
                }
            }

            std::optional<double> sddf_opt;
            for (int k = 0; k < lams.size(); ++k) {
                const double &lam = lams[k];
                const double &t = ts[k];
                if (lam <= 1. && lam >= 0.) {
                    if (is_negative) {
                        if (t <= 0.) {
                            if (!sddf_opt.has_value() || -t < std::abs(sddf_opt.value())) { sddf_opt = t; }
                        }
                    } else {
                        if (t >= 0.) {
                            if (!sddf_opt.has_value() || t < sddf_opt.value()) { sddf_opt = t; }
                        }
                    }
                }
            }
            if (!sddf_opt.has_value()) { sddf_opt = std::numeric_limits<double>::infinity(); }
            sddf[i] = sddf_opt.value();
        }

        return sddf;
    }

    void
    Space2D::ComputeNormals(const double delta, const bool use_kdtree, const bool parallel) const {
        const long n_vtx = m_surface_->GetNumVertices();

        Eigen::Matrix2Xd query_points(2, 4 * n_vtx);  // px-, py-, px+, py+
        Eigen::Matrix24d deltas;
        // clang-format off
            deltas << -delta, 0, +delta, 0,
                      0, -delta, 0, +delta;
        // clang-format on
        for (int i = 0; i < static_cast<int>(n_vtx); ++i) { query_points.block<2, 4>(0, i * 4) << deltas.colwise() + m_surface_->vertices.col(i); }
        Eigen::Matrix4Xd sdf = ComputeSdf(query_points, SignMethod::kPolygon, use_kdtree, parallel).reshaped(4, n_vtx);
        m_surface_->normals = (sdf.block(2, 0, 2, n_vtx) - sdf.block(0, 0, 2, n_vtx)) / (delta * 2.);
        m_surface_->normals.colwise().normalize();
    }

    void
    Space2D::ComputeOutsideFlags(const Eigen::Ref<const Eigen::MatrixXd> &map_image, const double free_threshold) const {
        const long n_obj = m_surface_->GetNumObjects();
        m_surface_->outside_flags.resize(n_obj);
        for (int i = 0; i < n_obj; ++i) {
            auto vertices = m_surface_->GetObjectVertices(i);
            Eigen::Vector2d p = vertices.rowwise().mean();
            const ssize_t u = std::lround(p[0]);
            const ssize_t v = std::lround(p[1]);
            const bool inside = WindingNumber(p, vertices);
            m_surface_->outside_flags[i] = inside == (map_image(v, u) <= free_threshold);
        }
    }

    void
    Space2D::ComputeOutsideFlags() const {
        const long n_obj = m_surface_->GetNumObjects();
        m_surface_->outside_flags.resize(n_obj);
        for (int i = 0; i < n_obj; ++i) {
            auto vertices = m_surface_->GetObjectVertices(i);
            Eigen::Vector2d p = vertices.rowwise().mean();
            Eigen::Matrix2Xd v = p.replicate(1, vertices.cols()).array() - vertices.array();
            bool &&is_negative_sdf = (m_surface_->GetObjectNormals(i).array() * v.array()).mean() < 0.;
            bool &&inside = WindingNumber(p, vertices);
            m_surface_->outside_flags[i] = inside == is_negative_sdf;
        }
    }
}  // namespace erl::geometry
