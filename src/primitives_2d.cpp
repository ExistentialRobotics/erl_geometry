#include "erl_geometry/primitives_2d.hpp"

#include "erl_geometry/intersection.hpp"

namespace erl::geometry {

    std::vector<Eigen::Vector2d>
    Line2D::ComputeIntersections(const Line2D &line) const {
        double lam1, lam2;
        bool intersected;
        ComputeIntersectionBetweenTwoLines2D(p0, p1, line.p0, line.p1, lam1, lam2, intersected);
        if (!intersected) { return {}; }
        return {p0 + lam1 * (p1 - p0)};
    }

    std::vector<Eigen::Vector2d>
    Line2D::ComputeIntersections(const Ray2D &ray) const {
        return ray.ComputeIntersections(*this);
    }

    std::vector<Eigen::Vector2d>
    Line2D::ComputeIntersections(const Segment2D &segment) const {
        return segment.ComputeIntersections(*this);
    }

    std::vector<Eigen::Vector2d>
    Segment2D::ComputeIntersections(const Line2D &line) const {
        double lam1, lam2;
        bool intersected;
        ComputeIntersectionBetweenTwoLines2D(p0, p1, line.p0, line.p1, lam1, lam2, intersected);
        if (!intersected) { return {}; }
        if (lam1 < 0 || lam1 > 1) { return {}; }
        return {p0 + lam1 * (p1 - p0)};
    }

    std::vector<Eigen::Vector2d>
    Segment2D::ComputeIntersections(const Segment2D &segment) const {
        double lam1, lam2;
        bool intersected;
        ComputeIntersectionBetweenTwoLines2D(p0, p1, segment.p0, segment.p1, lam1, lam2, intersected);
        if (!intersected) { return {}; }
        if (lam1 < 0 || lam1 > 1) { return {}; }
        if (lam2 < 0 || lam2 > 1) { return {}; }
        return {p0 + lam1 * (p1 - p0)};
    }

    std::vector<Eigen::Vector2d>
    Segment2D::ComputeIntersections(const Ray2D &ray) const {
        return ray.ComputeIntersections(*this);
    }

    std::vector<Eigen::Vector2d>
    Ray2D::ComputeIntersections(const Line2D &line) const {
        double lam, dist;
        bool intersected;
        ComputeIntersectionBetweenRayAndLine2D(origin, direction, line.p0, line.p1, lam, dist, intersected);
        if (!intersected) { return {}; }
        if (dist < 0) { return {}; }
        return {origin + dist * direction};
    }

    std::vector<Eigen::Vector2d>
    Ray2D::ComputeIntersections(const Segment2D &segment) const {
        double lam, dist;
        bool intersected;
        ComputeIntersectionBetweenRayAndLine2D(origin, direction, segment.p0, segment.p1, lam, dist, intersected);
        if (!intersected) { return {}; }
        if (lam < 0 || lam > 1) { return {}; }
        if (dist < 0) { return {}; }
        return {origin + dist * direction};
    }

    std::vector<Eigen::Vector2d>
    Ray2D::ComputeIntersections(const Ray2D &ray) const {
        double lam, dist;
        bool intersected;
        ComputeIntersectionBetweenRayAndLine2D(origin, direction, ray.origin, ray.origin + ray.direction, lam, dist, intersected);
        if (!intersected) { return {}; }
        if (dist < 0) { return {}; }
        if (lam < 0) { return {}; }
        return {origin + dist * direction};
    }

    bool
    AxisAlignedRectangle2D::IsOnBoundary(const Eigen::Vector2d &point) const {
        const auto [bl, tl, tr, br] = GetVertices();
        if (Segment2D(-1, Eigen::Vector2d(bl.x(), bl.y()), Eigen::Vector2d(tl.x(), tl.y())).IsOnBoundary(point)) { return true; }
        if (Segment2D(-1, Eigen::Vector2d(tl.x(), tl.y()), Eigen::Vector2d(tr.x(), tr.y())).IsOnBoundary(point)) { return true; }
        if (Segment2D(-1, Eigen::Vector2d(tr.x(), tr.y()), Eigen::Vector2d(br.x(), br.y())).IsOnBoundary(point)) { return true; }
        if (Segment2D(-1, Eigen::Vector2d(br.x(), br.y()), Eigen::Vector2d(bl.x(), bl.y())).IsOnBoundary(point)) { return true; }
        return false;
    }

    std::vector<Eigen::Vector2d>
    AxisAlignedRectangle2D::ComputeIntersections(const Line2D &line) const {
        double d1, d2;
        bool intersected, is_inside;
        Eigen::Vector2d v = line.p1 - line.p0;
        v.normalize();
        const Eigen::Vector2d v_inv = v.cwiseInverse();
        ComputeIntersectionBetweenRayAndAabb2D(line.p0, v_inv, m_min, m_max, d1, d2, intersected, is_inside);
        if (!intersected) { return {}; }
        if (std::abs(d1 - d2) < std::numeric_limits<double>::min()) { return {line.p0 + d1 * v}; }
        return {line.p0 + d1 * v, line.p0 + d2 * v};
    }

    std::vector<Eigen::Vector2d>
    AxisAlignedRectangle2D::ComputeIntersections(const Segment2D &segment) const {
        double d1, d2;
        bool intersected, is_inside;
        Eigen::Vector2d v = segment.p1 - segment.p0;
        v.normalize();
        const Eigen::Vector2d v_inv = v.cwiseInverse();
        ComputeIntersectionBetweenRayAndAabb2D(segment.p0, v_inv, m_min, m_max, d1, d2, intersected, is_inside);
        if (!intersected) { return {}; }
        std::vector<Eigen::Vector2d> intersections;
        if (d1 >= 0 && d1 <= 1) { intersections.emplace_back(segment.p0 + d1 * v); }
        if (std::abs(d1 - d2) < std::numeric_limits<double>::min()) { return intersections; }
        if (d2 >= 0 && d2 <= 1) { intersections.emplace_back(segment.p0 + d2 * v); }
        return intersections;
    }

    std::vector<Eigen::Vector2d>
    AxisAlignedRectangle2D::ComputeIntersections(const Ray2D &ray) const {
        double d1, d2;
        bool intersected, is_inside;
        const Eigen::Vector2d v_inv = ray.direction.cwiseInverse();
        ComputeIntersectionBetweenRayAndAabb2D(ray.origin, v_inv, m_min, m_max, d1, d2, intersected, is_inside);
        if (!intersected) { return {}; }
        std::vector<Eigen::Vector2d> intersections;
        if (d1 >= 0) { intersections.emplace_back(ray.origin + d1 * ray.direction); }
        if (std::abs(d1 - d2) < std::numeric_limits<double>::min()) { return intersections; }
        if (d2 >= 0) { intersections.emplace_back(ray.origin + d2 * ray.direction); }
        return intersections;
    }

    bool
    Rectangle2D::IsOnBoundary(const Eigen::Vector2d &point) const {
        const auto [bl, tl, tr, br] = GetVertices();
        if (Segment2D(-1, Eigen::Vector2d(bl.x(), bl.y()), Eigen::Vector2d(tl.x(), tl.y())).IsOnBoundary(point)) { return true; }
        if (Segment2D(-1, Eigen::Vector2d(tl.x(), tl.y()), Eigen::Vector2d(tr.x(), tr.y())).IsOnBoundary(point)) { return true; }
        if (Segment2D(-1, Eigen::Vector2d(tr.x(), tr.y()), Eigen::Vector2d(br.x(), br.y())).IsOnBoundary(point)) { return true; }
        if (Segment2D(-1, Eigen::Vector2d(br.x(), br.y()), Eigen::Vector2d(bl.x(), bl.y())).IsOnBoundary(point)) { return true; }
        return false;
    }

    std::vector<Eigen::Vector2d>
    Rectangle2D::ComputeIntersections(const Line2D &line) const {
        const Eigen::Vector2d p0 = m_rotation_matrix_.transpose() * (line.p0 - m_center_);
        Eigen::Vector2d v = m_rotation_matrix_.transpose() * (line.p1 - line.p0);
        v.normalize();

        double d1, d2;
        bool intersected, is_inside;
        ComputeIntersectionBetweenRayAndAabb2D(p0, v.cwiseInverse(), -m_half_sizes_, m_half_sizes_, d1, d2, intersected, is_inside);
        if (!intersected) { return {}; }
        if (std::abs(d1 - d2) < std::numeric_limits<double>::min()) { return {line.p0 + d1 * v}; }
        return {line.p0 + d1 * v, line.p0 + d2 * v};
    }

    std::vector<Eigen::Vector2d>
    Rectangle2D::ComputeIntersections(const Segment2D &segment) const {
        const Eigen::Vector2d p0 = m_rotation_matrix_.transpose() * (segment.p0 - m_center_);
        Eigen::Vector2d v = m_rotation_matrix_.transpose() * (segment.p1 - segment.p0);
        v.normalize();

        double d1, d2;
        bool intersected, is_inside;
        ComputeIntersectionBetweenRayAndAabb2D(p0, v.cwiseInverse(), -m_half_sizes_, m_half_sizes_, d1, d2, intersected, is_inside);
        if (!intersected) { return {}; }
        std::vector<Eigen::Vector2d> intersections;
        if (d1 >= 0 && d1 <= 1) { intersections.emplace_back(segment.p0 + d1 * v); }
        if (std::abs(d1 - d2) < std::numeric_limits<double>::min()) { return intersections; }
        if (d2 >= 0 && d2 <= 1) { intersections.emplace_back(segment.p0 + d2 * v); }
        return intersections;
    }

    std::vector<Eigen::Vector2d>
    Rectangle2D::ComputePointsOnBoundary(std::size_t num_points) const {
        std::vector<Eigen::Vector2d> points;
        points.reserve(num_points);
        const std::size_t num_points_x = std::lround(static_cast<double>(num_points) * m_half_sizes_.x() / (2.0 * m_half_sizes_.x() + m_half_sizes_.y()));
        const std::size_t num_points_y = num_points / 2 - num_points_x;
        const double step_x = 2.0 * m_half_sizes_.x() / static_cast<double>(num_points_x - 1);
        const double step_y = 2.0 * m_half_sizes_.y() / static_cast<double>(num_points_y - 1);
        double shift = 0;
        for (std::size_t i = 0; i < num_points_y; ++i) {
            const Eigen::Vector2d p(-m_half_sizes_.x(), -m_half_sizes_.y() + shift);
            points.emplace_back(m_rotation_matrix_ * p + m_center_);
            shift += step_y;
        }
        shift = 0;
        for (std::size_t i = 0; i < num_points_x; ++i) {
            const Eigen::Vector2d p(-m_half_sizes_.x() + shift, m_half_sizes_.y());
            points.emplace_back(m_rotation_matrix_ * p + m_center_);
            shift += step_x;
        }
        shift = 0;
        for (std::size_t i = 0; i < num_points_y; ++i) {
            const Eigen::Vector2d p(m_half_sizes_.x(), m_half_sizes_.y() - shift);
            points.emplace_back(m_rotation_matrix_ * p + m_center_);
            shift += step_y;
        }
        shift = 0;
        for (std::size_t i = 0; i < num_points_x; ++i) {
            const Eigen::Vector2d p(m_half_sizes_.x() - shift, -m_half_sizes_.y());
            points.emplace_back(m_rotation_matrix_ * p + m_center_);
            shift += step_x;
        }
        return points;
    }

    std::vector<Eigen::Vector2d>
    Ellipse2D::ComputeIntersections(const Line2D &line) const {
        Eigen::Vector2d p0 = m_rotation_matrix_.transpose() * (line.p0 - m_center_);
        Eigen::Vector2d p1 = m_rotation_matrix_.transpose() * (line.p1 - m_center_);

        double lam1, lam2;
        bool intersected;
        ComputeIntersectionBetweenLineAndEllipse2D(p0.x(), p0.y(), p1.x(), p1.y(), m_radii_.x(), m_radii_.y(), lam1, lam2, intersected);
        if (!intersected) { return {}; }
        const Eigen::Vector2d p10 = line.p1 - line.p0;
        if (std::abs(lam1 - lam2) < std::numeric_limits<double>::min()) { return {line.p0 + lam1 * p10}; }
        return {line.p0 + lam1 * p10, line.p0 + lam2 * p10};
    }

    std::vector<Eigen::Vector2d>
    Ellipse2D::ComputeIntersections(const Segment2D &segment) const {
        Eigen::Vector2d p0 = m_rotation_matrix_.transpose() * (segment.p0 - m_center_);
        Eigen::Vector2d p1 = m_rotation_matrix_.transpose() * (segment.p1 - m_center_);

        double lam1, lam2;
        bool intersected;
        ComputeIntersectionBetweenLineAndEllipse2D(p0.x(), p0.y(), p1.x(), p1.y(), m_radii_.x(), m_radii_.y(), lam1, lam2, intersected);
        if (!intersected) { return {}; }
        std::vector<Eigen::Vector2d> intersections;
        if (lam1 >= 0 && lam1 <= 1) { intersections.emplace_back(segment.p0 + lam1 * (segment.p1 - segment.p0)); }
        if (std::abs(lam1 - lam2) < std::numeric_limits<double>::min()) { return intersections; }
        if (lam2 >= 0 && lam2 <= 1) { intersections.emplace_back(segment.p0 + lam2 * (segment.p1 - segment.p0)); }
        return intersections;
    }

    std::vector<Eigen::Vector2d>
    Ellipse2D::ComputeIntersections(const Ray2D &ray) const {
        Eigen::Vector2d p0 = m_rotation_matrix_.transpose() * (ray.origin - m_center_);
        Eigen::Vector2d v = m_rotation_matrix_.transpose() * ray.direction;

        double lam1, lam2;
        bool intersected;
        ComputeIntersectionBetweenLineAndEllipse2D(p0.x(), p0.y(), p0.x() + v.x(), p0.y() + v.y(), m_radii_.x(), m_radii_.y(), lam1, lam2, intersected);
        if (!intersected) { return {}; }
        std::vector<Eigen::Vector2d> intersections;
        if (lam1 >= 0) { intersections.emplace_back(ray.origin + lam1 * ray.direction); }
        if (std::abs(lam1 - lam2) < std::numeric_limits<double>::min()) { return intersections; }
        if (lam2 >= 0) { intersections.emplace_back(ray.origin + lam2 * ray.direction); }
        return intersections;
    }

    std::vector<Eigen::Vector2d>
    Ellipse2D::ComputePointsOnBoundary(const std::size_t num_points, const double start_angle, const double end_angle) const {
        std::vector<Eigen::Vector2d> points;
        points.reserve(num_points);
        const double step = (end_angle - start_angle) / static_cast<double>(num_points - 1);
        double theta = start_angle;
        for (std::size_t i = 0; i < num_points; ++i) {
            Eigen::Vector2d point(m_radii_.x() * std::cos(theta), m_radii_.y() * std::sin(theta));
            points.emplace_back(m_rotation_matrix_ * point + m_center_);
            theta += step;
        }
        return points;
    }

}  // namespace erl::geometry
