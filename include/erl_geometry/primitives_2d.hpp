#pragma once

#include "aabb.hpp"

#include <utility>

namespace erl::geometry {

    struct Line2D;

    struct Segment2D;

    struct Ray2D;

    struct Primitive2D {

        enum class Type {
            kLine2D = 0,
            kSegment2D = 1,
            kRay2D = 2,
            kAxisAlignedRectangle = 3,
            kRectangle = 4,
            kEllipse = 5,
        };

        int id = -1;

        virtual ~Primitive2D() = default;

        [[nodiscard]] virtual Type
        GetType() const = 0;

        [[nodiscard]] virtual bool
        IsInside(const Eigen::Vector2d &point) const = 0;

        [[nodiscard]] virtual bool
        IsOnBoundary(const Eigen::Vector2d &point) const = 0;

        [[nodiscard]] virtual std::vector<Eigen::Vector2d>
        ComputeIntersections(const Line2D &line) const = 0;

        [[nodiscard]] virtual std::vector<Eigen::Vector2d>
        ComputeIntersections(const Segment2D &segment) const = 0;

        [[nodiscard]] virtual std::vector<Eigen::Vector2d>
        ComputeIntersections(const Ray2D &ray) const = 0;

        [[nodiscard]] virtual double
        GetOrientationAngle() const = 0;
    };

    struct Line2D : Primitive2D {
        Eigen::Vector2d p0 = {0.0, 0.0};
        Eigen::Vector2d p1 = {0.0, 0.0};

        Line2D(const int id, Eigen::Vector2d p0, Eigen::Vector2d p1)
            : p0(std::move(p0)),
              p1(std::move(p1)) {
            this->id = id;
        }

        [[nodiscard]] Type
        GetType() const override {
            return Type::kLine2D;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector2d &point) const override {
            return IsOnBoundary(point);
        }

        [[nodiscard]] bool
        IsOnBoundary(const Eigen::Vector2d &point) const override {
            if (const Eigen::Vector2d v0 = p1 - p0, v1 = point - p0; std::abs(v0.x() * v1.y() - v0.y() * v1.x()) == 0) { return true; }
            return false;
        }

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Line2D &line) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Segment2D &segment) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Ray2D &ray) const override;

        [[nodiscard]] double
        GetOrientationAngle() const override {
            return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
        }
    };

    struct Segment2D : Line2D {

        Segment2D(const int id, Eigen::Vector2d p0, Eigen::Vector2d p1)
            : Line2D(id, std::move(p0), std::move(p1)) {}

        [[nodiscard]] Type
        GetType() const override {
            return Type::kSegment2D;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector2d &point) const override {
            return IsOnBoundary(point);
        }

        [[nodiscard]] bool
        IsOnBoundary(const Eigen::Vector2d &point) const override {
            if (const Eigen::Vector2d v0 = p1 - p0, v1 = point - p0; std::abs(v0.x() * v1.y() - v0.y() * v1.x()) == 0) {
                const double lam = v1.norm() / v0.norm();
                return lam >= 0 && lam <= 1;
            }
            return false;
        }

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Line2D &line) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Segment2D &segment) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Ray2D &ray) const override;
    };

    struct Ray2D : Primitive2D {
        Eigen::Vector2d origin;
        Eigen::Vector2d direction;

        Ray2D(const int id, Eigen::Vector2d origin, Eigen::Vector2d direction)
            : origin(std::move(origin)),
              direction(std::move(direction)) {
            this->id = id;
        }

        [[nodiscard]] Type
        GetType() const override {
            return Type::kRay2D;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector2d &point) const override {
            return IsOnBoundary(point);
        }

        [[nodiscard]] bool
        IsOnBoundary(const Eigen::Vector2d &point) const override {
            if (const Eigen::Vector2d v1 = point - origin; std::abs(direction.x() * v1.y() - direction.y() * v1.x()) == 0) { return true; }
            return false;
        }

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Line2D &line) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Segment2D &segment) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Ray2D &ray) const override;

        [[nodiscard]] double
        GetOrientationAngle() const override {
            return std::atan2(direction.y(), direction.x());
        }
    };

    class AxisAlignedRectangle2D : public Primitive2D, public Aabb2D {

    public:
        AxisAlignedRectangle2D(const int id, const Eigen::Vector2d &center, const Eigen::Vector2d &half_sizes)
            : Aabb2D(center - half_sizes, center + half_sizes) {
            this->id = id;
        }

        [[nodiscard]] Type
        GetType() const override {
            return Type::kAxisAlignedRectangle;
        }

        [[nodiscard]] const Eigen::Vector2d &
        GetCenter() const {
            return center;
        }

        [[nodiscard]] double
        GetWidth() const {
            return half_sizes.x() * 2;
        }

        [[nodiscard]] double
        GetHeight() const {
            return half_sizes.y() * 2;
        }

        [[nodiscard]] double
        GetOrientationAngle() const override {
            return 0;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector2d &point) const override {
            return (m_min.array() <= point.array()).all() && (point.array() <= m_max.array()).all();
        }

        [[nodiscard]] bool
        IsOnBoundary(const Eigen::Vector2d &point) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Line2D &line) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Segment2D &segment) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Ray2D &ray) const override;

        /**
         *
         * @return Vertices of the rectangle in clockwise order starting from the bottom-left corner.
         */
        [[nodiscard]] std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d>
        GetVertices() const {
            return {
                Eigen::Vector2d(m_min.x(), m_min.y()),
                Eigen::Vector2d(m_min.x(), m_max.y()),
                Eigen::Vector2d(m_max.x(), m_max.y()),
                Eigen::Vector2d(m_max.x(), m_min.y()),
            };
        }

        /**
         *
         * @return Edges of the rectangle in clockwise order starting from the left vertical edge.
         */
        [[nodiscard]] std::tuple<Segment2D, Segment2D, Segment2D, Segment2D>
        GetEdges() const {
            const auto [bl, tl, tr, br] = GetVertices();
            return {
                Segment2D{-1, {bl.x(), bl.y()}, {tl.x(), tl.y()}},
                Segment2D{-1, {tl.x(), tl.y()}, {tr.x(), tr.y()}},
                Segment2D{-1, {tr.x(), tr.y()}, {br.x(), br.y()}},
                Segment2D{-1, {br.x(), br.y()}, {bl.x(), bl.y()}},
            };
        }
    };

    class Rectangle2D : public Primitive2D {
        Eigen::Vector2d m_center_;
        Eigen::Vector2d m_half_sizes_;
        double m_angle_;
        Eigen::Matrix2d m_rotation_matrix_;

    public:
        Rectangle2D(const int id, Eigen::Vector2d center, Eigen::Vector2d half_sizes, const double angle)
            : m_center_(std::move(center)),
              m_half_sizes_(std::move(half_sizes)),
              m_angle_(angle) {
            this->id = id;
            UpdateMatrices();
        }

        [[nodiscard]] Type
        GetType() const override {
            return Type::kRectangle;
        }

        [[nodiscard]] const Eigen::Vector2d &
        GetCenter() const {
            return m_center_;
        }

        [[nodiscard]] const Eigen::Vector2d &
        GetHalfSizes() const {
            return m_half_sizes_;
        }

        [[nodiscard]] double
        GetOrientationAngle() const override {
            return m_angle_;
        }

        [[nodiscard]] const Eigen::Matrix2d &
        GetRotationMatrix() const {
            return m_rotation_matrix_;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector2d &point) const override {
            Eigen::Vector2d p = m_rotation_matrix_.transpose() * (point - m_center_);
            return std::abs(p.x()) <= m_half_sizes_.x() && std::abs(p.y()) <= m_half_sizes_.y();
        }

        [[nodiscard]] bool
        IsOnBoundary(const Eigen::Vector2d &point) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Line2D &line) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Segment2D &segment) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Ray2D &ray) const override {
            return ComputeIntersections(Line2D{-1, ray.origin, ray.origin + ray.direction});
        }

        /**
         *
         * @return Vertices of the rectangle in clockwise order starting from the bottom-left corner.
         */
        [[nodiscard]] std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d>
        GetVertices() const {
            Eigen::Vector2d min = m_rotation_matrix_ * -m_half_sizes_ + m_center_;
            Eigen::Vector2d max = m_rotation_matrix_ * m_half_sizes_ + m_center_;
            return {
                Eigen::Vector2d(min.x(), min.y()),
                Eigen::Vector2d(min.x(), max.y()),
                Eigen::Vector2d(max.x(), max.y()),
                Eigen::Vector2d(max.x(), min.y()),
            };
        }

        /**
         *
         * @return Edges of the rectangle in clockwise order starting from the left vertical edge.
         */
        [[nodiscard]] std::tuple<Segment2D, Segment2D, Segment2D, Segment2D>
        GetEdges() const {
            const auto [bl, tl, tr, br] = GetVertices();
            return {
                Segment2D{-1, {bl.x(), bl.y()}, {tl.x(), tl.y()}},
                Segment2D{-1, {tl.x(), tl.y()}, {tr.x(), tr.y()}},
                Segment2D{-1, {tr.x(), tr.y()}, {br.x(), br.y()}},
                Segment2D{-1, {br.x(), br.y()}, {bl.x(), bl.y()}},
            };
        }

        Rectangle2D &
        Translate(const Eigen::Vector2d &translation) {
            m_center_ += translation;
            return *this;
        }

        Rectangle2D &
        Rotate(const double angle) {
            m_angle_ += angle;
            UpdateMatrices();
            return *this;
        }

        Rectangle2D &
        SetCenter(const Eigen::Vector2d &center) {
            m_center_ = center;
            return *this;
        }

        Rectangle2D &
        SetOrientationAngle(const double angle) {
            m_angle_ = angle;
            UpdateMatrices();
            return *this;
        }

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputePointsOnBoundary(std::size_t num_points) const;

    private:
        void
        UpdateMatrices() {
            m_rotation_matrix_ << std::cos(m_angle_), -std::sin(m_angle_), std::sin(m_angle_), std::cos(m_angle_);
        }
    };

    class Ellipse2D : public Primitive2D {
        Eigen::Vector2d m_center_;
        Eigen::Vector2d m_radii_;
        double m_angle_;
        Eigen::Matrix2d m_rotation_matrix_;
        Eigen::Matrix2d m_scaled_rotation_matrix_;  // sigma

    public:
        Ellipse2D(const int id, Eigen::Vector2d center, const double a, const double b, const double angle)
            : m_center_(std::move(center)),
              m_radii_(a, b),
              m_angle_(angle) {
            this->id = id;
            UpdateMatrices();
        }

        [[nodiscard]] Type
        GetType() const override {
            return Type::kEllipse;
        }

        [[nodiscard]] const Eigen::Vector2d &
        GetCenter() const {
            return m_center_;
        }

        [[nodiscard]] const Eigen::Vector2d &
        GetRadii() const {
            return m_radii_;
        }

        [[nodiscard]] double
        GetRadiusX() const {
            return m_radii_.x();
        }

        [[nodiscard]] double
        GetRadiusY() const {
            return m_radii_.y();
        }

        [[nodiscard]] double
        GetOrientationAngle() const override {
            return m_angle_;
        }

        [[nodiscard]] const Eigen::Matrix2d &
        GetRotationMatrix() const {
            return m_rotation_matrix_;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector2d &point) const override {
            Eigen::Vector2d p = (m_rotation_matrix_.transpose() * (point - m_center_)).array() / m_radii_.array();
            return p.squaredNorm() <= 1.0;
        }

        [[nodiscard]] bool
        IsOnBoundary(const Eigen::Vector2d &point) const override {
            Eigen::Vector2d p = (m_rotation_matrix_.transpose() * (point - m_center_)).array() / m_radii_.array();
            return p.squaredNorm() == 1.0;
        }

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Line2D &line) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Segment2D &segment) const override;

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputeIntersections(const Ray2D &ray) const override;

        Ellipse2D &
        Translate(const Eigen::Vector2d &translation) {
            m_center_ += translation;
            return *this;
        }

        Ellipse2D &
        Rotate(const double angle) {
            m_angle_ += angle;
            UpdateMatrices();
            return *this;
        }

        Ellipse2D &
        SetCenter(const Eigen::Vector2d &center) {
            m_center_ = center;
            return *this;
        }

        Ellipse2D &
        SetOrientationAngle(const double angle) {
            m_angle_ = angle;
            UpdateMatrices();
            return *this;
        }

        [[nodiscard]] std::vector<Eigen::Vector2d>
        ComputePointsOnBoundary(std::size_t num_points, double start_angle, double end_angle) const;

    private:
        void
        UpdateMatrices() {
            m_rotation_matrix_ << std::cos(m_angle_), -std::sin(m_angle_), std::sin(m_angle_), std::cos(m_angle_);
            const double a = 1.0 / m_radii_.x();
            const double b = 1.0 / m_radii_.y();
            m_scaled_rotation_matrix_ << a * a, 0, 0, b * b;
            m_scaled_rotation_matrix_ = m_rotation_matrix_ * m_scaled_rotation_matrix_ * m_rotation_matrix_.transpose();
        }
    };

}  // namespace erl::geometry
