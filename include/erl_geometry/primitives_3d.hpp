#pragma once

#include "aabb.hpp"

#include <utility>

namespace erl::geometry {

    // struct Line3D;

    // struct Segment3D;

    // struct Ray3D;

    struct Primitive3D {

        enum class Type {
            kLine3D = 0,
            kSegment3D = 1,
            kRay3D = 2,
            kPlane = 3,
            kTriangle = 4,
            kAxisAlignedBox = 5,
            kBox = 6,
            kEllipsoid = 7,
        };

        int id = -1;

        virtual ~Primitive3D() = default;

        [[nodiscard]] virtual Type
        GetType() const = 0;

        [[nodiscard]] virtual bool
        IsInside(const Eigen::Vector3d &point) const = 0;
    };

    class AxisAlignedBox : public Primitive3D, public Aabb3Dd {
    public:
        AxisAlignedBox(const int id, const Eigen::Vector3d &center, const Eigen::Vector3d &half_sizes)
            : Primitive3D(),
              Aabb3Dd(center, half_sizes) {
            this->id = id;
        }

        [[nodiscard]] Type
        GetType() const override {
            return Type::kAxisAlignedBox;
        }

        [[nodiscard]] const Eigen::Vector3d &
        GetCenter() const {
            return center;
        }

        [[nodiscard]] double
        GetWidth() const {
            return half_sizes.x() * 2.0;
        }

        [[nodiscard]] double
        GetHeight() const {
            return half_sizes.y() * 2.0;
        }

        [[nodiscard]] double
        GetDepth() const {
            return half_sizes.z() * 2.0;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector3d &point) const override {
            return contains(point);
        }
    };

    class Box : public Primitive3D {
        Eigen::Vector3d m_center_;
        Eigen::Vector3d m_half_sizes_;
        Eigen::Matrix3d m_rotation_;

    public:
        Box(const int id, Eigen::Vector3d center, Eigen::Vector3d half_sizes, Eigen::Matrix3d rotation)
            : m_center_(std::move(center)),
              m_half_sizes_(std::move(half_sizes)),
              m_rotation_(std::move(rotation)) {
            this->id = id;
        }

        [[nodiscard]] Type
        GetType() const override {
            return Type::kBox;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector3d &point) const override {
            Eigen::Vector3d p = m_rotation_.transpose() * (point - m_center_);
            return std::abs(p.x()) <= m_half_sizes_.x() && std::abs(p.y()) <= m_half_sizes_.y() && std::abs(p.z()) <= m_half_sizes_.z();
        }

        [[nodiscard]] const Eigen::Vector3d &
        GetCenter() const {
            return m_center_;
        }

        [[nodiscard]] const Eigen::Vector3d &
        GetHalfSizes() const {
            return m_half_sizes_;
        }

        [[nodiscard]] const Eigen::Matrix3d &
        GetRotationMatrix() const {
            return m_rotation_;
        }

        Box &
        Translate(const Eigen::Vector3d &translation) {
            m_center_ += translation;
            return *this;
        }

        Box &
        Rotate(const Eigen::Matrix3d &rotation) {
            m_rotation_ = rotation * m_rotation_;
            return *this;
        }

        Box &
        SetCenter(const Eigen::Vector3d &center) {
            m_center_ = center;
            return *this;
        }

        Box &
        SetRotationMatrix(const Eigen::Matrix3d &rotation) {
            m_rotation_ = rotation;
            return *this;
        }
    };

    class Ellipsoid : public Primitive3D {
        Eigen::Vector3d m_center_;
        Eigen::Vector3d m_radii_;
        Eigen::Matrix3d m_rotation_;
        Eigen::Matrix3d m_scaled_rotation_matrix_;

    public:
        Ellipsoid(const int id, Eigen::Vector3d center, Eigen::Vector3d radius, Eigen::Matrix3d rotation)
            : m_center_(std::move(center)),
              m_radii_(std::move(radius)),
              m_rotation_(std::move(rotation)) {
            this->id = id;
            UpdateMatrices();
        }

        [[nodiscard]] Type
        GetType() const override {
            return Type::kEllipsoid;
        }

        [[nodiscard]] bool
        IsInside(const Eigen::Vector3d &point) const override {
            Eigen::Vector3d p = (m_rotation_.transpose() * (point - m_center_)).array() / m_radii_.array();
            return p.squaredNorm() <= 1.0;
        }

        [[nodiscard]] const Eigen::Vector3d &
        GetCenter() const {
            return m_center_;
        }

        [[nodiscard]] const Eigen::Vector3d &
        GetRadii() const {
            return m_radii_;
        }

        [[nodiscard]] const Eigen::Matrix3d &
        GetRotationMatrix() const {
            return m_rotation_;
        }

        Ellipsoid &
        Translate(const Eigen::Vector3d &translation) {
            m_center_ += translation;
            return *this;
        }

        Ellipsoid &
        Rotate(const Eigen::Matrix3d &rotation) {
            m_rotation_ = rotation * m_rotation_;
            UpdateMatrices();
            return *this;
        }

        Ellipsoid &
        SetCenter(const Eigen::Vector3d &center) {
            m_center_ = center;
            return *this;
        }

        Ellipsoid &
        SetRotationMatrix(const Eigen::Matrix3d &rotation) {
            m_rotation_ = rotation;
            UpdateMatrices();
            return *this;
        }

    private:
        void
        UpdateMatrices() {
            const double a = 1.0 / m_radii_[0];
            const double b = 1.0 / m_radii_[1];
            const double c = 1.0 / m_radii_[2];
            m_scaled_rotation_matrix_ << a, 0, 0, 0, b, 0, 0, 0, c;
            m_scaled_rotation_matrix_ = m_rotation_ * m_scaled_rotation_matrix_ * m_rotation_.transpose();
        }
    };

}  // namespace erl::geometry
