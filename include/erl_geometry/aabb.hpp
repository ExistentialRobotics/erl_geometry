#pragma once

#include <utility>
#include <vector>

#include "erl_common/eigen.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    template<typename ScalarType, int Dim>
    struct AABB : public Eigen::AlignedBox<ScalarType, Dim> {

        typedef ScalarType Scalar;
        typedef Eigen::Vector<Scalar, Dim> Point;

        Point center = {};
        Point half_sizes = {};

        AABB() = default;

        AABB(Point center, Scalar half_size)
            : Eigen::AlignedBox<Scalar, Dim>(center.array() - half_size, center.array() + half_size),
              center(std::move(center)),
              half_sizes(Point::Constant(half_size)) {}

        AABB(const Point &min, const Point &max)
            : Eigen::AlignedBox<Scalar, Dim>(min, max),
              center((min + max) / 2),
              half_sizes((max - min) / 2) {}
    };

    typedef AABB<double, 2> Aabb2D;
    typedef AABB<double, 3> Aabb3D;
}  // namespace erl::geometry

namespace YAML {
    template<typename AABB>
    struct ConvertAABB {
        inline static Node
        encode(const AABB &rhs) {
            Node node;
            node["center"] = rhs.center;
            node["half_sizes"] = rhs.half_sizes;
            return node;
        }

        inline static bool
        decode(const Node &node, AABB &rhs) {
            if (!node.IsMap() || node.size() != 2) { return false; }
            rhs.center = node["center"].as<typename AABB::Point>();
            rhs.half_sizes = node["half_sizes"].as<typename AABB::Point>();
            return true;
        }
    };

    template<>
    struct convert<erl::geometry::Aabb2D> : public ConvertAABB<erl::geometry::Aabb2D> {};

    template<>
    struct convert<erl::geometry::Aabb3D> : public ConvertAABB<erl::geometry::Aabb3D> {};

    template<typename AABB>
    Emitter &
    PrintAABB(Emitter &out, const AABB &rhs) {
        out << BeginMap;
        out << Key << "center" << Value << rhs.center;
        out << Key << "half_sizes" << Value << rhs.half_sizes;
        out << EndMap;
        return out;
    }

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::Aabb2D &rhs) {
        return PrintAABB(out, rhs);
    }

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::Aabb3D &rhs) {
        return PrintAABB(out, rhs);
    }
}  // namespace YAML
