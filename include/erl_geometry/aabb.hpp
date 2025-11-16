#pragma once

#include "erl_common/yaml.hpp"

#include <utility>

namespace erl::geometry {

    template<typename Dtype, int Dim>
    struct Aabb : Eigen::AlignedBox<Dtype, Dim> {

        using Scalar = Dtype;
        using Point = Eigen::Vector<Scalar, Dim>;

        Point center = {};
        Point half_sizes = {};

        Aabb() = default;

        Aabb(Point center, Scalar half_size)
            : Eigen::AlignedBox<Scalar, Dim>(
                  center.array() - half_size,
                  center.array() + half_size),
              center(std::move(center)),
              half_sizes(Point::Constant(half_size)) {}

        Aabb(const Point &min, const Point &max)
            : Eigen::AlignedBox<Scalar, Dim>(min, max),
              center((min + max) / 2),
              half_sizes((max - min) / 2) {}

        [[nodiscard]] Aabb
        Padding(const Point &padding) const {
            return {this->m_min - padding, this->m_max + padding};
        }

        [[nodiscard]] Aabb
        Padding(Scalar padding) const {
            return {this->m_min.array() - padding, this->m_max.array() + padding};
        }

        bool
        operator==(const Aabb &rhs) const {
            return center == rhs.center && half_sizes == rhs.half_sizes;
        }

        bool
        operator!=(const Aabb &rhs) const {
            return !(*this == rhs);
        }

        [[nodiscard]] bool
        IsValid() const {
            for (int i = 0; i < Dim; ++i) {
                if (half_sizes[i] < 0) { return false; }
            }
            return true;
        }

        [[nodiscard]] Aabb
        Intersection(const Aabb &rhs) const {
            return {this->m_min.cwiseMax(rhs.m_min), this->m_max.cwiseMin(rhs.m_max)};
        }

        template<typename Dtype2>
        Aabb<Dtype2, Dim>
        Cast() const {
            return {this->m_min.template cast<Dtype2>(), this->m_max.template cast<Dtype2>()};
        }
    };

    using Aabb2Dd = Aabb<double, 2>;
    using Aabb3Dd = Aabb<double, 3>;
    using Aabb2Df = Aabb<float, 2>;
    using Aabb3Df = Aabb<float, 3>;
}  // namespace erl::geometry

template<typename Dtype, int Dim>
struct YAML::convert<erl::geometry::Aabb<Dtype, Dim>> {
    static Node
    encode(const erl::geometry::Aabb<Dtype, Dim> &aabb) {
        Node node;
        node["center"] = aabb.center;
        node["half_sizes"] = aabb.half_sizes;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::Aabb<Dtype, Dim> &aabb) {
        if (!node.IsMap()) { return false; }
        using Point = typename erl::geometry::Aabb<Dtype, Dim>::Point;
        aabb.center = node["center"].as<Point>();
        aabb.half_sizes = node["half_sizes"].as<Point>();
        aabb = erl::geometry::Aabb<Dtype, Dim>(aabb.center, aabb.half_sizes);
        return true;
    }
};

template<typename Dtype, int Dim>
struct erl::common::program_options::ParseOption<erl::geometry::Aabb<Dtype, Dim>>
    : erl::common::program_options::ParseOptionBase {
    void
    Run(ProgramOptionsData &po_data,
        const std::string &option_name,
        erl::geometry::Aabb<Dtype, Dim> &member) {

        std::vector<Dtype> center;
        std::vector<Dtype> half_sizes;

        po_data.desc.add_options()(
            GetBoostOptionName(option_name, "center").c_str(),
            po::value<std::vector<Dtype>>()->multitoken()->notifier(
                [&center](const std::vector<Dtype> &vals) { center = vals; }),
            "Center of the AABB");

        po_data.desc.add_options()(
            GetBoostOptionName(option_name, "half_sizes").c_str(),
            po::value<std::vector<Dtype>>()->multitoken()->notifier(
                [&half_sizes](const std::vector<Dtype> &vals) { half_sizes = vals; }),
            "Half sizes of the AABB");

        po_data.Parse();

        if (center.empty() && half_sizes.empty()) { return; }

        ERL_ASSERTM(
            center.size() == static_cast<std::size_t>(Dim),
            "Expecting {} values for {}.center, got {}",
            Dim,
            option_name,
            center.size());
        ERL_ASSERTM(
            half_sizes.size() == static_cast<std::size_t>(Dim),
            "Expecting {} values for {}.half_sizes, got {}",
            Dim,
            option_name,
            half_sizes.size());

        for (int i = 0; i < Dim; ++i) {
            ERL_ASSERTM(
                half_sizes[i] > 0,
                "Half size must be non-negative for {}.half_sizes[{}], got {}",
                option_name,
                i,
                half_sizes[i]);
            member.center[i] = center[i];
            member.half_sizes[i] = half_sizes[i];
        }

        member = erl::geometry::Aabb<Dtype, Dim>(member.center, member.half_sizes);
    }
};

#ifdef ERL_ROS_VERSION_1
template<typename Dtype, int Dim>
struct erl::common::ros_params::LoadRos1Param<erl::geometry::Aabb<Dtype, Dim>> {
    using Aabb = erl::geometry::Aabb<Dtype, Dim>;

    static void
    Run(ros::NodeHandle &nh, const std::string &param_name, Aabb &member) {
        using Point = typename Aabb::Point;
        LoadRos1Param<Point>::Run(nh, GetRos1ParamPath(param_name, "center"), member.center);
        LoadRos1Param<Point>::Run(
            nh,
            GetRos1ParamPath(param_name, "half_sizes"),
            member.half_sizes);
        member = Aabb(member.center, member.half_sizes);
    }
};
#endif

#ifdef ERL_ROS_VERSION_2
template<typename Dtype, int Dim>
struct erl::common::ros_params::LoadRos2Param<erl::geometry::Aabb<Dtype, Dim>> {
    using Aabb = erl::geometry::Aabb<Dtype, Dim>;

    static void
    Run(rclcpp::Node *node, const std::string &param_name, Aabb &member) {
        using Point = typename Aabb::Point;
        LoadRos2Param<Point>::Run(node, GetRos2ParamPath(param_name, "center"), member.center);
        LoadRos2Param<Point>::Run(
            node,
            GetRos2ParamPath(param_name, "half_sizes"),
            member.half_sizes);
        member = Aabb(member.center, member.half_sizes);
    }
};
#endif
