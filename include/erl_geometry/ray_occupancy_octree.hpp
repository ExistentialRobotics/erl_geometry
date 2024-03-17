#pragma once

#include "erl_common/eigen.hpp"
#include "occupancy_octree_base.hpp"
#include "occupancy_octree_node.hpp"
#include "occupancy_octree_drawer.hpp"

namespace erl::geometry {

    class RayOccupancyOctreeNode : public OccupancyOctreeNode {
    protected:
        std::vector<Eigen::Vector3d> m_start_points_;
        std::vector<Eigen::Vector3d> m_end_points_;
        std::vector<Eigen::Vector3d> m_directions_inv_;

    public:
        RayOccupancyOctreeNode() = default;

        RayOccupancyOctreeNode(const RayOccupancyOctreeNode &other);

        //-- comparison
        bool
        operator==(const RayOccupancyOctreeNode &other) const {
            // If a node is occupied, it must have at least one ray. Then it will never be pruned.
            // Nodes in free space should not have a ray so that they can be pruned.
            return m_value_ == other.m_value_ && m_start_points_ == other.m_start_points_ && m_end_points_ == other.m_end_points_ &&
                   m_directions_inv_ == other.m_directions_inv_;
        }

        bool
        operator!=(const RayOccupancyOctreeNode &other) const {
            return m_value_ != other.m_value_ || m_start_points_ != other.m_start_points_ || m_end_points_ != other.m_end_points_ ||
                   m_directions_inv_ != other.m_directions_inv_;
        }

        //-- data manipulation
        inline void
        CopyData(const RayOccupancyOctreeNode &other) {
            m_value_ = other.m_value_;
            m_start_points_ = other.m_start_points_;
            m_end_points_ = other.m_end_points_;
            m_directions_inv_ = other.m_directions_inv_;
        }

        [[nodiscard]] inline const std::vector<Eigen::Vector3d> &
        GetStartPoints() const {
            return m_start_points_;
        }

        [[nodiscard]] inline const std::vector<Eigen::Vector3d> &
        GetEndPoints() const {
            return m_end_points_;
        }

        [[nodiscard]] inline const std::vector<Eigen::Vector3d> &
        GetDirectionsInv() const {
            return m_directions_inv_;
        }

        [[nodiscard]] inline std::size_t
        GetNumRays() const {
            return m_start_points_.size();
        }

        void
        AddRay(Eigen::Vector3d start_point, Eigen::Vector3d end_point) {
            Eigen::Vector3d direction_inv = end_point - start_point;
            double norm = direction_inv.norm();
            direction_inv = norm / direction_inv.array();

            m_start_points_.push_back(std::move(start_point));
            m_end_points_.push_back(std::move(end_point));
            m_directions_inv_.push_back(std::move(direction_inv));
        }

        //-- file IO
        std::istream &
        ReadData(std::istream &s) override;

        std::ostream &
        WriteData(std::ostream &s) const override;
    };

    class RayOccupancyOctree : public OccupancyOctreeBase<RayOccupancyOctreeNode> {
    protected:
        std::size_t m_max_num_rays_per_node_ = 1000;

    public:
        using Super = OccupancyOctreeBase<RayOccupancyOctreeNode>;
        typedef OccupancyOctreeDrawer<RayOccupancyOctree> Drawer;

        struct Setting : public common::OverrideYamlable<OccupancyOctreeBaseSetting, Setting> {
            std::size_t max_num_rays_per_node = 1000;
        };

        explicit RayOccupancyOctree(double resolution);

        explicit RayOccupancyOctree(const std::shared_ptr<Setting> &setting);

        explicit RayOccupancyOctree(const std::string &filename);

        [[nodiscard]] std::shared_ptr<AbstractOctree>
        Create() const override {
            return std::make_shared<RayOccupancyOctree>(0.1);
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const;

        [[nodiscard]] std::string
        GetTreeType() const override {
            return "RayOccupancyOctree";
        }

        bool
        IsNodeCollapsible(const RayOccupancyOctreeNode *node) const override;

        void
        InsertPointCloud(
            const Eigen::Ref<const Eigen::Matrix3Xd> &points,
            const Eigen::Ref<const Eigen::Vector3d> &sensor_origin,
            double max_range,
            bool parallel,
            bool lazy_eval,
            bool discretize) override;

        void
        InsertPointCloudRays(
            const Eigen::Ref<const Eigen::Matrix3Xd> &points,
            const Eigen::Ref<const Eigen::Vector3d> &sensor_origin,
            double max_range,
            bool parallel,
            bool lazy_eval) override;

        bool
        InsertRay(double sx, double sy, double sz, double ex, double ey, double ez, double max_range, bool lazy_eval) override;

    protected:
        /**
         * Static member object which ensures that this OcTree's prototype ends up in the classIDMapping only once.
         * You need this as a static member in any derived octree class in order to read .ot files through the
         * AbstractOcTree factory. You should also call ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer {
        public:
            StaticMemberInitializer() {
                auto tree = std::make_shared<RayOccupancyOctree>(0.1);
                tree->ClearKeyRays();
                AbstractOctree::RegisterTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the StaticMemberInitializer, causing this tree failing
             * to register. Needs to be called from the constructor of this octree.
             */
            void
            EnsureLinking(){};
        };

        inline static StaticMemberInitializer s_init_ = {};
    };
}  // namespace erl::geometry

namespace YAML {
    template<>
    struct convert<erl::geometry::RayOccupancyOctree::Setting> {
        inline static Node
        encode(const erl::geometry::RayOccupancyOctree::Setting &rhs) {
            Node node = convert<erl::geometry::OccupancyOctreeBaseSetting>::encode(rhs);
            node["max_num_rays_per_node"] = rhs.max_num_rays_per_node;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::geometry::RayOccupancyOctree::Setting &rhs) {
            if (!convert<erl::geometry::OccupancyOctreeBaseSetting>::decode(node, rhs)) { return false; }
            rhs.max_num_rays_per_node = node["max_num_rays_per_node"].as<std::size_t>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::RayOccupancyOctree::Setting &rhs) {
        out << BeginMap;
        out << Key << "log_odd_min" << Value << rhs.log_odd_min;
        out << Key << "log_odd_max" << Value << rhs.log_odd_max;
        out << Key << "probability_hit" << Value << rhs.probability_hit;
        out << Key << "probability_miss" << Value << rhs.probability_miss;
        out << Key << "probability_occupied_threshold" << Value << rhs.probability_occupied_threshold;
        out << Key << "resolution" << Value << rhs.resolution;
        out << Key << "use_change_detection" << Value << rhs.use_change_detection;
        out << Key << "use_aabb_limit" << Value << rhs.use_aabb_limit;
        out << Key << "aabb" << Value << rhs.aabb;
        out << Key << "max_num_rays_per_node" << Value << rhs.max_num_rays_per_node;
        out << EndMap;
        return out;
    }

    template<>
    struct convert<erl::geometry::RayOccupancyOctree::Drawer::Setting>
        : public ConvertOccupancyOctreeDrawerSetting<erl::geometry::RayOccupancyOctree::Drawer::Setting> {};

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::RayOccupancyOctree::Drawer::Setting &rhs) {
        return PrintOccupancyOctreeDrawerSetting(out, rhs);
    }
}  // namespace YAML
