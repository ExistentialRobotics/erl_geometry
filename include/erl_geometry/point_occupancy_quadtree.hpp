#pragma once

#include "erl_common/eigen.hpp"
#include "occupancy_quadtree_base.hpp"
#include "occupancy_quadtree_node.hpp"
#include "occupancy_quadtree_drawer.hpp"

namespace erl::geometry {

    class PointOccupancyQuadtreeNode : public OccupancyQuadtreeNode {
    protected:
        std::vector<Eigen::Vector2d> m_points_;

    public:
        PointOccupancyQuadtreeNode() = default;

        PointOccupancyQuadtreeNode(const PointOccupancyQuadtreeNode &other);

        //-- comparison
        bool
        operator==(const PointOccupancyQuadtreeNode &other) const {
            // If a node is occupied, it must have a point. Then it will never be pruned.
            // Nodes in free space should not have a point so that they can be pruned.
            return m_value_ == other.m_value_ && m_points_ == other.m_points_;
        }

        bool
        operator!=(const PointOccupancyQuadtreeNode &other) const {
            return m_value_ != other.m_value_ || m_points_ != other.m_points_;
        }

        //-- data manipulation
        inline void
        CopyData(const PointOccupancyQuadtreeNode &other) {
            m_value_ = other.m_value_;
            m_points_ = other.m_points_;
        }

        [[nodiscard]] inline const std::vector<Eigen::Vector2d> &
        GetPoints() const {
            return m_points_;
        }

        [[nodiscard]] inline std::size_t
        GetNumPoints() const {
            return m_points_.size();
        }

        void
        AddPoint(Eigen::Vector2d point) {
            m_points_.push_back(std::move(point));
        }

        //-- file IO
        std::istream &
        ReadData(std::istream &s) override;

        std::ostream &
        WriteData(std::ostream &s) const override;
    };

    class PointOccupancyQuadtree : public OccupancyQuadtreeBase<PointOccupancyQuadtreeNode> {
    protected:
        std::size_t m_max_num_points_per_node_ = 1;

    public:
        using Super = OccupancyQuadtreeBase<PointOccupancyQuadtreeNode>;

        struct Setting : public common::OverrideYamlable<OccupancyQuadtreeBaseSetting, Setting> {
            std::size_t max_num_points_per_node = 1;
        };

        explicit PointOccupancyQuadtree(double resolution);

        explicit PointOccupancyQuadtree(const std::shared_ptr<Setting> &setting);

        explicit PointOccupancyQuadtree(const std::string &filename);

        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Create() const override {
            return std::make_shared<PointOccupancyQuadtree>(0.1);
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const;

        [[nodiscard]] std::string
        GetTreeType() const override {
            return "PointOccupancyQuadtree";
        }

        typedef OccupancyQuadtreeDrawer<PointOccupancyQuadtree> Drawer;

        bool
        IsNodeCollapsible(const PointOccupancyQuadtreeNode *node) const override;

        void
        InsertPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd> &points,
            const Eigen::Ref<const Eigen::Vector2d> &sensor_origin,
            double max_range,
            bool parallel,
            bool lazy_eval,
            bool discretize) override;

        void
        InsertPointCloudRays(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            double max_range,
            bool parallel,
            bool lazy_eval) override;

        bool
        InsertRay(double sx, double sy, double ex, double ey, double max_range, bool lazy_eval) override;

    protected:
        /**
         * Static member object which ensures that this OcTree's prototype ends up in the classIDMapping only once.
         * You need this as a static member in any derived octree class in order to read .ot files through the
         * AbstractOcTree factory. You should also call ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer {
        public:
            StaticMemberInitializer() {
                auto tree = std::make_shared<PointOccupancyQuadtree>(0.1);
                tree->ClearKeyRays();
                AbstractQuadtree::RegisterTreeType(tree);
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
    struct convert<erl::geometry::PointOccupancyQuadtree::Setting> {
        inline static Node
        encode(const erl::geometry::PointOccupancyQuadtree::Setting &rhs) {
            Node node = convert<erl::geometry::OccupancyQuadtreeBaseSetting>::encode(rhs);
            node["max_num_points_per_node"] = rhs.max_num_points_per_node;
            return node;
        }

        inline static bool
        decode(const Node &node, erl::geometry::PointOccupancyQuadtree::Setting &rhs) {
            if (!convert<erl::geometry::OccupancyQuadtreeBaseSetting>::decode(node, rhs)) { return false; }
            rhs.max_num_points_per_node = node["max_num_points_per_node"].as<std::size_t>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::PointOccupancyQuadtree::Setting &rhs) {
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
        out << Key << "max_num_points_per_node" << Value << rhs.max_num_points_per_node;
        out << EndMap;
        return out;
    }

    template<>
    struct convert<erl::geometry::PointOccupancyQuadtree::Drawer::Setting>
        : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::PointOccupancyQuadtree::Drawer::Setting> {};

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::PointOccupancyQuadtree::Drawer::Setting &rhs) {
        return PrintOccupancyQuadtreeDrawerSetting(out, rhs);
    }
}  // namespace YAML
