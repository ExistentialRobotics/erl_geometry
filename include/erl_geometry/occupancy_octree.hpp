#pragma once

#include "occupancy_octree_base.hpp"
#include "occupancy_octree_node.hpp"
#include "occupancy_octree_drawer.hpp"

namespace erl::geometry {

    class OccupancyOctree : public OccupancyOctreeBase<OccupancyOctreeNode> {

    public:
        using Setting = OccupancyOctreeBaseSetting;

        explicit OccupancyOctree(double resolution);

        explicit OccupancyOctree(const std::shared_ptr<Setting> &setting);

        explicit OccupancyOctree(const std::string &filename);

        [[nodiscard]] std::shared_ptr<AbstractOctree>
        Create() const override {
            return std::make_shared<OccupancyOctree>(0.1);
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            auto setting = std::make_shared<Setting>();
            setting->log_odd_min = this->GetLogOddMin();
            setting->log_odd_max = this->GetLogOddMax();
            setting->probability_hit = this->GetProbabilityHit();
            setting->probability_miss = this->GetProbabilityMiss();
            setting->probability_occupied_threshold = this->GetOccupancyThreshold();
            setting->resolution = this->GetResolution();
            setting->use_change_detection = this->m_use_change_detection_;
            setting->use_aabb_limit = this->m_use_aabb_limit_;
            setting->aabb = this->m_aabb_;
            return setting;
        }

        [[nodiscard]] std::string
        GetTreeType() const override {
            return "OccupancyOctree";
        }

        typedef OccupancyOctreeDrawer<OccupancyOctree> Drawer;

    protected:
        /**
         * Static member object which ensures that this OcTree's prototype ends up in the classIDMapping only once.
         * You need this as a static member in any derived octree class in order to read .ot files through the
         * AbstractOcTree factory. You should also call ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer {
        public:
            StaticMemberInitializer() {
                auto tree = std::make_shared<OccupancyOctree>(0.1);
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
    struct convert<erl::geometry::OccupancyOctree::Drawer::Setting>
        : public ConvertOccupancyOctreeDrawerSetting<erl::geometry::OccupancyOctree::Drawer::Setting> {};

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::OccupancyOctree::Drawer::Setting &rhs) {
        return PrintOccupancyOctreeDrawerSetting(out, rhs);
    }
}  // namespace YAML
