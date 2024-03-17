#pragma once

#include "occupancy_quadtree_base.hpp"
#include "occupancy_quadtree_node.hpp"
#include "occupancy_quadtree_drawer.hpp"

namespace erl::geometry {

    class OccupancyQuadtree : public OccupancyQuadtreeBase<OccupancyQuadtreeNode> {

    public:
        using Setting = OccupancyQuadtreeBaseSetting;

        explicit OccupancyQuadtree(double resolution);

        explicit OccupancyQuadtree(const std::shared_ptr<Setting> &setting);

        explicit OccupancyQuadtree(const std::string &filename);

        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Create() const override {
            return std::make_shared<OccupancyQuadtree>(0.1);
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
            return "OccupancyQuadtree";
        }

        typedef OccupancyQuadtreeDrawer<OccupancyQuadtree> Drawer;

    protected:
        /**
         * Static member object which ensures that this OcTree's prototype ends up in the classIDMapping only once. You need this as a
         * static member in any derived octree class in order to read .ot files through the AbstractOcTree factory. You should also call
         * ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer {
        public:
            StaticMemberInitializer() {
                auto tree = std::make_shared<OccupancyQuadtree>(0.1);
                tree->ClearKeyRays();
                AbstractQuadtree::RegisterTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the StaticMemberInitializer, causing this tree failing to register.
             * Needs to be called from the constructor of this octree.
             */
            void
            EnsureLinking(){};
        };

        inline static StaticMemberInitializer s_init_ = {};
    };
}  // namespace erl::geometry

namespace YAML {

    template<>
    struct convert<erl::geometry::OccupancyQuadtree::Drawer::Setting>
        : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::OccupancyQuadtree::Drawer::Setting> {};

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::OccupancyQuadtree::Drawer::Setting &rhs) {
        return PrintOccupancyQuadtreeDrawerSetting(out, rhs);
    }
}  // namespace YAML
