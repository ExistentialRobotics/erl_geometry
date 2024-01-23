#pragma once

#include "occupancy_octree_base.hpp"
#include "occupancy_octree_node.hpp"
#include "occupancy_octree_drawer.hpp"

namespace erl::geometry {
    
    class OccupancyOctree : public OccupancyOctreeBase<OccupancyOctreeNode> {

    public:
        explicit OccupancyOctree(double resolution)
            : OccupancyOctreeBase<OccupancyOctreeNode>(resolution) {
            s_init_.EnsureLinking();
        }

        explicit OccupancyOctree(const std::shared_ptr<Setting> &setting)
            : OccupancyOctreeBase<OccupancyOctreeNode>(setting) {}

        explicit OccupancyOctree(const std::string &filename)
            : OccupancyOctreeBase<OccupancyOctreeNode>(0.1) {  // resolution will be set by readBinary
            this->ReadBinary(filename);
        }

        [[nodiscard]] std::shared_ptr<AbstractOctree>
        Create() const override {
            return std::make_shared<OccupancyOctree>(0.1);
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
    struct convert<erl::geometry::OccupancyOctree::Setting> : public ConvertOccupancyOctreeBaseSetting<erl::geometry::OccupancyOctree::Setting> {};

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::OccupancyOctree::Setting &rhs) {
        return PrintOccupancyOctreeBaseSetting(out, rhs);
    }

    template<>
    struct convert<erl::geometry::OccupancyOctree::Drawer::Setting>
        : public ConvertOccupancyOctreeDrawerSetting<erl::geometry::OccupancyOctree::Drawer::Setting> {};

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::OccupancyOctree::Drawer::Setting &rhs) {
        return PrintOccupancyOctreeDrawerSetting(out, rhs);
    }
}  // namespace YAML
