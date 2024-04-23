#pragma once

#include "occupancy_octree_base.hpp"
#include "occupancy_octree_node.hpp"
#include "occupancy_octree_drawer.hpp"

namespace erl::geometry {

    class OccupancyOctree : public OccupancyOctreeBase<OccupancyOctreeNode, OccupancyOctreeBaseSetting> {
    public:
        using Setting = OccupancyOctreeBaseSetting;
        typedef OccupancyOctreeDrawer<OccupancyOctree> Drawer;

        explicit OccupancyOctree(const std::shared_ptr<OccupancyOctreeBaseSetting> &setting)
            : OccupancyOctreeBase<OccupancyOctreeNode, OccupancyOctreeBaseSetting>(setting) {
            s_init_.EnsureLinking();
        }

        OccupancyOctree(const OccupancyOctree &other) = default;
        OccupancyOctree &
        operator=(const OccupancyOctree &other) = default;
        OccupancyOctree(OccupancyOctree &&other) = default;
        OccupancyOctree &
        operator=(OccupancyOctree &&other) = default;

        OccupancyOctree()
            : OccupancyOctree(std::make_shared<OccupancyOctreeBaseSetting>()) {}

        explicit OccupancyOctree(const std::string &filename)
            : OccupancyOctree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read OccupancyOctree from file: %s", filename.c_str());
        }

        [[nodiscard]] inline std::string
        GetTreeType() const override {
            return "OccupancyOctree";
        }

    protected:
        [[nodiscard]] inline std::shared_ptr<AbstractOctree>
        Create() const override {
            return std::make_shared<OccupancyOctree>();
        }

        /**
         * Static member object which ensures that this OcTree's prototype ends up in the classIDMapping only once.
         * You need this as a static member in any derived octree class in order to read .ot files through the
         * AbstractOcTree factory. You should also call ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer {
        public:
            StaticMemberInitializer() {
                auto tree = std::make_shared<OccupancyOctree>();
                tree->ClearKeyRays();
                AbstractOctree::RegisterTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the StaticMemberInitializer, causing this tree failing
             * to register. Needs to be called from the constructor of this octree.
             */
            void
            EnsureLinking() {}
        };

        inline static StaticMemberInitializer s_init_ = {};
    };
}  // namespace erl::geometry

namespace YAML {

    template<>
    struct convert<erl::geometry::OccupancyOctree::Drawer::Setting>
        : public ConvertOccupancyOctreeDrawerSetting<erl::geometry::OccupancyOctree::Drawer::Setting> {};
}  // namespace YAML
