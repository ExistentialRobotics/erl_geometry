#pragma once

#include "occupancy_octree_base.hpp"
#include "occupancy_octree_node.hpp"
#include "occupancy_octree_drawer.hpp"

namespace erl::geometry {

    class OccupancyOctree : public OccupancyOctreeBase<OccupancyOctreeNode> {
    public:
        using Super = OccupancyOctreeBase<OccupancyOctreeNode>;
        using Setting = Super::Setting;
        typedef OccupancyOctreeDrawer<OccupancyOctree> Drawer;

        explicit OccupancyOctree(const std::shared_ptr<Setting> &setting)
            : OccupancyOctreeBase<OccupancyOctreeNode>(setting) {
            s_init_.EnsureLinking();
        }

        OccupancyOctree()
            : OccupancyOctree(std::make_shared<Setting>()) {}

        explicit OccupancyOctree(const std::string &filename)
            : OccupancyOctree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read %s from file: %s", GetTreeType().c_str(), filename.c_str());
        }

        OccupancyOctree(const OccupancyOctree &) = delete;  // no copy constructor

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return Super::m_setting_;
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
            EnsureLinking(){}
        };

        inline static StaticMemberInitializer s_init_ = {};
    };
}  // namespace erl::geometry

namespace YAML {

    template<>
    struct convert<erl::geometry::OccupancyOctree::Drawer::Setting>
        : public ConvertOccupancyOctreeDrawerSetting<erl::geometry::OccupancyOctree::Drawer::Setting> {};
}  // namespace YAML
