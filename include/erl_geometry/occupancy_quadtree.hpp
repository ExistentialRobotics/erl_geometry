#pragma once

#include "occupancy_quadtree_base.hpp"
#include "occupancy_quadtree_node.hpp"
#include "occupancy_quadtree_drawer.hpp"

namespace erl::geometry {

    class OccupancyQuadtree : public OccupancyQuadtreeBase<OccupancyQuadtreeNode> {
    public:
        using Super = OccupancyQuadtreeBase<OccupancyQuadtreeNode>;
        using Setting = OccupancyQuadtreeBaseSetting;
        typedef OccupancyQuadtreeDrawer<OccupancyQuadtree> Drawer;

        explicit OccupancyQuadtree(const std::shared_ptr<Setting> &setting)
            : OccupancyQuadtreeBase<OccupancyQuadtreeNode>(setting) {
            s_init_.EnsureLinking();
        }

        OccupancyQuadtree()
            : OccupancyQuadtree(std::make_shared<Setting>()) {}

        explicit OccupancyQuadtree(const std::string &filename)
            : OccupancyQuadtree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read %s from file: %s", GetTreeType().c_str(), filename.c_str());
        }

        OccupancyQuadtree(const OccupancyQuadtree &) = delete;  // no copy constructor

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return Super::m_setting_;
        }

        [[nodiscard]] inline std::string
        GetTreeType() const override {
            return "OccupancyQuadtree";
        }

    protected:
        [[nodiscard]] inline std::shared_ptr<AbstractQuadtree>
        Create() const override {
            return std::make_shared<OccupancyQuadtree>();
        }

        /**
         * Static member object which ensures that this OcTree's prototype ends up in the classIDMapping only once. You need this as a
         * static member in any derived octree class in order to read .ot files through the AbstractOcTree factory. You should also call
         * ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer {
        public:
            StaticMemberInitializer() {
                auto tree = std::make_shared<OccupancyQuadtree>(std::make_shared<Setting>());
                tree->ClearKeyRays();
                AbstractQuadtree::RegisterTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the StaticMemberInitializer, causing this tree failing to register.
             * Needs to be called from the constructor of this quadtree.
             */
            void
            EnsureLinking(){}
        };

        inline static StaticMemberInitializer s_init_ = {};
    };
}  // namespace erl::geometry

namespace YAML {

    template<>
    struct convert<erl::geometry::OccupancyQuadtree::Drawer::Setting>
        : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::OccupancyQuadtree::Drawer::Setting> {};
}  // namespace YAML
