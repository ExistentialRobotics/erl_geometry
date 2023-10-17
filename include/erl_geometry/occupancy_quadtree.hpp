#pragma once

#include "occupancy_quadtree_base.hpp"
#include "occupancy_quadtree_node.hpp"
#include "occupancy_quadtree_drawer.hpp"

namespace erl::geometry {

    class OccupancyQuadtree : public OccupancyQuadtreeBase<OccupancyQuadtreeNode> {

    public:
        explicit OccupancyQuadtree(double resolution)
            : OccupancyQuadtreeBase<OccupancyQuadtreeNode>(resolution) {
            s_init_.EnsureLinking();
        }

        explicit OccupancyQuadtree(const std::string &filename)
            : OccupancyQuadtreeBase<OccupancyQuadtreeNode>(0.1) {  // resolution will be set by readBinary
            this->ReadBinary(filename);
        }

        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Create() const override {
            return std::make_shared<OccupancyQuadtree>(0.1);
        }

        [[nodiscard]] std::string
        GetTreeType() const override {
            return ERL_AS_STRING(OccupancyQuadtree);
        }

        typedef OccupancyQuadtreeDrawer<OccupancyQuadtree> Drawer;

    protected:
        /**
         * Static member object which ensures that this OcTree's prototype
         * ends up in the classIDMapping only once. You need this as a
         * static member in any derived octree class in order to read .ot
         * files through the AbstractOcTree factory. You should also call
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
             * Dummy function to ensure that MSVC does not drop the
             * StaticMemberInitializer, causing this tree failing to register.
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
    struct convert<erl::geometry::OccupancyQuadtree::Setting> : public ConvertOccupancyQuadtreeBaseSetting<erl::geometry::OccupancyQuadtree::Setting> {};

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::OccupancyQuadtree::Setting &rhs) {
        return PrintOccupancyQuadtreeBaseSetting(out, rhs);
    }

    template<>
    struct convert<erl::geometry::OccupancyQuadtree::Drawer::Setting>
        : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::OccupancyQuadtree::Drawer::Setting> {};

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::OccupancyQuadtree::Drawer::Setting &rhs) {
        return PrintOccupancyQuadtreeDrawerSetting(out, rhs);
    }
}  // namespace YAML
