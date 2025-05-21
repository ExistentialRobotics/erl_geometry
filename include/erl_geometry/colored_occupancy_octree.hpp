#pragma once

#include "colored_occupancy_octree_node.hpp"
#include "occupancy_octree_base.hpp"

namespace erl::geometry {
    template<typename Dtype>
    class ColoredOccupancyOctree : public OccupancyOctreeBase<
                                       Dtype,
                                       ColoredOccupancyOctreeNode,
                                       OccupancyOctreeBaseSetting> {
    public:
        using Setting = OccupancyOctreeBaseSetting;
        using Super =
            OccupancyOctreeBase<Dtype, ColoredOccupancyOctreeNode, OccupancyOctreeBaseSetting>;

        explicit ColoredOccupancyOctree(const std::shared_ptr<OccupancyOctreeBaseSetting> &setting)
            : Super(setting) {}

        ColoredOccupancyOctree()
            : ColoredOccupancyOctree(std::make_shared<Setting>()) {}

        explicit ColoredOccupancyOctree(const std::string &filename)
            : ColoredOccupancyOctree() {
            ERL_ASSERTM(
                this->LoadData(filename),
                "Failed to read ColoredOccupancyOctree from file: {}",
                filename);
        }

        ColoredOccupancyOctree(const ColoredOccupancyOctree &other) = default;
        ColoredOccupancyOctree &
        operator=(const ColoredOccupancyOctree &other) = default;
        ColoredOccupancyOctree(ColoredOccupancyOctree &&other) = default;
        ColoredOccupancyOctree &
        operator=(ColoredOccupancyOctree &&other) = default;

    protected:
        [[nodiscard]] std::shared_ptr<geometry::AbstractOctree<Dtype>>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<OccupancyOctreeBaseSetting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(
                    setting == nullptr,
                    "setting is not the type for ColoredOccupancyOctree.");
                tree_setting = std::make_shared<OccupancyOctreeBaseSetting>();
            }
            return std::make_shared<ColoredOccupancyOctree>(tree_setting);
        }
    };

    using ColoredOccupancyOctreeD = ColoredOccupancyOctree<double>;
    using ColoredOccupancyOctreeF = ColoredOccupancyOctree<float>;
}  // namespace erl::geometry
