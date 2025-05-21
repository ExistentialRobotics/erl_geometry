#pragma once

#include "colored_occupancy_quadtree_node.hpp"
#include "occupancy_quadtree_base.hpp"

namespace erl::geometry {
    template<typename Dtype>
    class ColoredOccupancyQuadtree : public OccupancyQuadtreeBase<
                                         Dtype,
                                         ColoredOccupancyQuadtreeNode,
                                         OccupancyQuadtreeBaseSetting> {
    public:
        using Setting = OccupancyQuadtreeBaseSetting;
        using Super = OccupancyQuadtreeBase<
            Dtype,
            ColoredOccupancyQuadtreeNode,
            OccupancyQuadtreeBaseSetting>;

        explicit ColoredOccupancyQuadtree(
            const std::shared_ptr<OccupancyQuadtreeBaseSetting> &setting)
            : Super(setting) {}

        ColoredOccupancyQuadtree()
            : ColoredOccupancyQuadtree(std::make_shared<Setting>()) {}

        explicit ColoredOccupancyQuadtree(const std::string &filename)
            : ColoredOccupancyQuadtree() {
            ERL_ASSERTM(
                this->LoadData(filename),
                "Failed to read ColoredOccupancyQuadtree from file: {}",
                filename);
        }

        ColoredOccupancyQuadtree(const ColoredOccupancyQuadtree &other) = default;
        ColoredOccupancyQuadtree &
        operator=(const ColoredOccupancyQuadtree &other) = default;
        ColoredOccupancyQuadtree(ColoredOccupancyQuadtree &&other) = default;
        ColoredOccupancyQuadtree &
        operator=(ColoredOccupancyQuadtree &&other) = default;

    protected:
        [[nodiscard]] std::shared_ptr<geometry::AbstractQuadtree<Dtype>>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<OccupancyQuadtreeBaseSetting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(
                    setting == nullptr,
                    "setting is not the type for ColoredOccupancyQuadtree.");
                tree_setting = std::make_shared<OccupancyQuadtreeBaseSetting>();
            }
            return std::make_shared<ColoredOccupancyQuadtree>(tree_setting);
        }
    };

    using ColoredOccupancyQuadtreeD = ColoredOccupancyQuadtree<double>;
    using ColoredOccupancyQuadtreeF = ColoredOccupancyQuadtree<float>;
}  // namespace erl::geometry
