#pragma once

#include "pyobject_occupancy_quadtree_node.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

namespace erl::geometry {
    class PyObjectOccupancyQuadtree : public OccupancyQuadtreeBase<PyObjectOccupancyQuadtreeNode, OccupancyQuadtreeBaseSetting> {
    public:
        using Setting = OccupancyQuadtreeBaseSetting;
        using Drawer = OccupancyQuadtreeDrawer<PyObjectOccupancyQuadtree>;

        explicit PyObjectOccupancyQuadtree(const std::shared_ptr<OccupancyQuadtreeBaseSetting> &setting)
            : OccupancyQuadtreeBase(setting) {}

        PyObjectOccupancyQuadtree()
            : PyObjectOccupancyQuadtree(std::make_shared<OccupancyQuadtreeBaseSetting>()) {}

        explicit PyObjectOccupancyQuadtree(const std::string &filename)
            : PyObjectOccupancyQuadtree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read PyObjectOccupancyQuadtree from file: {}", filename);
        }

        PyObjectOccupancyQuadtree(const PyObjectOccupancyQuadtree &other) = default;
        PyObjectOccupancyQuadtree &
        operator=(const PyObjectOccupancyQuadtree &other) = default;
        PyObjectOccupancyQuadtree(PyObjectOccupancyQuadtree &&other) = default;
        PyObjectOccupancyQuadtree &
        operator=(PyObjectOccupancyQuadtree &&other) = default;

    protected:
        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(setting == nullptr, "setting is not the type for OccupancyQuadtree.");
                tree_setting = std::make_shared<Setting>();
            }
            return std::make_shared<PyObjectOccupancyQuadtree>(tree_setting);
        }
    };

    ERL_REGISTER_QUADTREE(PyObjectOccupancyQuadtree);
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::PyObjectOccupancyQuadtree::Drawer::Setting>
    : public ConvertOccupancyQuadtreeDrawerSetting<erl::geometry::PyObjectOccupancyQuadtree::Drawer::Setting> {};  // namespace YAML
