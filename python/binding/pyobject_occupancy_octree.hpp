#pragma once

#include "pyobject_occupancy_octree_node.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_octree_base.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"

namespace erl::geometry {
    class PyObjectOccupancyOctree : public OccupancyOctreeBase<PyObjectOccupancyOctreeNode, OccupancyOctreeBaseSetting> {
    public:
        using Setting = OccupancyOctreeBaseSetting;
        using Drawer = OccupancyOctreeDrawer<PyObjectOccupancyOctree>;

        explicit PyObjectOccupancyOctree(const std::shared_ptr<OccupancyOctreeBaseSetting> &setting)
            : OccupancyOctreeBase(setting) {}

        PyObjectOccupancyOctree()
            : PyObjectOccupancyOctree(std::make_shared<OccupancyOctreeBaseSetting>()) {}

        explicit PyObjectOccupancyOctree(const std::string &filename)
            : PyObjectOccupancyOctree() {  // resolution will be set by LoadData
            ERL_ASSERTM(this->LoadData(filename), "Failed to read PyObjectOccupancyOctree from file: {}", filename);
        }

        PyObjectOccupancyOctree(const PyObjectOccupancyOctree &other) = default;
        PyObjectOccupancyOctree &
        operator=(const PyObjectOccupancyOctree &other) = default;
        PyObjectOccupancyOctree(PyObjectOccupancyOctree &&other) = default;
        PyObjectOccupancyOctree &
        operator=(PyObjectOccupancyOctree &&other) = default;

    protected:
        [[nodiscard]] std::shared_ptr<AbstractOctree>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(setting == nullptr, "setting is not the type for OccupancyOctree.");
                tree_setting = std::make_shared<Setting>();
            }
            return std::make_shared<PyObjectOccupancyOctree>(tree_setting);
        }
    };

    ERL_REGISTER_OCTREE(PyObjectOccupancyOctree);
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::PyObjectOccupancyOctree::Drawer::Setting>
    : public ConvertOccupancyOctreeDrawerSetting<erl::geometry::PyObjectOccupancyOctree::Drawer::Setting> {};  // namespace YAML
