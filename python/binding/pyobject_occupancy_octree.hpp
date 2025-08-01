#pragma once

#include "pyobject_occupancy_octree_node.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_common/serialization.hpp"
#include "erl_geometry/occupancy_octree_base.hpp"
#include "erl_geometry/occupancy_octree_drawer.hpp"

namespace erl::geometry {

    template<typename Dtype>
    class PyObjectOccupancyOctree : public OccupancyOctreeBase<
                                        Dtype,
                                        PyObjectOccupancyOctreeNode,
                                        OccupancyOctreeBaseSetting> {
    public:
        using Setting = OccupancyOctreeBaseSetting;
        using Super = OccupancyOctreeBase<Dtype, PyObjectOccupancyOctreeNode, Setting>;
        using Drawer = OccupancyOctreeDrawer<PyObjectOccupancyOctree>;

        explicit PyObjectOccupancyOctree(const std::shared_ptr<OccupancyOctreeBaseSetting> &setting)
            : Super(setting) {}

        PyObjectOccupancyOctree()
            : PyObjectOccupancyOctree(std::make_shared<OccupancyOctreeBaseSetting>()) {}

        explicit PyObjectOccupancyOctree(const std::string &filename)
            : PyObjectOccupancyOctree() {
            ERL_ASSERTM(
                common::Serialization<PyObjectOccupancyOctree>::Read(filename, this),
                "Failed to read {} from file: {}",
                type_name<PyObjectOccupancyOctree>(),
                filename);
        }

        PyObjectOccupancyOctree(const PyObjectOccupancyOctree &other) = default;
        PyObjectOccupancyOctree &
        operator=(const PyObjectOccupancyOctree &other) = default;
        PyObjectOccupancyOctree(PyObjectOccupancyOctree &&other) = default;
        PyObjectOccupancyOctree &
        operator=(PyObjectOccupancyOctree &&other) = default;

    protected:
        [[nodiscard]] std::shared_ptr<AbstractOctree<Dtype>>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(
                    setting == nullptr,
                    "setting is not the type for {}.",
                    type_name<PyObjectOccupancyOctree>());
                tree_setting = std::make_shared<Setting>();
            }
            return std::make_shared<PyObjectOccupancyOctree>(tree_setting);
        }
    };

    using PyObjectOccupancyOctreeD = PyObjectOccupancyOctree<double>;
    using PyObjectOccupancyOctreeF = PyObjectOccupancyOctree<float>;
}  // namespace erl::geometry
