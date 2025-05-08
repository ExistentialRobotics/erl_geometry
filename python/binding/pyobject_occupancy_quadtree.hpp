#pragma once

#include "pyobject_occupancy_quadtree_node.hpp"

#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"

namespace erl::geometry {
    template<typename Dtype>
    class PyObjectOccupancyQuadtree : public OccupancyQuadtreeBase<
                                          Dtype,
                                          PyObjectOccupancyQuadtreeNode,
                                          OccupancyQuadtreeBaseSetting> {
    public:
        using Setting = OccupancyQuadtreeBaseSetting;
        using Super = OccupancyQuadtreeBase<
            Dtype,
            PyObjectOccupancyQuadtreeNode,
            OccupancyQuadtreeBaseSetting>;
        using Drawer = OccupancyQuadtreeDrawer<PyObjectOccupancyQuadtree>;

        explicit PyObjectOccupancyQuadtree(
            const std::shared_ptr<OccupancyQuadtreeBaseSetting> &setting)
            : Super(setting) {}

        PyObjectOccupancyQuadtree()
            : PyObjectOccupancyQuadtree(std::make_shared<OccupancyQuadtreeBaseSetting>()) {}

        explicit PyObjectOccupancyQuadtree(const std::string &filename)
            : PyObjectOccupancyQuadtree() {
            ERL_ASSERTM(
                common::Serialization<PyObjectOccupancyQuadtree>::Read(filename, *this),
                "Failed to read {} from file: {}",
                type_name<PyObjectOccupancyQuadtree>(),
                filename);
        }

        PyObjectOccupancyQuadtree(const PyObjectOccupancyQuadtree &other) = default;
        PyObjectOccupancyQuadtree &
        operator=(const PyObjectOccupancyQuadtree &other) = default;
        PyObjectOccupancyQuadtree(PyObjectOccupancyQuadtree &&other) = default;
        PyObjectOccupancyQuadtree &
        operator=(PyObjectOccupancyQuadtree &&other) = default;

    protected:
        [[nodiscard]] std::shared_ptr<AbstractQuadtree<Dtype>>
        Create(const std::shared_ptr<NdTreeSetting> &setting) const override {
            auto tree_setting = std::dynamic_pointer_cast<Setting>(setting);
            if (tree_setting == nullptr) {
                ERL_DEBUG_ASSERT(
                    setting == nullptr,
                    "setting is not the type for {}.",
                    type_name<PyObjectOccupancyQuadtree>());
                tree_setting = std::make_shared<Setting>();
            }
            return std::make_shared<PyObjectOccupancyQuadtree>(tree_setting);
        }
    };

    using PyObjectOccupancyQuadtreeD = PyObjectOccupancyQuadtree<double>;
    using PyObjectOccupancyQuadtreeF = PyObjectOccupancyQuadtree<float>;
}  // namespace erl::geometry
