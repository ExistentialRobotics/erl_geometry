#pragma once

#include "abstract_surface_mapping.hpp"
#include "surface_mapping_octree.hpp"

namespace erl::geometry {

    class AbstractSurfaceMapping3D : public AbstractSurfaceMapping {

    public:
        [[nodiscard]] virtual double
        GetSensorNoise() const = 0;

        [[nodiscard]] virtual unsigned int
        GetClusterLevel() const = 0;

        virtual OctreeKeySet
        GetChangedClusters() = 0;

        virtual std::shared_ptr<SurfaceMappingOctree>
        GetOctree() = 0;

        virtual bool
        Update(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            const Eigen::Ref<const Eigen::MatrixXd> &ranges) = 0;

        [[nodiscard]] virtual bool
        operator==(const AbstractSurfaceMapping3D & /*other*/) const {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }

        [[nodiscard]] virtual bool
        operator!=(const AbstractSurfaceMapping3D &other) const {
            return !(*this == other);
        }
    };
}  // namespace erl::geometry
