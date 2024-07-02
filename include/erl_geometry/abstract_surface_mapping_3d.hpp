#pragma once

#include "surface_mapping_octree.hpp"

namespace erl::geometry {

    class AbstractSurfaceMapping3D {

    public:
        virtual ~AbstractSurfaceMapping3D() = default;

        virtual OctreeKeySet
        GetChangedClusters() = 0;

        [[nodiscard]] virtual unsigned int
        GetClusterLevel() const = 0;

        virtual std::shared_ptr<SurfaceMappingOctree>
        GetOctree() = 0;

        [[nodiscard]] virtual double
        GetSensorNoise() const = 0;

        virtual bool
        Update(
            const Eigen::Ref<const Eigen::Matrix3d> &rotation,
            const Eigen::Ref<const Eigen::Vector3d> &translation,
            const Eigen::Ref<const Eigen::MatrixXd> &ranges) = 0;
    };

}  // namespace erl::geometry
