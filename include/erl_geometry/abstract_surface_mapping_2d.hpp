#pragma once

#include "surface_mapping_quadtree.hpp"

namespace erl::geometry {

    class AbstractSurfaceMapping2D {

    public:
        virtual ~AbstractSurfaceMapping2D() = default;

        virtual QuadtreeKeySet
        GetChangedClusters() = 0;

        [[nodiscard]] virtual unsigned int
        GetClusterLevel() const = 0;

        virtual std::shared_ptr<SurfaceMappingQuadtree>
        GetQuadtree() = 0;

        [[nodiscard]] virtual double
        GetSensorNoise() const = 0;

        virtual bool
        Update(
            const Eigen::Ref<const Eigen::Matrix2d> &rotation,
            const Eigen::Ref<const Eigen::Vector2d> &translation,
            const Eigen::Ref<const Eigen::MatrixXd> &ranges) = 0;
    };

}  // namespace erl::geometry
