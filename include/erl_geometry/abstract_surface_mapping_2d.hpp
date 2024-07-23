#pragma once

#include "abstract_surface_mapping.hpp"
#include "surface_mapping_quadtree.hpp"

#include "erl_common/exception.hpp"

namespace erl::geometry {

    class AbstractSurfaceMapping2D : public AbstractSurfaceMapping {

    public:
        [[nodiscard]] virtual double
        GetSensorNoise() const = 0;

        [[nodiscard]] virtual unsigned int
        GetClusterLevel() const = 0;

        virtual QuadtreeKeySet
        GetChangedClusters() = 0;

        virtual std::shared_ptr<SurfaceMappingQuadtree>
        GetQuadtree() = 0;

        virtual bool
        Update(
            const Eigen::Ref<const Eigen::Matrix2d> &rotation,
            const Eigen::Ref<const Eigen::Vector2d> &translation,
            const Eigen::Ref<const Eigen::MatrixXd> &ranges) = 0;

        [[nodiscard]] virtual bool
        operator==(const AbstractSurfaceMapping2D & /*other*/) const {
            throw NotImplemented(__PRETTY_FUNCTION__);
        }

        [[nodiscard]] virtual bool
        operator!=(const AbstractSurfaceMapping2D &other) const {
            return !(*this == other);
        }
    };
}  // namespace erl::geometry
