#pragma once

#include "kdtree_eigen_adaptor.hpp"
#include "marching_squares.hpp"
#include "surface_2d.hpp"

#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {
    class Space2D {
        using KdTree = nanoflann::KDTreeEigenMatrixAdaptor<
            Eigen::Matrix2Xd,
            2,
            nanoflann::metric_L2_Simple,
            false /*column-major, #rows=dim*/>;

        std::shared_ptr<Surface2D> m_surface_;
        std::unique_ptr<KdTree> m_kdtree_;

    public:
        enum class SignMethod {
            // this works well when points and normals are dense enough
            kPointNormal = 0,
            // this requires lower density than kPointNormal but may give a wrong result if the
            // data is too sparse
            kLineNormal = 1,
            // this works perfectly when m_surface_.outsideFlagsAvailable() returns true
            kPolygon = 2
        };

        static const char *
        GetSignMethodName(const SignMethod &type);

        static SignMethod
        GetSignMethodFromName(const std::string &type_name);

        Space2D(
            const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_vertices,
            const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_normals);

        Space2D(
            const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_vertices,
            const Eigen::Ref<const Eigen::VectorXb> &outside_flags,
            double delta = 0.01,
            bool parallel = false);

        Space2D(
            const Eigen::Ref<const Eigen::MatrixXd> &map_image,
            const common::GridMapInfo2Dd &grid_map_info,
            double free_threshold,
            double delta = 0.01,
            bool parallel = false);

        Space2D(const Space2D &other)
            : m_surface_(std::make_shared<Surface2D>(*other.m_surface_)),
              m_kdtree_(std::make_unique<KdTree>(2, other.m_kdtree_->m_data_matrix)) {}

        [[nodiscard]] const std::shared_ptr<Surface2D> &
        GetSurface() const {
            return m_surface_;
        }

        void
        Translate(const Eigen::Vector2d &translation) {
            m_surface_->Translate(translation);
        }

        [[nodiscard]] Space2D
        AddObstacles(
            const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &obstacle_vertices,
            double delta,
            bool parallel) const;

        /**
         * @brief Generate a map image of which the left-bottom is (xmin, ymin), the right-top is
         * (xmax, ymax), the x-axis is to the right, and the y-axis is to the top.
         * @param grid_map_info
         * @param anti_aliased
         * @return
         */
        [[nodiscard]] Eigen::MatrixX8U
        GenerateMapImage(const common::GridMapInfo2Dd &grid_map_info, bool anti_aliased = false)
            const;

        /**
         * @brief Compute the signed distance field of the space as an image of which the
         * left-bottom is (xmin, ymin), the right-top is (xmax, ymax), the x-axis is to the right,
         * and the y-axis is to the top.
         * @param grid_map_info
         * @param sign_method
         * @param use_kdtree
         * @param parallel
         * @return
         */
        [[nodiscard]] Eigen::MatrixXd
        ComputeSdfImage(
            const common::GridMapInfo2Dd &grid_map_info,
            SignMethod sign_method = SignMethod::kLineNormal,
            bool use_kdtree = false,
            bool parallel = false) const;

        [[nodiscard]] Eigen::VectorXd
        ComputeSdf(
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_points,
            SignMethod sign_method = SignMethod::kLineNormal,
            bool use_kdtree = false,
            bool parallel = false) const;

        [[nodiscard]] double
        ComputeSdfWithKdtree(const Eigen::Vector2d &q, SignMethod sign_method) const;
        [[nodiscard]] double
        ComputeSdfGreedily(const Eigen::Ref<const Eigen::Vector2d> &q, SignMethod sign_method)
            const;

        [[nodiscard]] Eigen::MatrixX<Eigen::VectorXd>
        ComputeDdf(
            const common::GridMapInfo2Dd &grid_map_info,
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
            bool parallel = false) const;

        [[nodiscard]] Eigen::VectorXd
        ComputeDdf(
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_points,
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
            bool parallel = false) const;

        [[nodiscard]] Eigen::MatrixX<Eigen::VectorXd>
        ComputeSddfV1(
            const common::GridMapInfo2Dd &grid_map_info,
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
            bool parallel = false) const;

        [[nodiscard]] Eigen::VectorXd
        ComputeSddfV1(
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_points,
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
            bool parallel = false) const;

        [[nodiscard]] Eigen::MatrixX<Eigen::VectorXd>
        ComputeSddfV2(
            const common::GridMapInfo2Dd &grid_map_info,
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
            SignMethod sign_method = SignMethod::kLineNormal,
            bool parallel = false) const;

        [[nodiscard]] Eigen::VectorXd
        ComputeSddfV2(
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_points,
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
            SignMethod sign_method = SignMethod::kLineNormal,
            bool parallel = false) const;

    private:
        [[nodiscard]] bool
        IsNegativeSdf(
            SignMethod sign_method,
            const Eigen::Ref<const Eigen::Vector2d> &q,
            int vertex_0_idx,
            int vertex_1_idx) const;

        [[nodiscard]] bool
        IsNegativeSdfByPointNormal(const Eigen::Ref<const Eigen::Vector2d> &q, int vertex_0_idx)
            const;

        [[nodiscard]] bool
        IsNegativeSdfBySegmentNormal(
            const Eigen::Ref<const Eigen::Vector2d> &q,
            int vertex_0_idx,
            int vertex_1_idx) const;

        [[nodiscard]] bool
        IsNegativeSdfByPolygon(const Eigen::Ref<const Eigen::Vector2d> &q, int vertex_0_idx) const;

        void
        ComputeNormals(double delta, bool use_kdtree = false, bool parallel = false) const;

        void
        ComputeOutsideFlags(
            const Eigen::Ref<const Eigen::MatrixXd> &map_image,
            double free_threshold) const;  // by winding number

        void
        ComputeOutsideFlags() const;  // by surface normals
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::Space2D::SignMethod> {
    static Node
    encode(const erl::geometry::Space2D::SignMethod &method);

    static bool
    decode(const Node &node, erl::geometry::Space2D::SignMethod &method);
};  // namespace YAML
