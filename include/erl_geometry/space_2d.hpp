#pragma once

#include <optional>

#include "erl_common/grid_map_info.hpp"
#include "erl_common/yaml.hpp"
#include "kdtree_eigen_adaptor.hpp"
#include "marching_square.hpp"
#include "surface_2d.hpp"
#include "winding_number.hpp"

namespace erl::geometry {
    class Space2D {
        using KdTree = KdTreeEigenAdaptor<double, 2>;

        std::shared_ptr<Surface2D> m_surface_;
        std::unique_ptr<KdTree> m_kdtree_;

    public:
        enum class SignMethod {
            kPointNormal = 0,  // this works well when points and normals are dense enough
            kLineNormal = 1,   // this requires lower density than kPointNormal, but may gives wrong result if the data is too sparse
            kPolygon = 2       // this works perfectly when m_surface_.outsideFlagsAvailable() returns true
        };

        static inline const char *
        GetSignMethodName(const SignMethod &type) {
            static const char *names[3] = {"kPointNormal", "kLineNormal", "kPolygon"};
            return names[static_cast<int>(type)];
        }

        static inline SignMethod
        GetSignMethodFromName(const std::string &type_name) {
            if (type_name == ERL_AS_STRING(kPointNormal)) { return SignMethod::kPointNormal; }
            if (type_name == ERL_AS_STRING(kLineNormal)) { return SignMethod::kLineNormal; }
            if (type_name == ERL_AS_STRING(kPolygon)) { return SignMethod::kPolygon; }

            throw std::runtime_error("Unknown sign method: " + type_name);
        }

        Space2D(
            const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_vertices,
            const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_normals
        );

        Space2D(
            const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &ordered_object_vertices,
            const Eigen::Ref<const Eigen::VectorXb> &outside_flags,
            double delta = 0.01,
            bool parallel = false
        );

        Space2D(
            const Eigen::Ref<const Eigen::MatrixXd> &map_image,
            const common::GridMapInfo2D &grid_map_info,
            double free_threshold,
            double delta = 0.01,
            bool parallel = false
        );

        Space2D(const Space2D &other)
            : m_surface_(std::make_shared<Surface2D>(*other.m_surface_)),
              m_kdtree_(std::make_unique<KdTree>(*other.m_kdtree_)) {}

        [[nodiscard]] inline const std::shared_ptr<Surface2D> &
        GetSurface() const {
            return m_surface_;
        }

        Space2D
        AddObstacles(const std::vector<Eigen::Ref<const Eigen::Matrix2Xd>> &obstacle_vertices, double delta, bool parallel);

        /**
         * @brief Generate a map image of which the left-bottom is (xmin, ymin), the right-top is (xmax, ymax), the x axis is to the right, and the y axis is
         * to the top.
         * @param grid_map_info
         * @param anti_aliased
         * @return
         */
        Eigen::MatrixX8U
        GenerateMapImage(const common::GridMapInfo2D &grid_map_info, bool anti_aliased = false);

        /**
         * @brief Compute the signed distance field of the space as an image of which the left-bottom is (xmin, ymin), the right-top is (xmax, ymax), the
         * x axis is to the right, and the y axis is to the top.
         * @param grid_map_info
         * @param sign_method
         * @param use_kdtree
         * @param parallel
         * @return
         */
        [[nodiscard]] Eigen::MatrixXd
        ComputeSdfImage(
            const common::GridMapInfo2D &grid_map_info, SignMethod sign_method = SignMethod::kLineNormal, bool use_kdtree = false, bool parallel = false
        ) const;

        [[nodiscard]] Eigen::VectorXd
        ComputeSdf(
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_points,
            SignMethod sign_method = SignMethod::kLineNormal,
            bool use_kdtree = false,
            bool parallel = false
        ) const;

        [[nodiscard]] double
        ComputeSdfWithKdtree(const Eigen::Vector2d &q, SignMethod sign_method) const;
        [[nodiscard]] double
        ComputeSdfGreedily(const Eigen::Ref<const Eigen::Vector2d> &q, SignMethod sign_method) const;

        [[nodiscard]] Eigen::MatrixX<Eigen::VectorXd>
        ComputeDdf(const common::GridMapInfo2D &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel = false) const;

        [[nodiscard]] Eigen::VectorXd
        ComputeDdf(const Eigen::Ref<const Eigen::Matrix2Xd> &query_points, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel = false)
            const;

        [[nodiscard]] Eigen::MatrixX<Eigen::VectorXd>
        ComputeSddfV1(const common::GridMapInfo2D &grid_map_info, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel = false) const;

        [[nodiscard]] Eigen::VectorXd
        ComputeSddfV1(const Eigen::Ref<const Eigen::Matrix2Xd> &query_points, const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions, bool parallel = false)
            const;

        [[nodiscard]] Eigen::MatrixX<Eigen::VectorXd>
        ComputeSddfV2(
            const common::GridMapInfo2D &grid_map_info,
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
            SignMethod sign_method = SignMethod::kLineNormal,
            bool parallel = false
        ) const;

        [[nodiscard]] Eigen::VectorXd
        ComputeSddfV2(
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_points,
            const Eigen::Ref<const Eigen::Matrix2Xd> &query_directions,
            SignMethod is_negative = SignMethod::kLineNormal,
            bool parallel = false
        ) const;

    private:
        [[nodiscard]] inline bool
        IsNegativeSdf(SignMethod sign_method, const Eigen::Ref<const Eigen::Vector2d> &q, int vertex_0_idx, int vertex_1_idx) const {
            switch (sign_method) {
                case SignMethod::kPointNormal:
                    return IsNegativeSdfByPointNormal(q, vertex_0_idx, vertex_1_idx);
                case SignMethod::kLineNormal:
                    return IsNegativeSdfBySegmentNormal(q, vertex_0_idx, vertex_1_idx);
                case SignMethod::kPolygon:
                    ERL_ASSERTM(
                        m_surface_->OutsideFlagsAvailable(), "outside flags of the polygons are not available. SignMethod::kPolygon cannot be used currently."
                    );
                    return IsNegativeSdfByPolygon(q, vertex_0_idx, vertex_1_idx);
            }

            throw std::runtime_error("Unknown sign method: " + std::to_string(int(sign_method)));
        }

        [[nodiscard]] inline bool
        IsNegativeSdfByPointNormal(const Eigen::Ref<const Eigen::Vector2d> &q, int vertex_0_idx, int) const {
            return m_surface_->normals.col(vertex_0_idx).dot(q - m_surface_->vertices.col(vertex_0_idx)) < 0.;
        }

        [[nodiscard]] inline bool
        IsNegativeSdfBySegmentNormal(const Eigen::Ref<const Eigen::Vector2d> &q, int vertex_0_idx, int vertex_1_idx) const {
            Eigen::Vector2d line_vec = m_surface_->vertices.col(vertex_1_idx) - m_surface_->vertices.col(vertex_0_idx);
            line_vec.normalize();
            Eigen::Vector2d normal = m_surface_->normals.col(vertex_0_idx) - line_vec * (m_surface_->normals.col(vertex_0_idx).dot(line_vec));
            return normal.dot(q - m_surface_->vertices.col(vertex_0_idx)) < 0.;
        }

        [[nodiscard]] inline bool
        IsNegativeSdfByPolygon(const Eigen::Ref<const Eigen::Vector2d> &q, int vertex_0_idx, int) const {
            const auto &kIdxObject = m_surface_->vertices_to_objects(vertex_0_idx);
            bool wn = WindingNumber(q, m_surface_->GetObjectVertices(kIdxObject));
            return wn == m_surface_->outside_flags[kIdxObject];
        }

        void
        ComputeNormals(double delta, bool use_kdtree = false, bool parallel = false);

        void
        ComputeOutsideFlags(const Eigen::Ref<const Eigen::MatrixXd> &map_image, double free_threshold);  // by winding number

        void
        ComputeOutsideFlags();  // by surface normals
    };
}  // namespace erl::geometry

namespace YAML {
    template<>
    struct convert<erl::geometry::Space2D::SignMethod> {
        static Node
        encode(const erl::geometry::Space2D::SignMethod &rhs) {
            return Node(erl::geometry::Space2D::GetSignMethodName(rhs));
        }

        static bool
        decode(const Node &node, erl::geometry::Space2D::SignMethod &rhs) {
            rhs = erl::geometry::Space2D::GetSignMethodFromName(node.as<std::string>());
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::Space2D::SignMethod &rhs) {
        out << erl::geometry::Space2D::GetSignMethodName(rhs);
        return out;
    }
}  // namespace YAML
