#pragma once

#include "aabb.hpp"
#include "abstract_occupancy_quadtree.hpp"
#include "occupancy_nd_tree_batch_ray_caster.hpp"
#include "occupancy_nd_tree_setting.hpp"
#include "quadtree_impl.hpp"

namespace erl::geometry {

    struct OccupancyQuadtreeBaseSetting
        : common::Yamlable<OccupancyQuadtreeBaseSetting, OccupancyNdTreeSetting> {
        bool use_change_detection = false;
        bool use_aabb_limit = false;
        Aabb2Dd aabb = {};

        bool
        operator==(const NdTreeSetting& other) const override;
    };

    template<typename Dtype, class Node, class Setting>
    class OccupancyQuadtreeBase
        : public QuadtreeImpl<Node, AbstractOccupancyQuadtree<Dtype>, Setting> {
        static_assert(std::is_base_of_v<OccupancyQuadtreeNode, Node>);
        static_assert(std::is_base_of_v<OccupancyQuadtreeBaseSetting, Setting>);

        std::shared_ptr<OccupancyQuadtreeBaseSetting> m_setting_ = nullptr;

    protected:
        QuadtreeKeyBoolMap m_changed_keys_ = {};
        // buffer used for inserting point cloud to track the end points
        QuadtreeKeyVectorMap m_end_point_mapping_ = {};

    public:
        using Super = QuadtreeImpl<Node, AbstractOccupancyQuadtree<Dtype>, Setting>;
        using Matrix2X = Eigen::Matrix2X<Dtype>;
        using Matrix2 = Eigen::Matrix2<Dtype>;
        using Vector2 = Eigen::Vector2<Dtype>;
        using VectorX = Eigen::VectorX<Dtype>;

        OccupancyQuadtreeBase() = delete;  // no default constructor

        explicit OccupancyQuadtreeBase(const std::shared_ptr<Setting>& setting);

        OccupancyQuadtreeBase(
            std::shared_ptr<Setting> setting,
            const std::shared_ptr<common::GridMapInfo2D<Dtype>>& map_info,
            const cv::Mat& image_map,
            Dtype occupied_threshold,
            int padding = 0);

        OccupancyQuadtreeBase(const OccupancyQuadtreeBase& other) = default;
        OccupancyQuadtreeBase&
        operator=(const OccupancyQuadtreeBase& other) = default;
        OccupancyQuadtreeBase(OccupancyQuadtreeBase&& other) noexcept = default;
        OccupancyQuadtreeBase&
        operator=(OccupancyQuadtreeBase&& other) noexcept = default;

        [[nodiscard]] std::shared_ptr<AbstractQuadtree<Dtype>>
        Clone() const override;

        //-- implement abstract methods
        void
        OnDeleteNodeChild(Node* node, Node* child, const QuadtreeKey& /*key*/) override;

        //-- Sample position
        /**
         * Sample positions from the free space.
         */
        void
        SamplePositions(std::size_t num_positions, std::vector<Vector2>& positions) const;

        //-- insert point cloud
    private:
        /**
         * Compute the end point mapping for a point cloud.
         * This function maps each point in the point cloud to its corresponding quadtree cell.
         * It also computes the range and difference vectors for each point.
         * The results are stored in the provided output parameters.
         * @param points 2xN matrix of points in the world frame.
         * @param sensor_origin 2D vector of the sensor origin in the world frame.
         * @param min_range Minimum range of the sensor. Points closer than this range are ignored.
         * @param max_range Maximum range of the sensor. Points beyond this range are ignored.
         * Non-positive value means no limit.
         * @param discrete Whether to discretize the points. i.e. merge points of the same key
         * @param ranges Output vector of ranges for each filtered point.
         * @param diffs Output vector of difference vectors for each filtered point.
         * @param filtered_points Output matrix of filtered points. If discrete is true, this
         * matrix contains the points for each quadtree cell. Otherwise, it is the same as points.
         * @param occupied_cells Output vector of occupied cells.
         */
        void
        ComputeOccupiedCells(
            const Eigen::Ref<const Matrix2X>& points,
            const Eigen::Ref<const Vector2>& sensor_origin,
            Dtype min_range,
            Dtype max_range,
            bool discrete,
            std::vector<Dtype>& ranges,
            std::vector<std::array<Dtype, 2>>& diffs,
            Matrix2X& filtered_points,
            QuadtreeKeyVector& occupied_cells);

        void
        ComputeFreeCells(
            const Eigen::Ref<const Matrix2X>& points,
            const Eigen::Ref<const Vector2>& sensor_origin,
            const std::vector<Dtype>& ranges,
            const std::vector<std::array<Dtype, 2>>& diffs,
            Dtype max_range,
            bool parallel);

    public:
        /**
         * Insert a point cloud in the world frame. Multiple points may fall into the same voxel
         * updated only once, and occupied nodes are preferred than free ones. This avoids
         * holes and is more efficient than the plain ray insertion of InsertPointCloudRays().
         * @param points 2xN matrix of points in the world frame
         * @param sensor_origin 2D vector of the sensor origin in the world frame
         * @param min_range Minimum range of the sensor. Points closer than this range are ignored.
         * @param max_range Maximum range of the sensor. Points beyond this range are ignored.
         * Non-positive value means no limit.
         * @param with_count whether to update nodes with consideration of the number of rays that
         * pass through or hit the node.
         * @param parallel whether to use parallel computation
         * @param lazy_eval Whether to update the occupancy of the nodes later. If true, the
         * occupancy is not updated until UpdateInnerOccupancy() is called.
         * @param discrete
         */
        virtual void
        InsertPointCloud(
            const Eigen::Ref<const Matrix2X>& points,
            const Eigen::Ref<const Vector2>& sensor_origin,
            Dtype min_range,
            Dtype max_range,
            bool with_count,
            bool parallel,
            bool lazy_eval,
            bool discrete);

        /**
         * Insert a point cloud ray by ray. Some cells may be updated multiple times. Benchmark
         * shows that this is slower and less accurate than InsertPointCloud.
         * @param points 2xN matrix of ray end points in the world frame.
         * @param sensor_origin 2D vector of the sensor origin in the world frame.
         * @param min_range Minimum range of the sensor. Points closer than this range are ignored.
         * @param max_range Maximum range of the sensor. Points beyond this range are ignored.
         * Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param lazy_eval Whether to update the occupancy of the nodes immediately. If true, the
         * occupancy is not updated until UpdateInnerOccupancy() is called.
         */
        virtual void
        InsertPointCloudRays(
            const Eigen::Ref<const Matrix2X>& points,
            const Eigen::Ref<const Vector2>& sensor_origin,
            Dtype min_range,
            Dtype max_range,
            bool parallel,
            bool lazy_eval);

        //-- insert ray
        /**
         * Insert a ray from (sx, sy) to (ex, ey) into the tree. The ray is cut at max_range if it
         * is positive.
         * @param sx metric x coordinate of the start point
         * @param sy metric y coordinate of the start point
         * @param ex metric x coordinate of the end point
         * @param ey metric y coordinate of the end point
         * @param min_range Minimum range to consider a hit.
         * @param max_range Maximum range after which the ray is cut. Non-positive value means no
         * limit.
         * @param lazy_eval Whether to update the occupancy of the nodes immediately. If true, the
         * occupancy is not updated until UpdateInnerOccupancy() is called.
         * @return
         */
        virtual bool
        InsertRay(
            Dtype sx,
            Dtype sy,
            Dtype ex,
            Dtype ey,
            Dtype min_range,
            Dtype max_range,
            bool lazy_eval);

        //-- cast ray
        OccupancyNdTreeBatchRayCaster<OccupancyQuadtreeBase, 2>
        GetBatchRayCaster(
            Matrix2X origins,
            Matrix2X directions,
            const VectorX& max_ranges,
            const VectorX& node_paddings,
            const Eigen::VectorXb& bidirectional_flags,
            const Eigen::VectorXb& leaf_only_flags,
            const Eigen::VectorXi& min_node_depths,
            const Eigen::VectorXi& max_node_depths) const;

        void
        CastRays(
            const Eigen::Ref<const Vector2>& position,
            const Eigen::Ref<const Matrix2>& rotation,
            const Eigen::Ref<const VectorX>& angles,
            bool ignore_unknown,
            Dtype max_range,
            bool prune_rays,
            bool parallel,
            std::vector<long>& hit_ray_indices,
            std::vector<Vector2>& hit_positions,
            std::vector<const Node*>& hit_nodes) const;

        void
        CastRays(
            const Eigen::Ref<const Matrix2X>& positions,
            const Eigen::Ref<const Matrix2X>& directions,
            bool ignore_unknown,
            Dtype max_range,
            bool prune_rays,
            bool parallel,
            std::vector<long>& hit_ray_indices,
            std::vector<Vector2>& hit_positions,
            std::vector<const Node*>& hit_nodes) const;

        const OccupancyQuadtreeNode*
        GetHitOccupiedNode(
            Dtype px,
            Dtype py,
            Dtype vx,
            Dtype vy,
            bool ignore_unknown,
            Dtype max_range,
            Dtype& ex,
            Dtype& ey) const override;

        /**
         * Cast a ray starting from (px, py) along (vx, vy) and get the hit surface point (ex, ey)
         * if the ray hits one.
         * @param px metric x coordinate of the start point
         * @param py metric y coordinate of the start point
         * @param vx x component of the ray direction
         * @param vy y component of the ray direction
         * @param ignore_unknown Whether unknown cells are ignored, i.e., treated as free. If false,
         * the ray casting aborts when an unknown cell is hit and returns false.
         * @param max_range Maximum range after which the ray casting is aborted. Non-positive value
         * means no limit.
         * @param ex metric x coordinate of the hit leaf cell
         * @param ey metric y coordinate of the hit leaf cell
         * @return node pointer if the ray hits an occupied cell, nullptr otherwise.
         */
        const Node*
        CastRay(
            Dtype px,
            Dtype py,
            Dtype vx,
            Dtype vy,
            bool ignore_unknown,
            Dtype max_range,
            Dtype& ex,
            Dtype& ey) const;

        //-- trace ray
        [[nodiscard]] const QuadtreeKeyBoolMap&
        GetChangedKeys() const;

        void
        ClearChangedKeys();

        [[nodiscard]] const QuadtreeKeyVectorMap&
        GetEndPointMaps() const;

        //-- update nodes' occupancy
        /**
         * Update the node at the given key with the given log-odds delta.
         * @param x
         * @param y
         * @param occupied
         * @param lazy_eval Whether update of inner nodes is omitted and only leaf nodes are
         * updated. This speeds up the intersection, but you need to call UpdateInnerOccupancy()
         * after all updates are done.
         * @return
         */
        Node*
        UpdateNode(Dtype x, Dtype y, bool occupied, bool lazy_eval);

        /**
         * Update the occupancy measurement of a given node
         * @param key of the node to update
         * @param occupied whether the node is observed occupied or not
         * @param lazy_eval Whether update of inner nodes is omitted and only leaf nodes are
         * updated. This speeds up the intersection, but you need to call UpdateInnerOccupancy()
         * after all updates are done.
         */
        Node*
        UpdateNode(const QuadtreeKey& key, bool occupied, bool lazy_eval);

        Node*
        UpdateNode(Dtype x, Dtype y, float log_odds_delta, bool lazy_eval);

        Node*
        UpdateNode(const QuadtreeKey& key, float log_odds_delta, bool lazy_eval);

    private:
        Node*
        UpdateNodeRecurs(
            Node* node,
            bool node_just_created,
            bool node_from_expansion,
            const QuadtreeKey& key,
            float log_odds_delta,
            bool lazy_eval);

    protected:
        void
        UpdateNodeLogOdds(Node* node, float log_odd_delta);

    public:
        void
        UpdateInnerOccupancy();

    protected:
        void
        UpdateInnerOccupancyRecurs(Node* node, uint32_t depth);

        void
        UpdateInnerNodeOccupancy(Node* node);

    public:
        /**
         * Set all nodes' log odds according to their current max likelihood of occupancy.
         */
        void
        ToMaxLikelihood() override;

        //--file IO
        bool
        ReadBinaryData(std::istream& s) override;

        bool
        WriteBinaryData(std::ostream& s) const override;
    };

}  // namespace erl::geometry

#include "occupancy_quadtree_base.tpp"

template<>
struct YAML::convert<erl::geometry::OccupancyQuadtreeBaseSetting> {
    static Node
    encode(const erl::geometry::OccupancyQuadtreeBaseSetting& setting);

    static bool
    decode(const Node& node, erl::geometry::OccupancyQuadtreeBaseSetting& setting);
};  // namespace YAML
