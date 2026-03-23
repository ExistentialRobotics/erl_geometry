#pragma once

#include "aabb.hpp"
#include "abstract_occupancy_octree.hpp"
#include "occupancy_nd_tree_batch_ray_caster.hpp"
#include "occupancy_nd_tree_setting.hpp"
#include "octree_impl.hpp"

namespace erl::geometry {

    struct OccupancyOctreeBaseSetting
        : common::Yamlable<OccupancyOctreeBaseSetting, OccupancyNdTreeSetting> {
        bool use_change_detection = false;
        bool use_aabb_limit = false;
        Aabb3Dd aabb = {};

        ERL_REFLECT_SCHEMA(
            OccupancyOctreeBaseSetting,
            ERL_REFLECT_MEMBER(OccupancyOctreeBaseSetting, use_change_detection),
            ERL_REFLECT_MEMBER(OccupancyOctreeBaseSetting, use_aabb_limit),
            ERL_REFLECT_MEMBER(OccupancyOctreeBaseSetting, aabb));

        bool
        operator==(const NdTreeSetting &other) const override;
    };

    template<typename Dtype, class Node, class Setting>
    class OccupancyOctreeBase : public OctreeImpl<Node, AbstractOccupancyOctree<Dtype>, Setting> {
        static_assert(std::is_base_of_v<OccupancyOctreeNode, Node>);
        static_assert(std::is_base_of_v<OccupancyOctreeBaseSetting, Setting>);

        std::shared_ptr<OccupancyOctreeBaseSetting> m_setting_ = nullptr;

    protected:
        OctreeKeyBoolMap m_changed_keys_ = {};
        // buffer used for inserting point cloud to track the end points
        OctreeKeyVectorMap m_end_point_mapping_ = {};

    public:
        using Super = OctreeImpl<Node, AbstractOccupancyOctree<Dtype>, Setting>;
        using Matrix3X = Eigen::Matrix3X<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;
        using VectorX = Eigen::VectorX<Dtype>;

        OccupancyOctreeBase() = delete;  // no default constructor

        explicit OccupancyOctreeBase(const std::shared_ptr<Setting> &setting);

        OccupancyOctreeBase(const OccupancyOctreeBase &other) = default;
        OccupancyOctreeBase &
        operator=(const OccupancyOctreeBase &other) = default;
        OccupancyOctreeBase(OccupancyOctreeBase &&other) noexcept = default;
        OccupancyOctreeBase &
        operator=(OccupancyOctreeBase &&other) noexcept = default;

        [[nodiscard]] std::shared_ptr<AbstractOctree<Dtype>>
        Clone() const override;

        //-- implement abstract methods
        void
        OnDeleteNodeChild(Node *node, Node *child, const OctreeKey &key) override;

        //-- frontier extraction

        /// A 3D frontier mesh representing the boundary between free and unknown space.
        /// Each frontier is a connected component of frontier patches (analogous to
        /// how the quadtree groups edge segments into polylines).
        struct Frontier {
            std::vector<Vector3> vertices;
            std::vector<Eigen::Vector3i> faces;  // each element holds 3 vertex indices
        };

        /// Extract frontiers (boundaries between free and unknown space).
        /// Each frontier is a connected triangle mesh of face patches.
        /// @param min_num_triangles Minimum number of triangles for a frontier to be returned.
        /// @param sort_by_area If true, sort frontiers by total surface area (descending).
        /// @return Vector of frontiers.
        [[nodiscard]] std::vector<Frontier>
        ExtractFrontiers(std::size_t min_num_triangles = 1, bool sort_by_area = true) const;

        /// A 2D frontier polyline from a horizontal slice through the octree.
        /// Each frontier is an ordered polyline (2 x N matrix) of XY vertices.
        using SliceFrontier = Eigen::Matrix2X<Dtype>;

        /// Extract 2D frontiers from a horizontal slice at a given z coordinate.
        /// The slice intersects all free leaves whose z-range contains z_slice.
        /// For each such leaf, the 4 lateral faces (W/E/S/N) are checked for
        /// unknown neighbors, producing edge segments that are chained into
        /// polylines — exactly like the quadtree frontier extraction.
        /// @param z_slice The z coordinate of the horizontal slice.
        /// @param min_num_vertices Minimum number of vertices for a frontier to be returned.
        /// @param sort_by_length If true, sort frontiers by total edge length (descending).
        /// @return Vector of 2D frontier polylines.
        [[nodiscard]] std::vector<SliceFrontier>
        ExtractSliceFrontiers(
            Dtype z_slice,
            std::size_t min_num_vertices = 1,
            bool sort_by_length = true) const;

        /// Extract frontiers within an axis-aligned bounding box.
        /// Free leaves intersecting the AABB are considered; the resulting mesh
        /// may slightly exceed the AABB at cell boundaries.
        /// @param aabb_min_x Minimum x coordinate of the query region.
        /// @param aabb_min_y Minimum y coordinate of the query region.
        /// @param aabb_min_z Minimum z coordinate of the query region.
        /// @param aabb_max_x Maximum x coordinate of the query region.
        /// @param aabb_max_y Maximum y coordinate of the query region.
        /// @param aabb_max_z Maximum z coordinate of the query region.
        /// @param min_num_triangles Minimum number of triangles for a frontier to be returned.
        /// @param sort_by_area If true, sort frontiers by total surface area (descending).
        /// @return Vector of frontiers.
        [[nodiscard]] std::vector<Frontier>
        ExtractFrontiers(
            Dtype aabb_min_x,
            Dtype aabb_min_y,
            Dtype aabb_min_z,
            Dtype aabb_max_x,
            Dtype aabb_max_y,
            Dtype aabb_max_z,
            std::size_t min_num_triangles = 1,
            bool sort_by_area = true) const;

        //-- Sample position
        /**
         * Sample positions from the free space.
         */
        void
        SamplePositions(std::size_t num_positions, std::vector<Vector3> &positions) const;

        //-- insert point cloud
        /**
         * Compute the end point mapping for a point cloud.
         * This function maps each point in the point cloud to its corresponding octree cell.
         * It also computes the range and difference vectors for each point.
         * The results are stored in the provided output parameters.
         * @param points 3xN matrix of points in the world frame.
         * @param sensor_origin 3D vector of the sensor origin in the world frame.
         * @param min_range Minimum range of the sensor. Points closer than this range are ignored.
         * @param max_range Maximum range of the sensor. Points beyond this range are ignored.
         * Non-positive value means no limit.
         * @param discrete Whether to discretize the points. i.e. merge points of the same key
         * @param ranges Output vector of ranges for each filtered point.
         * @param diffs Output vector of difference vectors for each filtered point.
         * @param filtered_points Output matrix of filtered points. If discrete is true, this
         * matrix contains the points for each octree cell. Otherwise, it is the same as points.
         * @param occupied_cells Output vector of occupied cells.
         */
        void
        ComputeOccupiedCells(
            const Eigen::Ref<const Matrix3X> &points,
            const Eigen::Ref<const Vector3> &sensor_origin,
            Dtype min_range,
            Dtype max_range,
            bool discrete,
            std::vector<Dtype> &ranges,
            std::vector<std::array<Dtype, 3>> &diffs,
            Matrix3X &filtered_points,
            OctreeKeyVector &occupied_cells);

        void
        ComputeFreeCells(
            const Eigen::Ref<const Matrix3X> &points,
            const Eigen::Ref<const Vector3> &sensor_origin,
            const std::vector<Dtype> &ranges,
            const std::vector<std::array<Dtype, 3>> &diffs,
            Dtype max_range,
            bool with_count,
            bool parallel);

        /**
         * Insert a point cloud in the world frame. Multiple points may fall into the same voxel
         * updated only once, and occupied nodes are preferred than free ones. This avoids holes and
         * is more efficient than the plain ray insertion of InsertPointCloudRays().
         * @param points 3xN matrix of points in the world frame
         * @param sensor_origin 3D vector of the sensor origin in the world frame
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
            const Eigen::Ref<const Matrix3X> &points,
            const Eigen::Ref<const Vector3> &sensor_origin,
            Dtype min_range,
            Dtype max_range,
            bool with_count,
            bool parallel,
            bool lazy_eval,
            bool discrete);

        /**
         * Insert a point cloud ray by ray. Some cells may be updated multiple times. Benchmark
         * shows that this is slower and less accurate than InsertPointCloud.
         * @param points 3xN matrix of ray end points in the world frame.
         * @param sensor_origin 3D vector of the sensor origin in the world frame.
         * @param min_range Minimum range of the sensor. Points closer than this range are ignored.
         * @param max_range Maximum range of the sensor. Points beyond this range are ignored.
         * Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param lazy_eval Whether to update the occupancy of the nodes immediately. If true, the
         * occupancy is not updated until UpdateInnerOccupancy() is called.
         */
        virtual void
        InsertPointCloudRays(
            const Eigen::Ref<const Matrix3X> &points,
            const Eigen::Ref<const Vector3> &sensor_origin,
            Dtype min_range,
            Dtype max_range,
            bool parallel,
            bool lazy_eval);

        //-- insert ray
        /**
         * Insert a ray from (sx, sy) to (ex, ey) into the tree. The ray is cut at max_range if it
         * is positive.
         * @param sx metric x coordinate of the start point.
         * @param sy metric y coordinate of the start point.
         * @param sz metric z coordinate of the start point.
         * @param ex metric x coordinate of the end point.
         * @param ey metric y coordinate of the end point.
         * @param ez metric z coordinate of the end point.
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
            Dtype sz,
            Dtype ex,
            Dtype ey,
            Dtype ez,
            Dtype min_range,
            Dtype max_range,
            bool lazy_eval);

        //-- cast ray
        OccupancyNdTreeBatchRayCaster<OccupancyOctreeBase, 3>
        GetBatchRayCaster(
            Matrix3X origins,
            Matrix3X directions,
            const VectorX &max_ranges,
            const VectorX &node_paddings,
            const Eigen::VectorXb &bidirectional_flags,
            const Eigen::VectorXb &leaf_only_flags,
            const Eigen::VectorXi &min_node_depths,
            const Eigen::VectorXi &max_node_depths) const;

        void
        CastRays(
            const Eigen::Ref<const Vector3> &position,
            const Eigen::Ref<const Matrix3> &rotation,
            const Eigen::Ref<const VectorX> &azimuth_angles,
            const Eigen::Ref<const VectorX> &elevation_angles,
            bool ignore_unknown,
            Dtype max_range,
            bool prune_rays,  // whether to prune rays after the first hit of the same occupied node
            bool parallel,
            std::vector<std::pair<long, long>>
                &hit_ray_indices,  // (azimuth_idx, elevation_idx) of the hit rays
            std::vector<Vector3> &hit_positions,
            std::vector<const Node *> &hit_nodes) const;

        void
        CastRays(
            const Eigen::Ref<const Matrix3X> &positions,
            const Eigen::Ref<const Matrix3X> &directions,
            bool ignore_unknown,
            Dtype max_range,
            bool prune_rays,
            bool parallel,
            std::vector<long> &hit_ray_indices,
            std::vector<Vector3> &hit_positions,
            std::vector<const Node *> &hit_nodes) const;

        const OccupancyOctreeNode *
        GetHitOccupiedNode(
            Dtype px,
            Dtype py,
            Dtype pz,
            Dtype vx,
            Dtype vy,
            Dtype vz,
            bool ignore_unknown,
            Dtype max_range,
            Dtype &ex,
            Dtype &ey,
            Dtype &ez) const override;

        /**
         * Cast a ray starting from (px, py) along (vx, vy) and get the hit surface point (ex, ey)
         * if the ray hits one.
         * @param px metric x coordinate of the start point
         * @param py metric y coordinate of the start point
         * @param pz metric z coordinate of the start point
         * @param vx x component of the ray direction
         * @param vy y component of the ray direction
         * @param vz z component of the ray direction
         * @param ignore_unknown Whether unknown cells are ignored, i.e., treated as free. If false,
         * the ray casting aborts when an unknown cell is hit and returns false.
         * @param max_range Maximum range after which the ray casting is aborted. Non-positive value
         * means no limit.
         * @param ex metric x coordinate of the hit leaf cell
         * @param ey metric y coordinate of the hit leaf cell
         * @param ez metric z coordinate of the hit leaf cell
         * @return node pointer if the ray hits an occupied cell, nullptr otherwise.
         */
        const Node *
        CastRay(
            Dtype px,
            Dtype py,
            Dtype pz,
            Dtype vx,
            Dtype vy,
            Dtype vz,
            bool ignore_unknown,
            Dtype max_range,
            Dtype &ex,
            Dtype &ey,
            Dtype &ez) const;

        //-- trace ray
        [[nodiscard]] const OctreeKeyBoolMap &
        GetChangedKeys() const;

        void
        ClearChangedKeys();

        [[nodiscard]] const OctreeKeyVectorMap &
        GetEndPointMaps() const;

        //-- update nodes' occupancy
        /**
         * Update the node at the given key with the given log-odds delta.
         * @param x
         * @param y
         * @param z
         * @param occupied
         * @param lazy_eval Whether update of inner nodes is omitted and only leaf nodes are
         * updated. This speeds up the intersection, but you need to call UpdateInnerOccupancy()
         * after all updates are done.
         * @return
         */
        Node *
        UpdateNode(Dtype x, Dtype y, Dtype z, bool occupied, bool lazy_eval);

        /**
         * Update the occupancy measurement of a given node
         * @param key of the node to update
         * @param occupied whether the node is observed occupied or not
         * @param lazy_eval Whether update of inner nodes is omitted and only leaf nodes are
         * updated. This speeds up the intersection, but you need to call UpdateInnerOccupancy()
         * after all updates are done.
         */
        Node *
        UpdateNode(const OctreeKey &key, bool occupied, bool lazy_eval);

        Node *
        UpdateNode(Dtype x, Dtype y, Dtype z, float log_odds_delta, bool lazy_eval);

        Node *
        UpdateNode(const OctreeKey &key, float log_odds_delta, bool lazy_eval);

    private:
        template<typename LeafIterator>
        [[nodiscard]] std::vector<Frontier>
        ExtractFrontiersImpl(
            LeafIterator it,
            LeafIterator it_end,
            std::size_t min_num_triangles,
            bool sort_by_area) const;

        Node *
        UpdateNodeRecurs(
            Node *node,
            bool node_just_created,
            bool node_from_expansion,
            const OctreeKey &key,
            Dtype log_odds_delta,
            bool lazy_eval);

    protected:
        void
        UpdateNodeLogOdds(Node *node, float log_odd_delta);

    public:
        void
        UpdateInnerOccupancy();

    protected:
        void
        UpdateInnerOccupancyRecurs(Node *node, uint32_t depth);

        void
        UpdateInnerNodeOccupancy(Node *node);

    public:
        /**
         * Set all nodes' log odds according to their current max likelihood of occupancy.
         */
        void
        ToMaxLikelihood() override;

    protected:
        //--file IO
        bool
        ReadBinaryData(std::istream &s) override;

        bool
        WriteBinaryData(std::ostream &s) const override;
    };

}  // namespace erl::geometry

#include "occupancy_octree_base.tpp"
