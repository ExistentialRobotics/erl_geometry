#pragma once
#ifndef Q_MOC_RUN
    #include "erl_geometry/ros/ros2/msg/occupancy_tree_msg.hpp"

    #include <nav_msgs/msg/occupancy_grid.hpp>
    #include <rclcpp/rclcpp.hpp>
    #include <rviz_common/message_filter_display.hpp>
    #include <rviz_common/properties/float_property.hpp>
    #include <rviz_common/properties/int_property.hpp>
    #include <rviz_default_plugins/displays/map/map_display.hpp>
#endif

namespace erl::geometry::rviz_plugin {

    /**
     * @brief Display for visualizing occupancy quadtree / octree as a 2D occupancy grid map.
     * @details This display subscribes to a topic of type erl_geometry::msg::OccupancyTreeMsg and
     * converts the quadtree / octree into a nav_msgs::msg::OccupancyGrid for visualization.
     */

    class OccupancyTreeMapDisplay : public rviz_default_plugins::displays::MapDisplay {
        Q_OBJECT

    private:
        rclcpp::Subscription<erl_geometry::msg::OccupancyTreeMsg>::SharedPtr m_sub_ = nullptr;
        rviz_common::properties::IntProperty* m_tree_depth_property_ = nullptr;
        rviz_common::properties::FloatProperty* m_max_height_property_ = nullptr;
        rviz_common::properties::FloatProperty* m_min_height_property_ = nullptr;

        nav_msgs::msg::OccupancyGrid::SharedPtr m_occupancy_map_ = nullptr;
        double m_tree_resolution_inv_;
        uint32_t m_tree_max_depth_;
        uint32_t m_tree_key_offset_;

    public:
        OccupancyTreeMapDisplay();
        virtual ~OccupancyTreeMapDisplay();

    private Q_SLOTS:
        void
        UpdateTreeDepth();
        void
        UpdateMaxHeight();
        void
        UpdateMinHeight();

    protected:
        void
        onInitialize() override;
        void
        subscribe() override;
        void
        unsubscribe() override;

        template<typename Dtype>
        void
        HandleMessageForQuadtree(const erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr& msg);

        template<typename Dtype>
        void
        HandleMessageForOctree(const erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr& msg);

        void
        HandleMessage(const erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr& msg);

        [[nodiscard]] OctreeKey
        CoordToKey(double x, double y, double z) const;

        [[nodiscard]] OctreeKey
        CoordToKey(double x, double y, double z, uint32_t depth) const;

        void
        KeyToCoord(const OctreeKey& key, double& x, double& y, double& z) const;

        void
        KeyToCoord(const OctreeKey& key, uint32_t depth, double& x, double& y, double& z) const;

        [[nodiscard]] QuadtreeKey
        CoordToKey(double x, double y) const;

        [[nodiscard]] QuadtreeKey
        CoordToKey(double x, double y, uint32_t depth) const;

        void
        KeyToCoord(const QuadtreeKey& key, double& x, double& y) const;

        void
        KeyToCoord(const QuadtreeKey& key, uint32_t depth, double& x, double& y) const;
    };
}  // namespace erl::geometry::rviz_plugin
