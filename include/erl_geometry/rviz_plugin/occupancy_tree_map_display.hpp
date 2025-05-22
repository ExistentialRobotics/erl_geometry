#pragma once
#ifndef Q_MOC_RUN
    #include "erl_geometry/ros_msgs/occupancy_tree_msg.hpp"

    #include <message_filters/subscriber.h>
    #include <nav_msgs/OccupancyGrid.h>
    #include <ros/ros.h>
    #include <rviz/default_plugin/map_display.h>
    #include <rviz/properties/float_property.h>
    #include <rviz/properties/int_property.h>
#endif

namespace erl::geometry::rviz_plugin {

    class OccupancyTreeMapDisplay : public rviz::MapDisplay {
        Q_OBJECT

    private:
        std::shared_ptr<message_filters::Subscriber<erl_geometry::OccupancyTreeMsg>> m_sub_;

        rviz::IntProperty* m_tree_depth_property_;
        rviz::FloatProperty* m_max_height_property_ = nullptr;
        rviz::FloatProperty* m_min_height_property_ = nullptr;

        nav_msgs::OccupancyGrid::Ptr m_occupancy_map_ = nullptr;
        double m_tree_resolution_inv_;
        uint32_t m_tree_max_depth_;
        uint32_t m_tree_key_offset_;

    public:
        OccupancyTreeMapDisplay();
        virtual ~OccupancyTreeMapDisplay();

    private Q_SLOTS:
        void
        UpdateTopic();
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
        HandleMessageForQuadtree(const erl_geometry::OccupancyTreeMsgConstPtr& msg);

        template<typename Dtype>
        void
        HandleMessageForOctree(const erl_geometry::OccupancyTreeMsgConstPtr& msg);

        void
        HandleMessage(const erl_geometry::OccupancyTreeMsgConstPtr& msg);

        [[nodiscard]] OctreeKey
        CoordToKey(double x, double y, double z) const;

        [[nodiscard]] OctreeKey
        CoordToKey(double x, double y, double z, uint32_t depth) const;

        [[nodiscard]] void
        KeyToCoord(const OctreeKey& key, double& x, double& y, double& z) const;

        [[nodiscard]] void
        KeyToCoord(const OctreeKey& key, uint32_t depth, double& x, double& y, double& z) const;

        [[nodiscard]] QuadtreeKey
        CoordToKey(double x, double y) const;

        [[nodiscard]] QuadtreeKey
        CoordToKey(double x, double y, uint32_t depth) const;

        [[nodiscard]] void
        KeyToCoord(const QuadtreeKey& key, double& x, double& y) const;

        [[nodiscard]] void
        KeyToCoord(const QuadtreeKey& key, uint32_t depth, double& x, double& y) const;
    };

    // template<typename OcTreeType>
    // class TemplatedOccupancyMapDisplay : public OccupancyTreeMapDisplay {
    // protected:
    //     void
    //     HandleMessage(const erl_geometry::OccupancyTreeMsgConstPtr& msg);
    // };

}  // namespace erl::geometry::rviz_plugin
