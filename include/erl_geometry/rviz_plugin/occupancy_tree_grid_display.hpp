#pragma once
#ifndef Q_MOC_RUN
    #include "erl_geometry/ros_msgs/occupancy_tree_msg.hpp"
    #include "rviz/ogre_helpers/point_cloud.h"

    #include <message_filters/subscriber.h>
    #include <ros/ros.h>
    #include <rviz/display.h>
#endif

namespace rviz {
    class RosTopicProperty;
    class IntProperty;
    class EnumProperty;
    class FloatProperty;
}  // namespace rviz

namespace erl::geometry::rviz_plugin {

    class OccupancyTreeGridDisplay : public rviz::Display {
        Q_OBJECT

    private:
        typedef std::vector<rviz::PointCloud::Point> VPoint;
        typedef std::vector<VPoint> VVPoint;

        std::shared_ptr<message_filters::Subscriber<erl_geometry::OccupancyTreeMsg> > m_sub_;
        std::mutex m_mutex_;

        // point buffer
        VVPoint m_new_points_;
        VVPoint m_point_buf_;
        bool m_new_points_received_ = false;

        // Ogre-rviz point clouds
        std::vector<rviz::PointCloud*> m_clouds_{};
        std::vector<double> m_box_size_{};
        std_msgs::Header m_header_;

        // Plugin properties
        rviz::IntProperty* m_queue_size_property_ = nullptr;
        rviz::RosTopicProperty* m_tree_topic_property_ = nullptr;
        rviz::EnumProperty* m_tree_render_mode_property_ = nullptr;
        rviz::EnumProperty* m_tree_color_mode_property_ = nullptr;
        rviz::IntProperty* m_tree_depth_property_ = nullptr;
        rviz::FloatProperty* m_alpha_property_ = nullptr;
        rviz::FloatProperty* m_max_height_property_ = nullptr;
        rviz::FloatProperty* m_min_height_property_ = nullptr;

        u_int32_t m_queue_size_ = 0;
        uint32_t m_messages_received_ = 0;
        float m_color_factor_;
        bool m_is_2d_ = false;
        double m_tree_resolution_ = 0.0;

    public:
        OccupancyTreeGridDisplay();
        virtual ~OccupancyTreeGridDisplay();

        // Overrides from Display
        void
        onInitialize() override;
        void
        update(float wall_dt, float ros_dt) override;
        void
        reset() override;

    private Q_SLOTS:
        void
        UpdateQueueSize();
        void
        UpdateTreeTopic();
        void
        UpdateTreeDepth();
        void
        UpdateTreeRenderMode();
        void
        UpdateTreeColorMode();
        void
        UpdateAlpha();
        void
        UpdateMaxHeight();
        void
        UpdateMinHeight();

    protected:
        // overrides from Display
        void
        onEnable() override;
        void
        onDisable() override;

        void
        Subscribe();
        void
        Unsubscribe();

        template<typename Dtype>
        void
        IncomingMessageCallbackForQuadtree(const erl_geometry::OccupancyTreeMsgConstPtr& msg);

        template<typename Dtype>
        void
        IncomingMessageCallbackForOctree(const erl_geometry::OccupancyTreeMsgConstPtr& msg);

        virtual void
        IncomingMessageCallback(const erl_geometry::OccupancyTreeMsgConstPtr& msg);

        void
        SetColor(double z_pos, double min_z, double max_z, rviz::PointCloud::Point& point);

        void
        Clear();

        virtual bool
        UpdateFromTf();
    };
}  // namespace erl::geometry::rviz_plugin
