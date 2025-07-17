#pragma once
#ifndef Q_MOC_RUN
    #include "erl_geometry/ros/ros2/msg/occupancy_tree_msg.hpp"

    #include <message_filters/subscriber.h>
    #include <rclcpp/rclcpp.hpp>
    #include <rviz_common/message_filter_display.hpp>
    #include <rviz_common/properties/enum_property.hpp>
    #include <rviz_common/properties/float_property.hpp>
    #include <rviz_common/properties/int_property.hpp>
    #include <rviz_common/properties/ros_topic_property.hpp>
    #include <rviz_default_plugins/visibility_control.hpp>
    #include <rviz_rendering/objects/point_cloud.hpp>
    #include <std_msgs/msg/header.hpp>
#endif

namespace erl::geometry::rviz_plugin {

    class RVIZ_DEFAULT_PLUGINS_PUBLIC OccupancyTreeGridDisplay
        : public rviz_common::MessageFilterDisplay<erl_geometry::msg::OccupancyTreeMsg> {

        Q_OBJECT

    private:
        using Super = rviz_common::MessageFilterDisplay<erl_geometry::msg::OccupancyTreeMsg>;

        typedef std::vector<rviz_rendering::PointCloud::Point> VPoint;
        typedef std::vector<VPoint> VVPoint;

        std::mutex m_mutex_;

        // point buffer
        VVPoint m_new_points_;
        VVPoint m_point_buf_;
        bool m_new_points_received_ = false;

        // Ogre-rviz point clouds
        std::vector<rviz_rendering::PointCloud*> m_clouds_{};
        std::vector<double> m_box_size_{};
        std_msgs::msg::Header m_header_;

        // Plugin properties
        rviz_common::properties::EnumProperty* m_tree_render_mode_property_ = nullptr;
        rviz_common::properties::EnumProperty* m_tree_color_mode_property_ = nullptr;
        rviz_common::properties::IntProperty* m_tree_depth_property_ = nullptr;
        rviz_common::properties::FloatProperty* m_alpha_property_ = nullptr;
        rviz_common::properties::FloatProperty* m_max_height_property_ = nullptr;
        rviz_common::properties::FloatProperty* m_min_height_property_ = nullptr;

        float m_color_factor_;
        bool m_is_2d_ = false;
        double m_tree_resolution_ = 0.0;

    public:
        OccupancyTreeGridDisplay();
        ~OccupancyTreeGridDisplay();

        // Overrides from Display
        void
        onInitialize() override;
        void
        update(float wall_dt, float ros_dt) override;
        void
        reset() override;

    private Q_SLOTS:
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
        void
        onEnable() override;
        void
        onDisable() override;

        void
        processMessage(erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr msg) override;

        template<typename Dtype>
        void
        ProcessMessageForQuadtree(const erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr& msg);

        template<typename Dtype>
        void
        ProcessMessageForOctree(const erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr& msg);

        void
        SetColor(
            double z_pos,
            double min_z,
            double max_z,
            rviz_rendering::PointCloud::Point& point);

        void
        Clear();

        bool
        UpdateFromTf();
    };
}  // namespace erl::geometry::rviz_plugin
