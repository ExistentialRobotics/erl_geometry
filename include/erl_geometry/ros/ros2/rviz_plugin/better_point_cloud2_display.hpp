#pragma once

#include <message_filters/subscriber.h>
#include <rviz_common/display.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker/marker_display.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/message_filter.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <QColor>

namespace erl::geometry::rviz_plugin {
    class RVIZ_DEFAULT_PLUGINS_PUBLIC BetterPointCloud2Display
        : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::PointCloud2> {
    private:
        std::unique_ptr<rviz_default_plugins::PointCloudCommon> m_point_cloud_common_ = nullptr;
        std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> m_marker_common_ = nullptr;
        rviz_common::properties::BoolProperty *m_show_normals_property_ = nullptr;
        rviz_common::properties::EnumProperty *m_normal_x_property_ = nullptr;
        rviz_common::properties::EnumProperty *m_normal_y_property_ = nullptr;
        rviz_common::properties::EnumProperty *m_normal_z_property_ = nullptr;
        rviz_common::properties::FloatProperty *m_normal_length_property_ = nullptr;
        rviz_common::properties::FloatProperty *m_normal_width_property_ = nullptr;
        rviz_common::properties::IntProperty *m_normal_start_property_ = nullptr;
        rviz_common::properties::IntProperty *m_normal_stride_property_ = nullptr;
        rviz_common::properties::EnumProperty *m_normal_color_mode_property_ = nullptr;
        rviz_common::properties::ColorProperty *m_normal_color_property_ = nullptr;

        visualization_msgs::msg::Marker::SharedPtr m_marker_;
        bool m_show_normals_ = false;
        std::string m_normal_x_name_ = "normal_x";
        std::string m_normal_y_name_ = "normal_y";
        std::string m_normal_z_name_ = "normal_z";
        float m_normal_length_ = 0.01f;
        float m_normal_width_ = 0.0005f;
        int m_normal_start_ = 0.0;
        int m_normal_stride_ = 10;
        QColor m_normal_color_ = Qt::red;
        std::string m_normal_color_mode_ = "Constant Color";

    public:
        BetterPointCloud2Display();

        void
        reset() override;

        void
        update(float wall_dt, float ros_dt) override;

        void
        onDisable() override;

    protected:
        void
        onInitialize() override;

        void
        processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) override;

        sensor_msgs::msg::PointCloud2::SharedPtr
        ProcessCloud(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
            bool &has_normals,
            float &color_ch_min,
            float &color_ch_max);

    private Q_SLOTS:
        void
        UpdateShowNormals();

        void
        UpdateNormalX();

        void
        UpdateNormalY();

        void
        UpdateNormalZ();

        void
        UpdateNormalLength();

        void
        UpdateNormalWidth();

        void
        UpdateNormalStart();

        void
        UpdateNormalStride();

        void
        UpdateNormalColorMode();

        void
        UpdateNormalColor();

    private:
        static void
        GetRainbowColor(float value, std_msgs::msg::ColorRGBA &color);
    };
}  // namespace erl::geometry::rviz_plugin
