#pragma once

#include <message_filters/subscriber.h>
#include <rviz/default_plugin/marker_display.h>
#include <rviz/default_plugin/marker_utils.h>
#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/message_filter.h>
#include <visualization_msgs/Marker.h>

#include <memory>
#include <QColor>

namespace erl::geometry::rviz_plugin {
    class BetterPointCloud2Display : public rviz::MarkerDisplay {
        Q_OBJECT
    private:
        std::unique_ptr<rviz::PointCloudCommon> m_point_cloud_common_ = nullptr;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_sub_ = nullptr;
        rviz::RosTopicProperty *&m_pcd_topic_property_ = marker_topic_property_;  // for clarity
        rviz::BoolProperty *m_show_normals_property_ = nullptr;
        rviz::EnumProperty *m_normal_x_property_ = nullptr;
        rviz::EnumProperty *m_normal_y_property_ = nullptr;
        rviz::EnumProperty *m_normal_z_property_ = nullptr;
        rviz::FloatProperty *m_normal_length_property_ = nullptr;
        rviz::FloatProperty *m_normal_width_property_ = nullptr;
        rviz::IntProperty *m_normal_start_property_ = nullptr;
        rviz::IntProperty *m_normal_stride_property_ = nullptr;
        rviz::EnumProperty *m_normal_color_mode_property_ = nullptr;
        rviz::ColorProperty *m_normal_color_property_ = nullptr;

        visualization_msgs::MarkerArrayPtr m_marker_delete_req_;
        visualization_msgs::MarkerArrayPtr m_marker_add_req_;
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
        ~BetterPointCloud2Display() override;

        void
        fixedFrameChanged() override;

        void
        update(float wall_dt, float ros_dt) override;

    protected:
        void
        onInitialize() override;

        void
        subscribe() override;

        void
        unsubscribe() override;

        void
        IncomingMessage(const sensor_msgs::PointCloud2::ConstPtr &cloud);

        sensor_msgs::PointCloud2Ptr
        ProcessCloud(
            const sensor_msgs::PointCloud2::ConstPtr &cloud,
            bool &has_normals,
            float &color_ch_min,
            float &color_ch_max);

    private Q_SLOTS:
        void
        UpdateTopic();

        void
        UpdateQueueSize();

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

        static void
        GetRainbowColor(float value, std_msgs::ColorRGBA &color);
    };
}  // namespace erl::geometry::rviz_plugin
