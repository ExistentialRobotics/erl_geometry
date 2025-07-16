#include "erl_geometry/rviz_plugin/better_point_cloud2_display.hpp"

#include "erl_common/fmt.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/validate_floats.h>

namespace erl::geometry::rviz_plugin {
    BetterPointCloud2Display::BetterPointCloud2Display()
        : rviz::MarkerDisplay(),
          m_point_cloud_common_(std::make_unique<rviz::PointCloudCommon>(this)) {

        delete marker_topic_property_;
        marker_topic_property_ = new rviz::RosTopicProperty(
            "PointCloud2 Topic",
            "",
            ros::message_traits::datatype<sensor_msgs::PointCloud2>(),
            "sensor_msgs::PointCloud2 topic to subscribe to.",
            this,
            SLOT(UpdateTopic()),
            this);
        MarkerDisplay::unsubscribe();  // turn off marker topic subscription

        // override the MarkerDisplay's queue size property
        delete queue_size_property_;
        queue_size_property_ = new rviz::IntProperty(
            "Queue Size",
            5,
            "Advanced: set the size of the incoming message queue. Increasing this is useful if "
            "your incoming TF data is delayed significantly from your data, but it can greatly "
            "increase memory usage if the messages are big.",
            this,
            SLOT(UpdateQueueSize()),
            this);
    }

    BetterPointCloud2Display::~BetterPointCloud2Display() {
        unsubscribe();
        m_point_cloud_common_->reset();
    }

    void
    BetterPointCloud2Display::fixedFrameChanged() {
        rviz::MarkerDisplay::fixedFrameChanged();
        if (m_point_cloud_common_) { m_point_cloud_common_->fixedFrameChanged(); }
    }

    void
    BetterPointCloud2Display::update(float wall_dt, float ros_dt) {
        rviz::MarkerDisplay::update(wall_dt, ros_dt);
        if (m_point_cloud_common_) { m_point_cloud_common_->update(wall_dt, ros_dt); }
    }

    void
    BetterPointCloud2Display::onInitialize() {
        // Use the threaded queue for processing of incoming messages
        update_nh_.setCallbackQueue(context_->getThreadedQueue());
        rviz::MarkerDisplay::onInitialize();
        m_point_cloud_common_->initialize(context_, scene_node_);

        // add a property to show normals
        m_show_normals_property_ = new rviz::BoolProperty(
            "Show Normals",
            m_show_normals_,
            "Show normals in the point cloud.",
            this,
            SLOT(UpdateShowNormals()),
            this);
        // add a property to set the x component of the normals
        m_normal_x_property_ = new rviz::EnumProperty(
            "Normal X",
            "normal_x",
            "Component of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalX()),
            this);
        // add a property to set the y component of the normals
        m_normal_y_property_ = new rviz::EnumProperty(
            "Normal Y",
            "normal_y",
            "Component of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalY()),
            this);
        // add a property to set the z component of the normals
        m_normal_z_property_ = new rviz::EnumProperty(
            "Normal Z",
            "normal_z",
            "Component of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalZ()),
            this);
        // add a property to set the length of the normals
        m_normal_length_property_ = new rviz::FloatProperty(
            "Normal Size",
            m_normal_length_,
            "Size of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalLength()),
            this);
        m_normal_length_property_->setMin(0.0f);
        // add a property to set the shaft diameter of the normals
        m_normal_width_property_ = new rviz::FloatProperty(
            "Normal Width",
            m_normal_width_,
            "Width of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalWidth()),
            this);
        m_normal_width_property_->setMin(0.0f);
        // add a property to set the start index to draw the normals
        m_normal_start_property_ = new rviz::IntProperty(
            "Normal Start",
            m_normal_start_,
            "Start index of the normals in the point cloud to draw.",
            this,
            SLOT(UpdateNormalStart()),
            this);
        m_normal_start_property_->setMin(0);
        // add a property to set the stride of the normals
        m_normal_stride_property_ = new rviz::IntProperty(
            "Normal Stride",
            m_normal_stride_,
            "Stride of the normals in the point cloud to draw.",
            this,
            SLOT(UpdateNormalStride()),
            this);
        m_normal_stride_property_->setMin(1);
        // add a property to set the color mode of the normals
        m_normal_color_mode_property_ = new rviz::EnumProperty(
            "Normal Color Mode",
            "Constant Color",
            "Color mode of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalColorMode()),
            this);
        // add a property to set the color of the normals
        m_normal_color_property_ = new rviz::ColorProperty(
            "Normal Color",
            m_normal_color_,
            "Color of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalColor()),
            this);

        m_marker_delete_req_.reset(new visualization_msgs::MarkerArray);
        m_marker_delete_req_->markers.resize(1);
        m_marker_add_req_.reset(new visualization_msgs::MarkerArray);
        m_marker_add_req_->markers.resize(1);

        auto &marker_add = m_marker_add_req_->markers[0];
        auto &marker_delete = m_marker_delete_req_->markers[0];

        marker_add.ns = "normals";
        marker_add.id = 0;
        marker_add.lifetime = ros::Duration(0);  // no expiration
        marker_add.action = visualization_msgs::Marker::ADD;
        marker_add.type = visualization_msgs::Marker::LINE_LIST;
        marker_add.color.a = 1.0f;
        marker_add.pose.orientation.w = 1.0f;

        marker_delete = marker_add;
        marker_delete.action = visualization_msgs::Marker::DELETEALL;
    }

    void
    BetterPointCloud2Display::subscribe() {
        if (!isEnabled()) { return; }
        try {
            unsubscribe();
            const std::string topic = m_pcd_topic_property_->getTopicStd();
            if (topic.empty()) { return; }
            m_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
                update_nh_,
                topic,
                queue_size_property_->getInt()));
            m_sub_->registerCallback(
                boost::bind(&BetterPointCloud2Display::IncomingMessage, this, _1));
        } catch (ros::Exception &e) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Topic",
                fmt::format("Error subscribing: ", e.what()));
            return;
        }
    }

    void
    BetterPointCloud2Display::unsubscribe() {
        if (m_sub_) {
            try {
                m_sub_->unsubscribe();
                m_sub_.reset();
            } catch (ros::Exception &e) {
                setStatusStd(
                    rviz::StatusProperty::Error,
                    "Topic",
                    fmt::format("Error unsubscribing: ", e.what()));
            }
        }
    }

    void
    BetterPointCloud2Display::UpdateTopic() {
        unsubscribe();
        reset();
        subscribe();
        context_->queueRender();
    }

    void
    BetterPointCloud2Display::UpdateQueueSize() {
        subscribe();
    }

    void
    BetterPointCloud2Display::UpdateShowNormals() {
        m_show_normals_ = m_show_normals_property_->getBool();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalX() {
        m_normal_x_name_ = m_normal_x_property_->getStdString();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalY() {
        m_normal_y_name_ = m_normal_y_property_->getStdString();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalZ() {
        m_normal_z_name_ = m_normal_z_property_->getStdString();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalLength() {
        m_normal_length_ = m_normal_length_property_->getFloat();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalWidth() {
        m_normal_width_ = m_normal_width_property_->getFloat();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalStart() {
        m_normal_start_ = m_normal_start_property_->getInt();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalStride() {
        m_normal_stride_ = m_normal_stride_property_->getInt();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalColorMode() {
        m_normal_color_mode_ = m_normal_color_mode_property_->getStdString();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalColor() {
        m_normal_color_ = m_normal_color_property_->getColor();
        UpdateTopic();
    }

    void
    BetterPointCloud2Display::GetRainbowColor(float value, std_msgs::ColorRGBA &color) {
        value = std::min(value, 1.0f);
        value = std::max(value, 0.0f);

        float h = value * 5.0f + 1.0f;
        int i = floor(h);
        float f = h - i;
        if (!(i & 1)) f = 1 - f;  // if i is even
        float n = 1 - f;

        if (i <= 1) {
            color.r = n, color.g = 0, color.b = 1;
        } else if (i == 2) {
            color.r = 0, color.g = n, color.b = 1;
        } else if (i == 3) {
            color.r = 0, color.g = 1, color.b = n;
        } else if (i == 4) {
            color.r = n, color.g = 1, color.b = 0;
        } else if (i >= 5) {
            color.r = 1, color.g = n, color.b = 0;
        }
    }

    void
    BetterPointCloud2Display::IncomingMessage(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
        bool has_normals = false;
        float color_ch_min = 0.0f;
        float color_ch_max = 0.0f;
        auto filtered = ProcessCloud(cloud, has_normals, color_ch_min, color_ch_max);
        if (!filtered) { return; }

        m_normal_x_property_->clearOptions();
        m_normal_y_property_->clearOptions();
        m_normal_z_property_->clearOptions();
        m_normal_color_mode_property_->clearOptions();
        m_normal_color_mode_property_->addOptionStd("Constant Color");
        for (auto &field: filtered->fields) {
            m_normal_x_property_->addOptionStd(field.name);
            m_normal_y_property_->addOptionStd(field.name);
            m_normal_z_property_->addOptionStd(field.name);
            m_normal_color_mode_property_->addOptionStd(field.name);
        }

        m_point_cloud_common_->addMessage(filtered);
        if (!m_show_normals_ || !has_normals) {
            MarkerDisplay::incomingMarkerArray(m_marker_delete_req_);
            return;
        }

        auto &marker_add = m_marker_add_req_->markers[0];
        auto &marker_delete = m_marker_delete_req_->markers[0];
        marker_add.header = filtered->header;
        marker_delete.header = filtered->header;
        MarkerDisplay::incomingMarkerArray(m_marker_delete_req_);

        /*
        The arrow type provides two different ways of specifying where the arrow should begin/end:
        Position/Orientation
            Pivot point is around the tip of its tail. Identity orientation points it along the +X
        axis. scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow
        height.

        Start/End Points
            You can also specify a start/end point for the arrow, using the points member. If you
        put points into the points member, it will assume you want to do things this way. The point
        at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the
        end. scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not
        zero, it specifies the head length.
        */
        const uint32_t point_count = filtered->width;
        const uint32_t point_step = filtered->point_step * m_normal_stride_;
        const int32_t xi = rviz::findChannelIndex(filtered, "x");
        const int32_t yi = rviz::findChannelIndex(filtered, "y");
        const int32_t zi = rviz::findChannelIndex(filtered, "z");
        const int32_t normal_xi = rviz::findChannelIndex(filtered, m_normal_x_name_);
        const int32_t normal_yi = rviz::findChannelIndex(filtered, m_normal_y_name_);
        const int32_t normal_zi = rviz::findChannelIndex(filtered, m_normal_z_name_);
        const int32_t color_idx = rviz::findChannelIndex(filtered, m_normal_color_mode_);
        const int32_t x_off = filtered->fields[xi].offset;
        const int32_t y_off = filtered->fields[yi].offset;
        const int32_t z_off = filtered->fields[zi].offset;
        const int32_t normal_x_off = filtered->fields[normal_xi].offset;
        const int32_t normal_y_off = filtered->fields[normal_yi].offset;
        const int32_t normal_z_off = filtered->fields[normal_zi].offset;
        const int32_t color_off = color_idx >= 0 ? filtered->fields[color_idx].offset : 0;
        const uint8_t *ptr = filtered->data.data() + m_normal_start_ * point_step;
        const uint8_t *ptr_end = filtered->data.data() + filtered->data.size();
        const int32_t normals_cnt = (point_count / m_normal_stride_) + 1;
        marker_add.points.clear();
        marker_add.colors.clear();
        marker_add.points.reserve(normals_cnt * 2);
        marker_add.colors.reserve(normals_cnt * 2);
        std_msgs::ColorRGBA color;
        std_msgs::ColorRGBA color_black;
        color_black.r = 0.0f;
        color_black.g = 0.0f;
        color_black.b = 0.0f;
        color_black.a = 1.0f;
        const bool use_constant_color = m_normal_color_mode_ == "Constant Color" && color_idx < 0;
        if (use_constant_color) {
            color.r = m_normal_color_.redF();
            color.g = m_normal_color_.greenF();
            color.b = m_normal_color_.blueF();
            color.a = m_normal_color_.alphaF();
        }
        const float color_range = color_ch_max - color_ch_min;
        for (; ptr < ptr_end; ptr += point_step) {
            float x = *reinterpret_cast<const float *>(ptr + x_off);
            float y = *reinterpret_cast<const float *>(ptr + y_off);
            float z = *reinterpret_cast<const float *>(ptr + z_off);
            float nx = *reinterpret_cast<const float *>(ptr + normal_x_off);
            float ny = *reinterpret_cast<const float *>(ptr + normal_y_off);
            float nz = *reinterpret_cast<const float *>(ptr + normal_z_off);

            float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
            nx /= norm;
            ny /= norm;
            nz /= norm;
            if (!rviz::validateFloats(x) || !rviz::validateFloats(y) || !rviz::validateFloats(z) ||
                !rviz::validateFloats(nx) || !rviz::validateFloats(ny) ||
                !rviz::validateFloats(nz)) {
                continue;
            }

            geometry_msgs::Point start_point;
            start_point.x = x;
            start_point.y = y;
            start_point.z = z;
            marker_add.points.push_back(start_point);
            geometry_msgs::Point end_point;
            end_point.x = x + nx * m_normal_length_;
            end_point.y = y + ny * m_normal_length_;
            end_point.z = z + nz * m_normal_length_;
            marker_add.points.push_back(end_point);
            if (!use_constant_color) {
                float color_value = *reinterpret_cast<const float *>(ptr + color_off);
                color_value = 1.0f - (color_value - color_ch_min) / color_range;
                GetRainbowColor(color_value, color);
            }
            marker_add.colors.push_back(color);        // start color
            marker_add.colors.push_back(color_black);  // end color
        }
        marker_add.scale.x = m_normal_width_;
        MarkerDisplay::incomingMarkerArray(m_marker_add_req_);
    }

    sensor_msgs::PointCloud2Ptr
    BetterPointCloud2Display::ProcessCloud(
        const sensor_msgs::PointCloud2::ConstPtr &cloud,
        bool &has_normals,
        float &color_ch_min,
        float &color_ch_max) {
        has_normals = false;
        color_ch_min = std::numeric_limits<float>::max();
        color_ch_max = std::numeric_limits<float>::lowest();
        // validate the data size is correct
        if (cloud->data.size() != cloud->width * cloud->height * cloud->point_step) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                "Data size does not match width, height, and point step.");
            return nullptr;
        }
        if (cloud->row_step != cloud->width * cloud->point_step) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                "Row step does not match width and point step.");
            return nullptr;
        }

        // validate x, y, z are present
        const int32_t xi = rviz::findChannelIndex(cloud, "x");
        const int32_t yi = rviz::findChannelIndex(cloud, "y");
        const int32_t zi = rviz::findChannelIndex(cloud, "z");
        if (xi < 0 || yi < 0 || zi < 0) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                "Position information not found or not complete.");
            return nullptr;
        }
        if (cloud->fields[xi].datatype != sensor_msgs::PointField::FLOAT32 ||
            cloud->fields[yi].datatype != sensor_msgs::PointField::FLOAT32 ||
            cloud->fields[zi].datatype != sensor_msgs::PointField::FLOAT32) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                "Position information is not of type FLOAT32.");
            return nullptr;
        }

        sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);
        const uint32_t xoff = cloud->fields[xi].offset;
        const uint32_t yoff = cloud->fields[yi].offset;
        const uint32_t zoff = cloud->fields[zi].offset;
        const int32_t color_idx = rviz::findChannelIndex(cloud, m_normal_color_mode_);
        const uint32_t color_ch_off = color_idx >= 0 ? cloud->fields[color_idx].offset : 0;
        const uint32_t point_step = cloud->point_step;
        const size_t point_count = cloud->width * cloud->height;
        filtered->data.resize(cloud->data.size());
        uint32_t output_count = 0;
        if (point_count == 0) {
            output_count = 0;
        } else {
            uint8_t *output_ptr = filtered->data.data();
            const uint8_t *ptr = cloud->data.data();
            const uint8_t *ptr_end = cloud->data.data() + cloud->data.size();
            const uint8_t *ptr_init = nullptr;
            size_t points_to_copy = 0;
            for (; ptr < ptr_end; ptr += point_step) {
                float x = *reinterpret_cast<const float *>(ptr + xoff);
                float y = *reinterpret_cast<const float *>(ptr + yoff);
                float z = *reinterpret_cast<const float *>(ptr + zoff);
                if (rviz::validateFloats(x) && rviz::validateFloats(y) && rviz::validateFloats(z)) {
                    float color = *reinterpret_cast<const float *>(ptr + color_ch_off);
                    if (color < color_ch_min) { color_ch_min = color; }
                    if (color > color_ch_max) { color_ch_max = color; }
                    if (points_to_copy == 0) {
                        ptr_init = ptr;
                        points_to_copy = 1;
                    } else {
                        ++points_to_copy;
                    }
                } else if (points_to_copy > 0) {
                    std::memcpy(output_ptr, ptr_init, point_step * points_to_copy);
                    output_ptr += point_step * points_to_copy;
                    points_to_copy = 0;
                }
            }
            if (points_to_copy > 0) {
                std::memcpy(output_ptr, ptr_init, point_step * points_to_copy);
                output_ptr += point_step * points_to_copy;
            }
            output_count = static_cast<uint32_t>(output_ptr - filtered->data.data()) / point_step;
        }

        if (m_show_normals_) {
            int32_t normal_xi = rviz::findChannelIndex(cloud, m_normal_x_name_);
            int32_t normal_yi = rviz::findChannelIndex(cloud, m_normal_y_name_);
            int32_t normal_zi = rviz::findChannelIndex(cloud, m_normal_z_name_);
            if (normal_xi < 0 || normal_yi < 0 || normal_zi < 0) {
                setStatusStd(
                    rviz::StatusProperty::Error,
                    "Message",
                    "Normal information not found or not complete.");  // we just ignore the normals
            } else if (
                cloud->fields[normal_xi].datatype != sensor_msgs::PointField::FLOAT32 ||
                cloud->fields[normal_yi].datatype != sensor_msgs::PointField::FLOAT32 ||
                cloud->fields[normal_zi].datatype != sensor_msgs::PointField::FLOAT32) {
                setStatusStd(
                    rviz::StatusProperty::Error,
                    "Message",
                    "Normal information is not of type FLOAT32.");  // we just ignore the normals
            } else {
                has_normals = true;  // set the flag to true
            }
        }
        filtered->header = cloud->header;
        filtered->fields = cloud->fields;
        filtered->data.resize(output_count * point_step);
        filtered->height = 1;
        filtered->width = output_count;
        filtered->is_bigendian = cloud->is_bigendian;
        filtered->point_step = point_step;
        filtered->row_step = point_step * output_count;
        filtered->is_dense = cloud->is_dense;
        return filtered;
    }

}  // namespace erl::geometry::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(erl::geometry::rviz_plugin::BetterPointCloud2Display, rviz::Display)
