#include "erl_geometry/ros/ros2/rviz_plugin/better_point_cloud2_display.hpp"

#include "erl_common/fmt.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>

namespace erl::geometry::rviz_plugin {
    BetterPointCloud2Display::BetterPointCloud2Display()
        : m_point_cloud_common_(new rviz_default_plugins::PointCloudCommon(this)),
          m_marker_common_(new rviz_default_plugins::displays::MarkerCommon(this)) {

        // add a property to show normals
        m_show_normals_property_ = new rviz_common::properties::BoolProperty(
            "Show Normals",
            m_show_normals_,
            "Show normals in the point cloud.",
            this,
            SLOT(UpdateShowNormals()),
            this);
        // add a property to set the x component of the normals
        m_normal_x_property_ = new rviz_common::properties::EnumProperty(
            "Normal X",
            "normal_x",
            "Component of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalX()),
            this);
        // add a property to set the y component of the normals
        m_normal_y_property_ = new rviz_common::properties::EnumProperty(
            "Normal Y",
            "normal_y",
            "Component of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalY()),
            this);
        // add a property to set the z component of the normals
        m_normal_z_property_ = new rviz_common::properties::EnumProperty(
            "Normal Z",
            "normal_z",
            "Component of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalZ()),
            this);
        // add a property to set the length of the normals
        m_normal_length_property_ = new rviz_common::properties::FloatProperty(
            "Normal Size",
            m_normal_length_,
            "Size of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalLength()),
            this);
        m_normal_length_property_->setMin(0.0f);
        // add a property to set the shaft diameter of the normals
        m_normal_width_property_ = new rviz_common::properties::FloatProperty(
            "Normal Width",
            m_normal_width_,
            "Width of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalWidth()),
            this);
        m_normal_width_property_->setMin(0.0f);
        // add a property to set the start index to draw the normals
        m_normal_start_property_ = new rviz_common::properties::IntProperty(
            "Normal Start",
            m_normal_start_,
            "Start index of the normals in the point cloud to draw.",
            this,
            SLOT(UpdateNormalStart()),
            this);
        m_normal_start_property_->setMin(0);
        // add a property to set the stride of the normals
        m_normal_stride_property_ = new rviz_common::properties::IntProperty(
            "Normal Stride",
            m_normal_stride_,
            "Stride of the normals in the point cloud to draw.",
            this,
            SLOT(UpdateNormalStride()),
            this);
        m_normal_stride_property_->setMin(1);
        // add a property to set the color mode of the normals
        m_normal_color_mode_property_ = new rviz_common::properties::EnumProperty(
            "Normal Color Mode",
            "Constant Color",
            "Color mode of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalColorMode()),
            this);
        // add a property to set the color of the normals
        m_normal_color_property_ = new rviz_common::properties::ColorProperty(
            "Normal Color",
            m_normal_color_,
            "Color of the normals in the point cloud.",
            this,
            SLOT(UpdateNormalColor()),
            this);

        // prepare marker messages for showing normals
        m_marker_.reset(new visualization_msgs::msg::Marker);
        m_marker_->ns = "normals";
        m_marker_->id = 0;
        m_marker_->lifetime = rclcpp::Duration::from_seconds(0);  // no expiration
        m_marker_->action = visualization_msgs::msg::Marker::ADD;
        m_marker_->type = visualization_msgs::msg::Marker::LINE_LIST;
        m_marker_->color.a = 1.0f;
        m_marker_->pose.orientation.w = 1.0f;
    }

    void
    BetterPointCloud2Display::reset() {
        MFDClass::reset();
        m_point_cloud_common_->reset();
        m_marker_common_->clearMarkers();
    }

    void
    BetterPointCloud2Display::update(float wall_dt, float ros_dt) {
        if (m_point_cloud_common_) { m_point_cloud_common_->update(wall_dt, ros_dt); }
        if (m_marker_common_) { m_marker_common_->update(wall_dt, ros_dt); }
    }

    void
    BetterPointCloud2Display::onDisable() {
        MFDClass::onDisable();
        m_point_cloud_common_->onDisable();
    }

    void
    BetterPointCloud2Display::onInitialize() {
        MFDClass::onInitialize();
        m_point_cloud_common_->initialize(context_, scene_node_);
        m_marker_common_->initialize(context_, scene_node_);
    }

    void
    BetterPointCloud2Display::processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
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
            m_marker_common_->clearMarkers();
            return;
        }

        m_marker_->header = filtered->header;
        m_marker_common_->clearMarkers();

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
        const int32_t xi = rviz_default_plugins::findChannelIndex(filtered, "x");
        const int32_t yi = rviz_default_plugins::findChannelIndex(filtered, "y");
        const int32_t zi = rviz_default_plugins::findChannelIndex(filtered, "z");
        const int32_t normal_xi =
            rviz_default_plugins::findChannelIndex(filtered, m_normal_x_name_);
        const int32_t normal_yi =
            rviz_default_plugins::findChannelIndex(filtered, m_normal_y_name_);
        const int32_t normal_zi =
            rviz_default_plugins::findChannelIndex(filtered, m_normal_z_name_);
        const int32_t color_idx =
            rviz_default_plugins::findChannelIndex(filtered, m_normal_color_mode_);
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
        m_marker_->points.clear();
        m_marker_->colors.clear();
        m_marker_->points.reserve(normals_cnt * 2);
        m_marker_->colors.reserve(normals_cnt * 2);
        std_msgs::msg::ColorRGBA color;
        std_msgs::msg::ColorRGBA color_black;
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
            if (!rviz_common::validateFloats(x) || !rviz_common::validateFloats(y) ||
                !rviz_common::validateFloats(z) || !rviz_common::validateFloats(nx) ||
                !rviz_common::validateFloats(ny) || !rviz_common::validateFloats(nz)) {
                continue;
            }

            geometry_msgs::msg::Point start_point;
            start_point.x = x;
            start_point.y = y;
            start_point.z = z;
            m_marker_->points.push_back(start_point);
            geometry_msgs::msg::Point end_point;
            end_point.x = x + nx * m_normal_length_;
            end_point.y = y + ny * m_normal_length_;
            end_point.z = z + nz * m_normal_length_;
            m_marker_->points.push_back(end_point);
            if (!use_constant_color) {
                float color_value = *reinterpret_cast<const float *>(ptr + color_off);
                color_value = 1.0f - (color_value - color_ch_min) / color_range;
                GetRainbowColor(color_value, color);
            }
            m_marker_->colors.push_back(color);        // start color
            m_marker_->colors.push_back(color_black);  // end color
        }
        m_marker_->scale.x = m_normal_width_;
        m_marker_common_->addMessage(m_marker_);
    }

    sensor_msgs::msg::PointCloud2::SharedPtr
    BetterPointCloud2Display::ProcessCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
        bool &has_normals,
        float &color_ch_min,
        float &color_ch_max) {
        has_normals = false;
        color_ch_min = std::numeric_limits<float>::max();
        color_ch_max = std::numeric_limits<float>::lowest();
        // validate the data size is correct
        if (cloud->data.size() != cloud->width * cloud->height * cloud->point_step) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                fmt::format(
                    "Data size ({} bytes) does not match width ({}) times height ({}) times "
                    "point_step ({}).  Dropping message.",
                    cloud->data.size(),
                    cloud->width,
                    cloud->height,
                    cloud->point_step));
            return nullptr;
        }
        if (cloud->row_step != cloud->width * cloud->point_step) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                fmt::format(
                    "Row step ({}) does not match width ({}) times point step ({}).",
                    cloud->row_step,
                    cloud->width,
                    cloud->point_step));
            return nullptr;
        }

        // validate x, y, z are present
        const int32_t xi = rviz_default_plugins::findChannelIndex(cloud, "x");
        const int32_t yi = rviz_default_plugins::findChannelIndex(cloud, "y");
        const int32_t zi = rviz_default_plugins::findChannelIndex(cloud, "z");
        if (xi < 0 || yi < 0 || zi < 0) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                "Position information not found or not complete.");
            return nullptr;
        }
        if (cloud->fields[xi].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            cloud->fields[yi].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            cloud->fields[zi].datatype != sensor_msgs::msg::PointField::FLOAT32) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                "Position information is not of type FLOAT32.");
            return nullptr;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr filtered =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        const uint32_t xoff = cloud->fields[xi].offset;
        const uint32_t yoff = cloud->fields[yi].offset;
        const uint32_t zoff = cloud->fields[zi].offset;
        const int32_t color_idx =
            rviz_default_plugins::findChannelIndex(cloud, m_normal_color_mode_);
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
                if (rviz_common::validateFloats(x) && rviz_common::validateFloats(y) &&
                    rviz_common::validateFloats(z)) {
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
            int32_t normal_xi = rviz_default_plugins::findChannelIndex(cloud, m_normal_x_name_);
            int32_t normal_yi = rviz_default_plugins::findChannelIndex(cloud, m_normal_y_name_);
            int32_t normal_zi = rviz_default_plugins::findChannelIndex(cloud, m_normal_z_name_);
            if (normal_xi < 0 || normal_yi < 0 || normal_zi < 0) {
                setStatusStd(
                    rviz_common::properties::StatusProperty::Warn,
                    "Message",
                    "Normal information not found or not complete.");  // we just ignore the normals
            } else if (
                cloud->fields[normal_xi].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
                cloud->fields[normal_yi].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
                cloud->fields[normal_zi].datatype != sensor_msgs::msg::PointField::FLOAT32) {
                setStatusStd(
                    rviz_common::properties::StatusProperty::Warn,
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

    void
    BetterPointCloud2Display::UpdateShowNormals() {
        m_show_normals_ = m_show_normals_property_->getBool();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalX() {
        m_normal_x_name_ = m_normal_x_property_->getStdString();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalY() {
        m_normal_y_name_ = m_normal_y_property_->getStdString();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalZ() {
        m_normal_z_name_ = m_normal_z_property_->getStdString();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalLength() {
        m_normal_length_ = m_normal_length_property_->getFloat();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalWidth() {
        m_normal_width_ = m_normal_width_property_->getFloat();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalStart() {
        m_normal_start_ = m_normal_start_property_->getInt();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalStride() {
        m_normal_stride_ = m_normal_stride_property_->getInt();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalColorMode() {
        m_normal_color_mode_ = m_normal_color_mode_property_->getStdString();
        updateTopic();
    }

    void
    BetterPointCloud2Display::UpdateNormalColor() {
        m_normal_color_ = m_normal_color_property_->getColor();
        updateTopic();
    }

    void
    BetterPointCloud2Display::GetRainbowColor(float value, std_msgs::msg::ColorRGBA &color) {
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

}  // namespace erl::geometry::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(erl::geometry::rviz_plugin::BetterPointCloud2Display, rviz_common::Display)
