#include "erl_geometry/ros/ros2/rviz_plugin/occupancy_tree_map_display.hpp"

#include "erl_geometry/occupancy_octree_base.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"

#include <rosidl_runtime_cpp/traits.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_default_plugins/displays/map/map_display.hpp>

namespace erl::geometry::rviz_plugin {

    static const std::size_t kMaxTreeDepth = 16;

    OccupancyTreeMapDisplay::OccupancyTreeMapDisplay()
        : rviz_default_plugins::displays::MapDisplay() {

        // change the msg type from nav_msgs::msg::OccupancyGrid to
        // erl_geometry::msg::OccupancyTreeMsg
        QString message_type = rosidl_generator_traits::name<erl_geometry::msg::OccupancyTreeMsg>();
        topic_property_->setName("Tree Topic");
        topic_property_->setMessageType(message_type);
        topic_property_->setDescription(message_type + " topic to subscribe to.");

        m_tree_depth_property_ = new rviz_common::properties::IntProperty(
            "Max. Octree Depth",
            kMaxTreeDepth,
            "Defines the maximum tree depth to display.",
            this,
            SLOT(UpdateTreeDepth()));

        m_max_height_property_ = new rviz_common::properties::FloatProperty(
            "Max. Height",
            std::numeric_limits<float>::infinity(),
            "Defines the maximum height to display",
            this,
            SLOT(UpdateMaxHeight()));

        m_min_height_property_ = new rviz_common::properties::FloatProperty(
            "Min. Height",
            -std::numeric_limits<float>::infinity(),
            "Defines the minimum height to display",
            this,
            SLOT(UpdateMinHeight()));

        m_occupancy_map_.reset(new nav_msgs::msg::OccupancyGrid());
    }

    OccupancyTreeMapDisplay::~OccupancyTreeMapDisplay() { unsubscribe(); }

    void
    OccupancyTreeMapDisplay::onInitialize() {
        rviz_default_plugins::displays::MapDisplay::onInitialize();
    }

    void
    OccupancyTreeMapDisplay::UpdateTreeDepth() {
        updateTopic();  // trigger a redraw
    }

    void
    OccupancyTreeMapDisplay::UpdateMaxHeight() {
        updateTopic();
    }

    void
    OccupancyTreeMapDisplay::UpdateMinHeight() {
        updateTopic();
    }

    void
    OccupancyTreeMapDisplay::subscribe() {
        if (!isEnabled()) { return; }

        if (topic_property_->isEmpty()) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Topic",
                QString("Error subscribing: Empty topic name"));
            return;
        }

        try {
            unsubscribe();

            rclcpp::SubscriptionOptions sub_opts;
            sub_opts.event_callbacks.message_lost_callback = [&](rclcpp::QOSMessageLostInfo& info) {
                setStatus(
                    rviz_common::properties::StatusProperty::Warn,
                    "Topic",
                    QString(
                        fmt::format(
                            "Some messages were lost:\n>\tNumber of new lost messages: "
                            "{}\n>\tTotal number of messages lost: {}",
                            info.total_count_change,
                            info.total_count)
                            .c_str()));
            };

            rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();
            m_sub_ = node->template create_subscription<erl_geometry::msg::OccupancyTreeMsg>(
                topic_property_->getTopicStd(),
                qos_profile,
                [this](const typename erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr message) {
                    HandleMessage(message);
                },
                sub_opts);
            subscription_start_time_ = node->now();
            setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
        } catch (std::exception& e) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Topic",
                QString("Error subscribing: ") + e.what());
        }
    }

    void
    OccupancyTreeMapDisplay::unsubscribe() {
        clear();  // clear the map

        try {
            m_sub_.reset();  // reset filters
        } catch (std::exception& e) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Topic",
                QString("Error unsubscribing: ") + e.what());
        }
    }

    template<typename Dtype>
    void
    OccupancyTreeMapDisplay::HandleMessageForQuadtree(
        const erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr& msg) {

        auto tree_setting = std::make_shared<OccupancyQuadtreeBaseSetting>();
        auto abstract_tree = AbstractQuadtree<Dtype>::CreateTree(msg->tree_type, tree_setting);
        auto tree = std::dynamic_pointer_cast<AbstractOccupancyQuadtree<Dtype>>(abstract_tree);
        if (tree == nullptr) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                fmt::format(
                    "Wrong occupancy tree type %s. Use a different display type.",
                    msg->tree_type));
            return;
        }
        RCLCPP_DEBUG(
            rviz_ros_node_.lock()->get_raw_node()->get_logger(),
            "Received OccupancyTreeMsg (size: %d bytes)",
            static_cast<int>(msg->data.size()));

        if (!LoadFromOccupancyTreeMsg<Dtype>(*msg, tree)) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                "Failed to load occupancy tree from message.");
            return;
        }

        m_tree_resolution_inv_ = 1.0 / tree->GetResolution();
        const uint32_t tree_depth = tree->GetTreeDepth();
        m_tree_max_depth_ = tree_depth;
        m_tree_key_offset_ = 1 << (tree_depth - 1);
        m_tree_depth_property_->setMax(static_cast<int>(tree_depth));
        auto selected_depth = std::min<uint32_t>(tree_depth, m_tree_depth_property_->getInt());
        int depth_shift = tree_depth - selected_depth;  // depth shift

        // get dimensions of quadtree
        Dtype min_x, min_y, max_x, max_y;
        tree->GetMetricMinMax(min_x, min_y, max_x, max_y);
        double res = tree->GetNodeSize(selected_depth);
        min_x -= res;
        min_y -= res;
        auto width = static_cast<uint32_t>(std::ceil((max_x - min_x) / res) + 1);
        auto height = static_cast<uint32_t>(std::ceil((max_y - min_y) / res) + 1);
        QuadtreeKey padded_min_key = CoordToKey(min_x, min_y, selected_depth);

        m_occupancy_map_->header = msg->header;
        m_occupancy_map_->info.resolution = res;
        m_occupancy_map_->info.width = width;
        m_occupancy_map_->info.height = height;
        KeyToCoord(
            padded_min_key,
            selected_depth,
            m_occupancy_map_->info.origin.position.x,
            m_occupancy_map_->info.origin.position.y);
        m_occupancy_map_->info.origin.position.x -= res * 0.5;
        m_occupancy_map_->info.origin.position.y -= res * 0.5;
        m_occupancy_map_->info.origin.position.z = 0.0;
        m_occupancy_map_->data.clear();
        m_occupancy_map_->data.resize(width * height, -1);

        // traverse all leafs in the tree:
        for (auto it = tree->GetTreeIterator(selected_depth); it->IsValid(); it->Next()) {
            const auto* node = static_cast<const OccupancyQuadtreeNode*>(it->GetNode());
            if (node == nullptr) {
                setStatusStd(
                    rviz_common::properties::StatusProperty::Error,
                    "Message",
                    "Failed to get node.");
                return;
            }
            if (node->HasAnyChild()) { continue; }  // skip inner nodes
            const bool occupied = tree->IsNodeOccupied(node);
            int int_size = 1 << (selected_depth - it->GetDepth());
            QuadtreeKey key = it->GetIndexKey();
            if (int_size == 1) {  // the selected depth, map grid size is 1x1
                int pos_x = (key[0] - padded_min_key[0]) >> depth_shift;
                int pos_y = (key[1] - padded_min_key[1]) >> depth_shift;
                int idx = width * pos_y + pos_x;
                ERL_ASSERTM(idx >= 0, "idx: %d", idx);
                if (occupied) {
                    float prob = node->GetOccupancy();
                    m_occupancy_map_->data[idx] = static_cast<int8_t>(std::round(prob * 100));
                } else if (m_occupancy_map_->data[idx] == -1) {
                    m_occupancy_map_->data[idx] = 0;
                }
            } else {  // map grid size is int_size x int_size
                int half_int_size = int_size >> 1;
                int pos_cx = (key[0] - padded_min_key[0]) >> depth_shift;
                int pos_cy = (key[1] - padded_min_key[1]) >> depth_shift;
                if (selected_depth == m_tree_max_depth_) {
                    --pos_cx;
                    --pos_cy;
                }
                for (int dy = half_int_size; dy > -half_int_size; --dy) {
                    int pos_y = std::max<int>(0, pos_cy + dy);
                    int idx0 = width * pos_y;
                    for (int dx = half_int_size; dx > -half_int_size; --dx) {
                        int pos_x = std::max<int>(0, pos_cx + dx);
                        int idx = idx0 + pos_x;
                        if (occupied) {
                            float prob = node->GetOccupancy();
                            m_occupancy_map_->data[idx] =
                                static_cast<int8_t>(std::round(prob * 100));
                        } else if (m_occupancy_map_->data[idx] == -1) {
                            m_occupancy_map_->data[idx] = 0;
                        }
                    }
                }
            }
        }
        this->processMessage(m_occupancy_map_);
    }

    template<typename Dtype>
    void
    OccupancyTreeMapDisplay::HandleMessageForOctree(
        const erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr& msg) {

        auto tree_setting = std::make_shared<OccupancyOctreeBaseSetting>();
        auto abstract_tree = AbstractOctree<Dtype>::CreateTree(msg->tree_type, tree_setting);
        auto tree = std::dynamic_pointer_cast<AbstractOccupancyOctree<Dtype>>(abstract_tree);
        if (tree == nullptr) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                fmt::format(
                    "Wrong occupancy tree type %s. Use a different display type.",
                    msg->tree_type));
            return;
        }
        RCLCPP_DEBUG(
            rviz_ros_node_.lock()->get_raw_node()->get_logger(),
            "Received OccupancyTreeMsg (size: %d bytes)",
            static_cast<int>(msg->data.size()));

        if (!LoadFromOccupancyTreeMsg<Dtype>(*msg, tree)) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                "Failed to load occupancy tree from message.");
            return;
        }

        m_tree_resolution_inv_ = 1.0 / tree->GetResolution();
        const uint32_t tree_depth = tree->GetTreeDepth();
        m_tree_max_depth_ = tree_depth;
        m_tree_key_offset_ = 1 << (tree_depth - 1);
        m_tree_depth_property_->setMax(static_cast<int>(tree_depth));
        auto selected_depth = std::min<uint32_t>(tree_depth, m_tree_depth_property_->getInt());
        int depth_shift = tree_depth - selected_depth;  // depth shift

        // get dimensions of octree
        Dtype min_x, min_y, min_z, max_x, max_y, max_z;
        tree->GetMetricMinMax(min_x, min_y, min_z, max_x, max_y, max_z);
        double res = tree->GetNodeSize(selected_depth);
        min_x -= res;
        min_y -= res;
        auto width = static_cast<uint32_t>(std::ceil((max_x - min_x) / res) + 1);
        auto height = static_cast<uint32_t>(std::ceil((max_y - min_y) / res) + 1);
        OctreeKey padded_min_key = CoordToKey(min_x, min_y, min_z, selected_depth);

        m_occupancy_map_->header = msg->header;
        m_occupancy_map_->info.resolution = res;
        m_occupancy_map_->info.width = width;
        m_occupancy_map_->info.height = height;
        KeyToCoord(
            padded_min_key,
            selected_depth,
            m_occupancy_map_->info.origin.position.x,
            m_occupancy_map_->info.origin.position.y,
            m_occupancy_map_->info.origin.position.z);
        m_occupancy_map_->info.origin.position.x -= res * 0.5;
        m_occupancy_map_->info.origin.position.y -= res * 0.5;
        m_occupancy_map_->info.origin.position.z = 0.0;
        m_occupancy_map_->data.clear();
        m_occupancy_map_->data.resize(width * height, -1);

        // traverse all leafs in the tree:
        const double max_height = std::min<double>(m_max_height_property_->getFloat(), max_z);
        const double min_height = std::max<double>(m_min_height_property_->getFloat(), min_z);

        for (auto it = tree->GetTreeIterator(selected_depth); it->IsValid(); it->Next()) {
            const auto* node = static_cast<const OccupancyOctreeNode*>(it->GetNode());
            if (node == nullptr) {
                setStatusStd(
                    rviz_common::properties::StatusProperty::Error,
                    "Message",
                    "Failed to get node.");
                return;
            }
            if (node->HasAnyChild()) { continue; }  // skip inner nodes
            double z = it->GetZ();
            if (z > max_height || z < min_height) { continue; }  // skip out of range voxels
            const bool occupied = tree->IsNodeOccupied(node);
            int int_size = 1 << (selected_depth - it->GetDepth());
            OctreeKey key = it->GetIndexKey();
            if (int_size == 1) {  // the selected depth, map grid size is 1x1
                int pos_x = (key[0] - padded_min_key[0]) >> depth_shift;
                int pos_y = (key[1] - padded_min_key[1]) >> depth_shift;
                int idx = width * pos_y + pos_x;
                if (occupied) {
                    float prob = node->GetOccupancy();
                    m_occupancy_map_->data[idx] = static_cast<int8_t>(std::round(prob * 100));
                } else if (m_occupancy_map_->data[idx] == -1) {
                    m_occupancy_map_->data[idx] = 0;
                }
            } else {  // map grid size is int_size x int_size
                int half_int_size = int_size >> 1;
                int pos_cx = (key[0] - padded_min_key[0]) >> depth_shift;
                int pos_cy = (key[1] - padded_min_key[1]) >> depth_shift;
                if (selected_depth == m_tree_max_depth_) {
                    --pos_cx;
                    --pos_cy;
                }
                for (int dx = half_int_size; dx > -half_int_size; --dx) {
                    for (int dy = half_int_size; dy > -half_int_size; --dy) {
                        int pos_x = std::max<int>(0, pos_cx + dx);
                        int pos_y = std::max<int>(0, pos_cy + dy);
                        int idx = width * pos_y + pos_x;
                        if (occupied) {
                            float prob = node->GetOccupancy();
                            m_occupancy_map_->data[idx] =
                                static_cast<int8_t>(std::round(prob * 100));
                        } else if (m_occupancy_map_->data[idx] == -1) {
                            m_occupancy_map_->data[idx] = 0;
                        }
                    }
                }
            }
        }
        this->processMessage(m_occupancy_map_);
    }

    void
    OccupancyTreeMapDisplay::HandleMessage(
        const erl_geometry::msg::OccupancyTreeMsg::ConstSharedPtr& msg) {
        if (msg->dim == 2) {
            if (msg->is_double) {
                HandleMessageForQuadtree<double>(msg);
            } else {
                HandleMessageForQuadtree<float>(msg);
            }
        } else if (msg->dim == 3) {
            if (msg->is_double) {
                HandleMessageForOctree<double>(msg);
            } else {
                HandleMessageForOctree<float>(msg);
            }
        } else {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                fmt::format("Unsupported dimension {}.", msg->dim));
        }
    }

    /// implementation copied from octree_impl.tpp

    OctreeKey
    OccupancyTreeMapDisplay::CoordToKey(double x, double y, double z) const {
        OctreeKey key;
        key[0] = static_cast<uint32_t>(std::floor(x * m_tree_resolution_inv_)) + m_tree_key_offset_;
        key[1] = static_cast<uint32_t>(std::floor(y * m_tree_resolution_inv_)) + m_tree_key_offset_;
        key[2] = static_cast<uint32_t>(std::floor(z * m_tree_resolution_inv_)) + m_tree_key_offset_;
        return key;
    }

    OctreeKey
    OccupancyTreeMapDisplay::CoordToKey(double x, double y, double z, uint32_t depth) const {
        const uint32_t diff = m_tree_max_depth_ - depth;
        if (!diff) { return CoordToKey(x, y, z); }
        OctreeKey key;
        uint32_t offset = static_cast<uint32_t>(1 << (diff - 1)) + m_tree_key_offset_;
        key[0] = static_cast<uint32_t>(std::floor(x * m_tree_resolution_inv_));
        key[0] = ((key[0] >> diff) << diff) + offset;
        key[1] = static_cast<uint32_t>(std::floor(y * m_tree_resolution_inv_));
        key[1] = ((key[1] >> diff) << diff) + offset;
        key[2] = static_cast<uint32_t>(std::floor(z * m_tree_resolution_inv_));
        key[2] = ((key[2] >> diff) << diff) + offset;
        return key;
    }

    void
    OccupancyTreeMapDisplay::KeyToCoord(const OctreeKey& key, double& x, double& y, double& z)
        const {
        const double r = 1.0 / m_tree_resolution_inv_;
        double diff_x = static_cast<double>(key[0]) - static_cast<double>(m_tree_key_offset_);
        double diff_y = static_cast<double>(key[1]) - static_cast<double>(m_tree_key_offset_);
        double diff_z = static_cast<double>(key[2]) - static_cast<double>(m_tree_key_offset_);
        x = (diff_x + 0.5) * r;
        y = (diff_y + 0.5) * r;
        z = (diff_z + 0.5) * r;
    }

    void
    OccupancyTreeMapDisplay::KeyToCoord(
        const OctreeKey& key,
        uint32_t depth,
        double& x,
        double& y,
        double& z) const {
        if (depth == 0) {
            x = 0.0;
            y = 0.0;
            z = 0.0;
            return;
        }
        const uint32_t diff = m_tree_max_depth_ - depth;
        if (diff == 0) {
            KeyToCoord(key, x, y, z);
            return;
        }
        double dsize = 1 << diff;
        double r = dsize / m_tree_resolution_inv_;
        double diff_x = static_cast<double>(key[0]) - static_cast<double>(m_tree_key_offset_);
        double diff_y = static_cast<double>(key[1]) - static_cast<double>(m_tree_key_offset_);
        double diff_z = static_cast<double>(key[2]) - static_cast<double>(m_tree_key_offset_);
        x = (std::floor(diff_x / dsize) + 0.5) * r;
        y = (std::floor(diff_y / dsize) + 0.5) * r;
        z = (std::floor(diff_z / dsize) + 0.5) * r;
    }

    /// implementation copied from quadtree_impl.tpp

    QuadtreeKey
    OccupancyTreeMapDisplay::CoordToKey(double x, double y) const {
        QuadtreeKey key;
        key[0] = static_cast<uint32_t>(std::floor(x * m_tree_resolution_inv_)) + m_tree_key_offset_;
        key[1] = static_cast<uint32_t>(std::floor(y * m_tree_resolution_inv_)) + m_tree_key_offset_;
        return key;
    }

    QuadtreeKey
    OccupancyTreeMapDisplay::CoordToKey(double x, double y, uint32_t depth) const {
        const uint32_t diff = m_tree_max_depth_ - depth;
        if (!diff) { return CoordToKey(x, y); }
        QuadtreeKey key;
        uint32_t offset = static_cast<uint32_t>(1 << (diff - 1)) + m_tree_key_offset_;
        key[0] = static_cast<uint32_t>(std::floor(x * m_tree_resolution_inv_));
        key[0] = ((key[0] >> diff) << diff) + offset;
        key[1] = static_cast<uint32_t>(std::floor(y * m_tree_resolution_inv_));
        key[1] = ((key[1] >> diff) << diff) + offset;
        return key;
    }

    void
    OccupancyTreeMapDisplay::KeyToCoord(const QuadtreeKey& key, double& x, double& y) const {
        const double r = 1.0 / m_tree_resolution_inv_;
        double diff_x = static_cast<double>(key[0]) - static_cast<double>(m_tree_key_offset_);
        double diff_y = static_cast<double>(key[1]) - static_cast<double>(m_tree_key_offset_);
        x = (diff_x + 0.5) * r;
        y = (diff_y + 0.5) * r;
    }

    void
    OccupancyTreeMapDisplay::KeyToCoord(
        const QuadtreeKey& key,
        uint32_t depth,
        double& x,
        double& y) const {
        if (depth == 0) {
            x = 0.0;
            y = 0.0;
            return;
        }
        const uint32_t diff = m_tree_max_depth_ - depth;
        if (diff == 0) {
            KeyToCoord(key, x, y);
            return;
        }
        double dsize = 1 << diff;
        double r = dsize / m_tree_resolution_inv_;
        double diff_x = static_cast<double>(key[0]) - static_cast<double>(m_tree_key_offset_);
        double diff_y = static_cast<double>(key[1]) - static_cast<double>(m_tree_key_offset_);
        x = (std::floor(diff_x / dsize) + 0.5) * r;
        y = (std::floor(diff_y / dsize) + 0.5) * r;
    }
}  // namespace erl::geometry::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(erl::geometry::rviz_plugin::OccupancyTreeMapDisplay, rviz_common::Display)
