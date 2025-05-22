#include "erl_geometry/rviz_plugin/occupancy_tree_map_display.hpp"

#include "erl_geometry/occupancy_octree_base.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"

#include <rviz/default_plugin/map_display.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/visualization_manager.h>

namespace erl::geometry::rviz_plugin {

    static const std::size_t kMaxTreeDepth = 16;

    OccupancyTreeMapDisplay::OccupancyTreeMapDisplay()
        : rviz::MapDisplay() {

        topic_property_->setName("Tree Topic");
        topic_property_->setMessageType(
            QString::fromStdString(
                ros::message_traits::datatype<erl_geometry::OccupancyTreeMsg>()));
        topic_property_->setDescription("erl_geometry::OccupancyTreeMsg topic to subscribe to.");

        m_tree_depth_property_ = new rviz::IntProperty(
            "Max. Octree Depth",
            kMaxTreeDepth,
            "Defines the maximum tree depth to display.",
            this,
            SLOT(UpdateTreeDepth()));

        m_max_height_property_ = new rviz::FloatProperty(
            "Max. Height",
            std::numeric_limits<float>::infinity(),
            "Defines the maximum height to display",
            this,
            SLOT(UpdateMaxHeight()));

        m_min_height_property_ = new rviz::FloatProperty(
            "Min. Height",
            -std::numeric_limits<float>::infinity(),
            "Defines the minimum height to display",
            this,
            SLOT(UpdateMinHeight()));

        m_occupancy_map_.reset(new nav_msgs::OccupancyGrid());
    }

    OccupancyTreeMapDisplay::~OccupancyTreeMapDisplay() { unsubscribe(); }

    void
    OccupancyTreeMapDisplay::onInitialize() {
        rviz::MapDisplay::onInitialize();
    }

    void
    OccupancyTreeMapDisplay::UpdateTreeDepth() {
        UpdateTopic();  // trigger a redraw
    }

    void
    OccupancyTreeMapDisplay::UpdateMaxHeight() {
        UpdateTopic();
    }

    void
    OccupancyTreeMapDisplay::UpdateMinHeight() {
        UpdateTopic();
    }

    void
    OccupancyTreeMapDisplay::UpdateTopic() {
        unsubscribe();
        reset();
        subscribe();
        context_->queueRender();
    }

    void
    OccupancyTreeMapDisplay::subscribe() {
        if (!isEnabled()) { return; }

        try {
            unsubscribe();
            const std::string& topicStr = topic_property_->getStdString();
            if (!topicStr.empty()) {
                m_sub_.reset(new message_filters::Subscriber<erl_geometry::OccupancyTreeMsg>());
                m_sub_->subscribe(threaded_nh_, topicStr, 5);
                m_sub_->registerCallback(
                    boost::bind(&OccupancyTreeMapDisplay::HandleMessage, this, _1));
            }
        } catch (ros::Exception& e) {
            setStatus(
                rviz::StatusProperty::Error,
                "Topic",
                (std::string("Error subscribing: ") + e.what()).c_str());
        }
    }

    void
    OccupancyTreeMapDisplay::unsubscribe() {
        clear();

        try {
            // reset filters
            m_sub_.reset();
        } catch (ros::Exception& e) {
            setStatus(
                rviz::StatusProperty::Error,
                "Topic",
                (std::string("Error unsubscribing: ") + e.what()).c_str());
        }
    }

    template<typename Dtype>
    void
    OccupancyTreeMapDisplay::HandleMessageForQuadtree(
        const erl_geometry::OccupancyTreeMsgConstPtr& msg) {

        auto tree_setting = std::make_shared<OccupancyQuadtreeBaseSetting>();
        auto abstract_tree = AbstractQuadtree<Dtype>::CreateTree(msg->tree_type, tree_setting);
        auto tree = std::dynamic_pointer_cast<AbstractOccupancyQuadtree<Dtype>>(abstract_tree);
        if (tree == nullptr) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                fmt::format(
                    "Wrong occupancy tree type %s. Use a different display type.",
                    msg->tree_type));
            return;
        }
        ROS_DEBUG("Received OccupancyTreeMsg (size: %d bytes)", static_cast<int>(msg->data.size()));

        if (!LoadFromOccupancyTreeMsg<Dtype>(*msg, tree)) {
            setStatusStd(
                rviz::StatusProperty::Error,
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
                setStatusStd(rviz::StatusProperty::Error, "Message", "Failed to get node.");
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
        this->incomingMap(m_occupancy_map_);
    }

    template<typename Dtype>
    void
    OccupancyTreeMapDisplay::HandleMessageForOctree(
        const erl_geometry::OccupancyTreeMsgConstPtr& msg) {

        auto tree_setting = std::make_shared<OccupancyOctreeBaseSetting>();
        auto abstract_tree = AbstractOctree<Dtype>::CreateTree(msg->tree_type, tree_setting);
        auto tree = std::dynamic_pointer_cast<AbstractOccupancyOctree<Dtype>>(abstract_tree);
        if (tree == nullptr) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                fmt::format(
                    "Wrong occupancy tree type %s. Use a different display type.",
                    msg->tree_type));
            return;
        }
        ROS_DEBUG("Received OccupancyTreeMsg (size: %d bytes)", static_cast<int>(msg->data.size()));

        if (!LoadFromOccupancyTreeMsg<Dtype>(*msg, tree)) {
            setStatusStd(
                rviz::StatusProperty::Error,
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
                setStatusStd(rviz::StatusProperty::Error, "Message", "Failed to get node.");
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
        this->incomingMap(m_occupancy_map_);
    }

    void
    OccupancyTreeMapDisplay::HandleMessage(const erl_geometry::OccupancyTreeMsgConstPtr& msg) {
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
                rviz::StatusProperty::Error,
                "Message",
                fmt::format("Unsupported dimension %d.", msg->dim));
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

    // template<typename OcTreeType>
    // void
    // TemplatedOccupancyMapDisplay<OcTreeType>::HandleMessage(
    //     const erl_geometry::OccupancyTreeMsgConstPtr& msg) {
    // }

}  // namespace erl::geometry::rviz_plugin

#include <pluginlib/class_list_macros.h>
// typedef octomap_rviz_plugin::TemplatedOccupancyMapDisplay<octomap::OcTree> OcTreeMapDisplay;
// typedef octomap_rviz_plugin::TemplatedOccupancyMapDisplay<octomap::OcTreeStamped>
//     OcTreeStampedMapDisplay;

// PLUGINLIB_EXPORT_CLASS(OcTreeMapDisplay, rviz::Display)
// PLUGINLIB_EXPORT_CLASS(OcTreeStampedMapDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(erl::geometry::rviz_plugin::OccupancyTreeMapDisplay, rviz::Display)
