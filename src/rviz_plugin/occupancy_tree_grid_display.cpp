// Code adpoted from octomap_rviz_plugin
#include "erl_geometry/rviz_plugin/occupancy_tree_grid_display.hpp"

#include "erl_common/fmt.hpp"
#include "erl_geometry/abstract_occupancy_octree.hpp"
#include "erl_geometry/abstract_occupancy_quadtree.hpp"
#include "erl_geometry/colored_occupancy_octree_node.hpp"
#include "erl_geometry/colored_occupancy_quadtree_node.hpp"
#include "erl_geometry/occupancy_octree_base.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"

#include <boost/bind.hpp>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/status_property.h>
#include <rviz/visualization_manager.h>

#include <QObject>
#include <sstream>

namespace erl::geometry::rviz_plugin {

    static const std::size_t kMaxTreeDepth = 16;

    enum VoxelRenderMode { FREE_VOXELS = 1, OCCUPIED_VOXELS = 2 };

    enum VoxelColorMode { CELL_COLOR = 0, Z_AXIS_COLOR = 1, PROBABLILTY_COLOR = 2 };

    OccupancyTreeGridDisplay::OccupancyTreeGridDisplay()
        : rviz::Display(),
          m_new_points_received_(false),
          m_queue_size_(5),
          m_messages_received_(0),
          m_color_factor_(0.8) {

        m_queue_size_property_ = new rviz::IntProperty(
            "Queue Size",
            m_queue_size_,
            "Advanced: set the size of the incoming message queue.  Increasing this "
            "is useful if your incoming TF data is delayed significantly from your"
            " image data, but it can greatly increase memory usage if the messages are big.",
            this,
            SLOT(UpdateQueueSize()));
        m_queue_size_property_->setMin(1);

        m_tree_topic_property_ = new rviz::RosTopicProperty(
            "Tree Topic",
            "",
            QString::fromStdString(ros::message_traits::datatype<erl_geometry::OccupancyTreeMsg>()),
            "erl_geometry::OccupancyTreeMsg topic to subscribe to (binary or full probability map)",
            this,
            SLOT(UpdateTreeTopic()));

        m_tree_render_mode_property_ = new rviz::EnumProperty(
            "Voxel Rendering",
            "Occupied Voxels",
            "Select voxel type.",
            this,
            SLOT(UpdateTreeRenderMode()));

        m_tree_render_mode_property_->addOption("Occupied Voxels", OCCUPIED_VOXELS);
        m_tree_render_mode_property_->addOption("Free Voxels", FREE_VOXELS);
        m_tree_render_mode_property_->addOption("All Voxels", FREE_VOXELS | OCCUPIED_VOXELS);

        m_tree_color_mode_property_ = new rviz::EnumProperty(
            "Voxel Coloring",
            "Z-Axis",
            "Select voxel coloring mode",
            this,
            SLOT(UpdateTreeColorMode()));

        m_tree_color_mode_property_->addOption("Cell Color", CELL_COLOR);
        m_tree_color_mode_property_->addOption("Z-Axis", Z_AXIS_COLOR);
        m_tree_color_mode_property_->addOption("Cell Probability", PROBABLILTY_COLOR);
        m_alpha_property_ = new rviz::FloatProperty(
            "Voxel Alpha",
            1.0,
            "Set voxel transparency alpha",
            this,
            SLOT(UpdateAlpha()));
        m_alpha_property_->setMin(0.0);
        m_alpha_property_->setMax(1.0);

        m_tree_depth_property_ = new rviz::IntProperty(
            "Max. Octree Depth",
            kMaxTreeDepth,
            "Defines the maximum tree depth",
            this,
            SLOT(UpdateTreeDepth()));
        m_tree_depth_property_->setMin(0);
        m_tree_depth_property_->setMax(kMaxTreeDepth);

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
    }

    OccupancyTreeGridDisplay::~OccupancyTreeGridDisplay() {
        Unsubscribe();
        for (auto it = m_clouds_.begin(); it != m_clouds_.end(); ++it) { delete *(it); }
        if (scene_node_) { scene_node_->detachAllObjects(); }
    }

    void
    OccupancyTreeGridDisplay::onInitialize() {
        std::lock_guard<std::mutex> guard(m_mutex_);
        m_box_size_.resize(kMaxTreeDepth);
        m_clouds_.resize(kMaxTreeDepth);
        m_point_buf_.resize(kMaxTreeDepth);
        m_new_points_.resize(kMaxTreeDepth);

        for (std::size_t i = 0; i < kMaxTreeDepth; ++i) {
            m_clouds_[i] = new rviz::PointCloud();
            m_clouds_[i]->setName(fmt::format("PointCloud Nr.{}", i));
            m_clouds_[i]->setRenderMode(rviz::PointCloud::RM_BOXES);
            scene_node_->attachObject(m_clouds_[i]);  // rviz::Display::scene_node_
        }
    }

    void
    OccupancyTreeGridDisplay::update(float /* wall_dt */, float /* ros_dt */) {
        if (m_new_points_received_) {
            std::lock_guard<std::mutex> lock(m_mutex_);

            for (size_t i = 0; i < kMaxTreeDepth; ++i) {
                double size = m_box_size_[i];
                m_clouds_[i]->clear();
                if (m_is_2d_) {
                    m_clouds_[i]->setDimensions(size, size, m_tree_resolution_);
                } else {
                    m_clouds_[i]->setDimensions(size, size, size);
                }
                m_clouds_[i]->addPoints(m_new_points_[i].data(), m_new_points_[i].size());
                m_new_points_[i].clear();
                m_clouds_[i]->setAlpha(m_alpha_property_->getFloat());
            }
            m_new_points_received_ = false;
        }
        UpdateFromTf();
    }

    void
    OccupancyTreeGridDisplay::reset() {
        Clear();
        m_messages_received_ = 0;
        setStatus(
            rviz::StatusProperty::Ok,
            "Messages",
            QString("0 binary octomap messages received"));
    }

    void
    OccupancyTreeGridDisplay::UpdateQueueSize() {
        m_queue_size_ = m_queue_size_property_->getInt();
        Subscribe();
    }

    void
    OccupancyTreeGridDisplay::UpdateTreeTopic() {
        Unsubscribe();
        reset();
        Subscribe();
        context_->queueRender();
    }

    void
    OccupancyTreeGridDisplay::UpdateTreeDepth() {
        UpdateTreeTopic();
    }

    void
    OccupancyTreeGridDisplay::UpdateTreeRenderMode() {
        UpdateTreeTopic();
    }

    void
    OccupancyTreeGridDisplay::UpdateTreeColorMode() {
        UpdateTreeTopic();
    }

    void
    OccupancyTreeGridDisplay::UpdateAlpha() {
        UpdateTreeTopic();
    }

    void
    OccupancyTreeGridDisplay::UpdateMaxHeight() {
        UpdateTreeTopic();
    }

    void
    OccupancyTreeGridDisplay::UpdateMinHeight() {
        UpdateTreeTopic();
    }

    void
    OccupancyTreeGridDisplay::onEnable() {
        scene_node_->setVisible(true);
        Subscribe();
    }

    void
    OccupancyTreeGridDisplay::onDisable() {
        scene_node_->setVisible(false);
        Unsubscribe();
        Clear();
    }

    void
    OccupancyTreeGridDisplay::Subscribe() {
        if (!isEnabled()) { return; }

        try {
            Unsubscribe();
            const std::string& kTopicStr = m_tree_topic_property_->getStdString();
            if (!kTopicStr.empty()) {
                m_sub_.reset(new message_filters::Subscriber<erl_geometry::OccupancyTreeMsg>());
                m_sub_->subscribe(threaded_nh_, kTopicStr, m_queue_size_);
                m_sub_->registerCallback(
                    boost::bind(&OccupancyTreeGridDisplay::IncomingMessageCallback, this, _1));
            }
        } catch (ros::Exception& e) {
            setStatus(
                rviz::StatusProperty::Error,
                "Topic",
                (std::string("Error subscribing: ") + e.what()).c_str());
        }
    }

    void
    OccupancyTreeGridDisplay::Unsubscribe() {
        Clear();

        try {
            m_sub_.reset();  // reset filters
        } catch (ros::Exception& e) {
            setStatus(
                rviz::StatusProperty::Error,
                "Topic",
                (std::string("Error unsubscribing: ") + e.what()).c_str());
        }
    }

    template<typename Dtype>
    void
    OccupancyTreeGridDisplay::IncomingMessageCallbackForQuadtree(
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

        m_header_ = msg->header;
        if (!UpdateFromTf()) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                fmt::format(
                    "Failed to transform from frame [%s] to frame [%s]",
                    m_header_.frame_id,
                    context_->getFrameManager()->getFixedFrame()));
            return;
        }

        // deserialize the message
        if (!LoadFromOccupancyTreeMsg<Dtype>(*msg, tree)) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                "Failed to deserialize quadtree message.");
            return;
        }
        const uint32_t tree_depth = tree->GetTreeDepth();
        m_tree_depth_property_->setMax(static_cast<int>(tree_depth));

        // get dimensions of quadtree
        Dtype min_x, min_y, max_x, max_y;
        tree->GetMetricMinMax(min_x, min_y, max_x, max_y);

        // reset rviz pointcloud classes
        for (uint32_t i = 0; i < kMaxTreeDepth; ++i) {
            m_point_buf_[i].clear();
            if (i < tree_depth) { m_box_size_[i] = tree->GetNodeSize(i + 1); }  // skip depth 0
        }

        std::size_t point_count = 0;
        auto selected_depth = std::min<uint32_t>(tree_depth, m_tree_depth_property_->getInt());
        float z = m_min_height_property_->getFloat();              // quadtree is 2D
        if (z < -100.0f) { z = 0.0; }                              // default height
        const int step_size = 1 << (tree_depth - selected_depth);  // for pruning of occluded voxels
        const int render_mode_mask = m_tree_render_mode_property_->getOptionInt();

        for (auto it = tree->GetTreeIterator(selected_depth); it->IsValid(); it->Next()) {
            const auto* node = static_cast<const OccupancyQuadtreeNode*>(it->GetNode());
            if (node == nullptr) {
                setStatusStd(rviz::StatusProperty::Error, "Message", "Failed to get node.");
                return;
            }
            if (node->HasAnyChild()) { continue; }  // skip inner nodes
            // render mode: 0b01 for free voxels, 0b10 for occupied voxels
            auto node_render_mode = static_cast<int>(tree->IsNodeOccupied(node)) + 1;
            if (!(node_render_mode & render_mode_mask)) { continue; }  // skip non-rendered voxels
            QuadtreeKey node_key = it->GetKey();
            bool all_neighbors_found = true;
            const uint32_t node_depth = it->GetDepth();
            int diff_base = (node_depth < tree_depth) ? 1 << (tree_depth - node_depth - 1) : 1;
            int diff[2] = {-((node_depth == tree_depth) ? diff_base : diff_base + 1), diff_base};
            // check if current voxel has neighbors on all sides -> no need to be displayed
            QuadtreeKey key;
            for (int axis = 0; axis < 2; ++axis) {
                int idx_0 = axis % 2;
                int idx_1 = (axis + 1) % 2;
                for (int i = 0; i < 2; ++i) {
                    key[idx_0] = node_key[idx_0] + diff[i];
                    // if rendering is restricted to tree_depth < maximum tree depth
                    // inner nodes with distance `step_size` can already occlude a voxel
                    for (key[idx_1] = node_key[idx_1] + diff[0] + 1;
                         key[idx_1] < node_key[idx_1] + diff[1];
                         node_key[idx_1] += step_size) {
                        const auto* neighbor_node = static_cast<const OccupancyQuadtreeNode*>(
                            tree->SearchNode(node_key, selected_depth));
                        if (!neighbor_node ||
                            !((static_cast<int>(tree->IsNodeOccupied(neighbor_node)) + 1) &
                              render_mode_mask)) {
                            all_neighbors_found = false;
                            break;
                        }
                    }
                    if (!all_neighbors_found) { break; }
                }
                if (!all_neighbors_found) { break; }
            }
            if (all_neighbors_found) { continue; }  // skip occluded voxels

            // display voxel if it does not have all the neighbors.
            rviz::PointCloud::Point new_point;
            new_point.position.x = it->GetX();
            new_point.position.y = it->GetY();
            new_point.position.z = z;
            // set color
            switch (static_cast<VoxelColorMode>(m_tree_color_mode_property_->getOptionInt())) {
                case CELL_COLOR: {
                    auto color_node = static_cast<const ColoredOccupancyQuadtreeNode*>(node);
                    if (color_node) {
                        auto& color = color_node->GetColor();
                        new_point.setColor(
                            static_cast<float>(color[0]) / 255.0f,
                            static_cast<float>(color[1]) / 255.0f,
                            static_cast<float>(color[2]) / 255.0f,
                            m_alpha_property_->getFloat());
                        break;
                    }
                    setStatus(
                        rviz::StatusProperty::Error,
                        "Messages",
                        QString(
                            "Cannot extract color, node is not derived from "
                            "ColoredOccupancyOctreeNode."));
                    [[fallthrough]];
                }
                case Z_AXIS_COLOR: {
                    if (tree->IsNodeOccupied(node)) {
                        SetColor(it->GetX(), min_x, max_x, new_point);
                    } else {
                        SetColor(it->GetY(), min_x, max_x, new_point);
                    }
                    break;
                }
                case PROBABLILTY_COLOR: {
                    float probability = node->GetOccupancy();
                    SetColor(probability, 0.0f, 1.0f, new_point);
                    break;
                }
                default:
                    break;
            }
            // push to point vectors
            m_point_buf_[node_depth - 1].push_back(new_point);
            ++point_count;
        }

        if (point_count) {  // swap the new points with the old ones
            std::lock_guard<std::mutex> lock(m_mutex_);
            m_new_points_received_ = true;
            for (size_t i = 0; i < kMaxTreeDepth; ++i) { m_new_points_[i].swap(m_point_buf_[i]); }
            m_is_2d_ = true;
            m_tree_resolution_ = tree->GetResolution();
        }
    }

    template<typename Dtype>
    void
    OccupancyTreeGridDisplay::IncomingMessageCallbackForOctree(
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

        m_header_ = msg->header;
        if (!UpdateFromTf()) {
            std::stringstream ss;
            ss << "Failed to transform from frame [" << m_header_.frame_id << "] to frame ["
               << context_->getFrameManager()->getFixedFrame() << "]";
            setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
            return;
        }

        // deserialize the message
        if (!LoadFromOccupancyTreeMsg<Dtype>(*msg, tree)) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                "Failed to deserialize octree message.");
            return;
        }

        const uint32_t tree_depth = tree->GetTreeDepth();
        m_tree_depth_property_->setMax(static_cast<int>(tree_depth));

        // get dimensions of octree
        Dtype min_x, min_y, min_z, max_x, max_y, max_z;
        tree->GetMetricMinMax(min_x, min_y, min_z, max_x, max_y, max_z);

        // reset rviz pointcloud classes
        for (uint32_t i = 0; i < kMaxTreeDepth; ++i) {
            m_point_buf_[i].clear();
            if (i < tree_depth) { m_box_size_[i] = tree->GetNodeSize(i + 1); }  // skip depth 0
        }

        std::size_t point_count = 0;
        auto selected_depth = std::min<uint32_t>(tree_depth, m_tree_depth_property_->getInt());
        const double max_height = std::min<double>(m_max_height_property_->getFloat(), max_z);
        const double min_height = std::max<double>(m_min_height_property_->getFloat(), min_z);
        const int step_size = 1 << (tree_depth - selected_depth);  // for pruning of occluded voxels
        const int render_mode_mask = m_tree_render_mode_property_->getOptionInt();
        for (auto it = tree->GetTreeIterator(selected_depth); it->IsValid(); it->Next()) {
            const auto* node = static_cast<const OccupancyOctreeNode*>(it->GetNode());
            if (node == nullptr) {
                setStatusStd(rviz::StatusProperty::Error, "Message", "Failed to get node.");
                return;
            }
            if (node->HasAnyChild()) { continue; }  // skip inner nodes
            double z = it->GetZ();
            if (z > max_height || z < min_height) { continue; }  // skip out of range voxels
            // render mode: 0b01 for free voxels, 0b10 for occupied voxels
            auto node_render_mode = static_cast<int>(tree->IsNodeOccupied(node)) + 1;
            if (!(node_render_mode & render_mode_mask)) { continue; }  // skip non-rendered voxels
            OctreeKey node_key = it->GetKey();
            bool all_neighbors_found = true;
            const uint32_t node_depth = it->GetDepth();
            int diff_base = (node_depth < tree_depth) ? 1 << (tree_depth - node_depth - 1) : 1;
            int diff[2] = {-((node_depth == tree_depth) ? diff_base : diff_base + 1), diff_base};
            // check if current voxel has neighbors on all sides -> no need to be displayed
            OctreeKey key;
            for (int axis = 0; axis < 3; ++axis) {
                int idx_0 = axis % 3;
                int idx_1 = (axis + 1) % 3;
                int idx_2 = (axis + 2) % 3;
                for (int i = 0; i < 2; ++i) {
                    key[idx_0] = node_key[idx_0] + diff[i];
                    // if rendering is restricted to tree_depth < maximum tree depth
                    // inner nodes with distance `step_size` can already occlude a voxel
                    for (key[idx_1] = node_key[idx_1] + diff[0] + 1;
                         key[idx_1] < node_key[idx_1] + diff[1];
                         node_key[idx_1] += step_size) {
                        for (node_key[idx_2] = node_key[idx_2] + diff[0] + 1;
                             node_key[idx_2] < node_key[idx_2] + diff[1];
                             node_key[idx_2] += step_size) {
                            const auto* neighbor_node = static_cast<const OccupancyOctreeNode*>(
                                tree->SearchNode(node_key, selected_depth));
                            if (!neighbor_node ||
                                !((static_cast<int>(tree->IsNodeOccupied(neighbor_node)) + 1) &
                                  render_mode_mask)) {
                                all_neighbors_found = false;
                                break;
                            }
                        }
                        if (!all_neighbors_found) { break; }
                    }
                }
                if (!all_neighbors_found) { break; }
            }
            if (all_neighbors_found) { continue; }  // skip voxels with all neighbors

            // display voxel if it does not have all the neighbors.
            rviz::PointCloud::Point new_point;
            new_point.position.x = it->GetX();
            new_point.position.y = it->GetY();
            new_point.position.z = z;
            // set voxel color
            switch (static_cast<VoxelColorMode>(m_tree_color_mode_property_->getOptionInt())) {
                case CELL_COLOR: {
                    auto colored_node = dynamic_cast<const ColoredOccupancyOctreeNode*>(node);
                    if (colored_node) {
                        auto& color = colored_node->GetColor();
                        new_point.setColor(
                            static_cast<float>(color[0]) / 255.0f,
                            static_cast<float>(color[1]) / 255.0f,
                            static_cast<float>(color[2]) / 255.0f,
                            m_alpha_property_->getFloat());
                        break;
                    }
                    setStatus(
                        rviz::StatusProperty::Error,
                        "Messages",
                        QString(
                            "Cannot extract color, node is not derived from "
                            "ColoredOccupancyOctreeNode."));
                    [[fallthrough]];
                }
                case Z_AXIS_COLOR: {
                    SetColor(z, min_z, max_z, new_point);
                    break;
                }
                case PROBABLILTY_COLOR: {
                    float probability = node->GetOccupancy();
                    SetColor(probability, 0.0f, 1.0f, new_point);
                    // new_point.setColor((1.0f - cell_probability), cell_probability, 0.0);
                    break;
                }
                default:
                    break;
            }
            // push to point vectors
            m_point_buf_[node_depth - 1].push_back(new_point);  // depth 0 is the root node
            ++point_count;
        }

        if (point_count) {  // swap the new points with the old ones
            std::lock_guard<std::mutex> guard(m_mutex_);
            m_new_points_received_ = true;
            for (size_t i = 0; i < kMaxTreeDepth; ++i) { m_new_points_[i].swap(m_point_buf_[i]); }
            m_is_2d_ = false;
            m_tree_resolution_ = tree->GetResolution();
        }
    }

    void
    OccupancyTreeGridDisplay::IncomingMessageCallback(
        const erl_geometry::OccupancyTreeMsgConstPtr& msg) {
        ++m_messages_received_;
        setStatus(
            rviz::StatusProperty::Ok,
            "Messages",
            QString::number(m_messages_received_) + " occupancy tree messages received");
        setStatusStd(rviz::StatusProperty::Ok, "Type", msg->tree_type.c_str());
        if (msg->dim == 2) {
            if (msg->is_double) {
                IncomingMessageCallbackForQuadtree<double>(msg);
            } else {
                IncomingMessageCallbackForQuadtree<float>(msg);
            }
        } else if (msg->dim == 3) {
            if (msg->is_double) {
                IncomingMessageCallbackForOctree<double>(msg);
            } else {
                IncomingMessageCallbackForOctree<float>(msg);
            }
        } else {
            setStatusStd(
                rviz::StatusProperty::Error,
                "Message",
                fmt::format("Unsupported occupancy tree dimension %d", msg->dim));
        }
    }

    void
    OccupancyTreeGridDisplay::SetColor(
        double z_pos,
        double min_z,
        double max_z,
        rviz::PointCloud::Point& point) {

        // hsv to rgb conversion
        constexpr float s = 1.0;
        constexpr float v = 1.0;
        auto h = static_cast<float>(
                     1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) *
                 m_color_factor_;
        h -= floor(h);
        h *= 6;
        auto i = static_cast<int>(floor(h));
        float f = h - static_cast<float>(i);
        if (!(i & 1)) { f = 1 - f; }  // if i is even
        float m = v * (1 - s);        // is 0
        float n = v * (1 - s * f);
        float alpha = m_alpha_property_->getFloat();
        switch (i) {
            case 6:
            case 0:  // red is max, suppose blue is 0
                point.setColor(v, n, m, alpha);
                break;
            case 1:
                point.setColor(n, v, m, alpha);
                break;
            case 2:  // green is max, suppose red is 0
                point.setColor(m, v, n, alpha);
                break;
            case 3:
                point.setColor(m, n, v, alpha);
                break;
            case 4:  // blue is max, suppose green is 0
                point.setColor(n, m, v, alpha);
                break;
            case 5:
                point.setColor(v, m, n, alpha);
                break;
            default:
                point.setColor(1, 0.5, 0.5, alpha);
                break;
        }
    }

    void
    OccupancyTreeGridDisplay::Clear() {
        std::lock_guard<std::mutex> guard(m_mutex_);
        for (auto& cloud: m_clouds_) { cloud->clear(); }  // reset rviz pointcloud boxes
    }

    bool
    OccupancyTreeGridDisplay::UpdateFromTf() {
        // get tf transform
        Ogre::Vector3 pos;
        Ogre::Quaternion orient;
        if (!context_->getFrameManager()->getTransform(m_header_, pos, orient)) { return false; }

        scene_node_->setOrientation(orient);
        scene_node_->setPosition(pos);
        return true;
    }
}  // namespace erl::geometry::rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(erl::geometry::rviz_plugin::OccupancyTreeGridDisplay, rviz::Display)
