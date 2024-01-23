#pragma once

#include <functional>
#include "erl_common/yaml.hpp"
#include "abstract_octree_drawer.hpp"

namespace erl::geometry {

    template<typename OccupancyOctreeType>
    class OccupancyOctreeDrawer : public AbstractOctreeDrawer {
    public:
        struct Setting : public common::OverrideYamlable<AbstractOctreeDrawer::Setting, Setting> {
            Eigen::Vector3d occupied_color = {0.5, 0.5, 0.5};  // gray
            Eigen::Vector3d free_color = {1.0, 1.0, 1.0};      // white
        };

        typedef std::function<void(
            const OccupancyOctreeDrawer *,        // this
            open3d::visualization::Visualizer *,  // visualizer
            typename OccupancyOctreeType::TreeInAabbIterator &)>
            DrawTreeCallback;
        typedef std::function<void(
            const OccupancyOctreeDrawer *,        // this
            open3d::visualization::Visualizer *,  // visualizer
            typename OccupancyOctreeType::LeafInAabbIterator &)>
            DrawLeafCallback;

    private:
        std::shared_ptr<Setting> m_setting_ = {};
        DrawTreeCallback m_draw_tree_ = {};
        DrawLeafCallback m_draw_leaf_ = {};

    public:
        explicit OccupancyOctreeDrawer(std::shared_ptr<Setting> setting, std::shared_ptr<const OccupancyOctreeType> octree = nullptr)
            : AbstractOctreeDrawer(std::static_pointer_cast<AbstractOctreeDrawer::Setting>(setting), std::move(octree)),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_, "setting is nullptr.");
        }

        using AbstractOctreeDrawer::DrawLeaves;
        using AbstractOctreeDrawer::DrawTree;

        void
        SetDrawTreeCallback(DrawTreeCallback draw_tree) {
            m_draw_tree_ = std::move(draw_tree);
        }

        void
        SetDrawLeafCallback(DrawLeafCallback draw_leaf) {
            m_draw_leaf_ = std::move(draw_leaf);
        }

        void
        DrawTree(open3d::visualization::Visualizer *visualizer) const override {
            if (m_octree_ == nullptr) { return; }
            ERL_ASSERTM(visualizer, "visualizer is nullptr.");

            visualizer->GetRenderOption().background_color_ = m_setting_->bg_color;
            std::shared_ptr<const OccupancyOctreeType> octree = std::dynamic_pointer_cast<const OccupancyOctreeType>(m_octree_);
            if (octree == nullptr) {
                ERL_WARN("octree is not an occupancy octree.");
                return;
            }

            auto it = octree->BeginTreeInAabb(
                m_setting_->area_min[0],
                m_setting_->area_min[1],
                m_setting_->area_min[2],
                m_setting_->area_max[0],
                m_setting_->area_max[1],
                m_setting_->area_max[2]);
            auto end = octree->EndTreeInAabb();
            auto boxes = std::make_shared<open3d::geometry::TriangleMesh>();
            auto node_border = std::make_shared<open3d::geometry::LineSet>();
            double area_size = (m_setting_->area_max - m_setting_->area_min).maxCoeff();
            for (; it != end; ++it) {
                double node_size = it.GetNodeSize();
                if (node_size > area_size) { continue; }  // skip nodes that are too large
                double half_size = node_size / 2.0;
                double x = it.GetX();
                double y = it.GetY();
                double z = it.GetZ();

                if (!it->HasAnyChild() && octree->IsNodeOccupied(*it)) {                                    // occupied leaf node
                    auto box = open3d::geometry::TriangleMesh::CreateBox(node_size, node_size, node_size);  // min is (0, 0, 0)
                    box->Translate(Eigen::Vector3d(x - half_size, y - half_size, z - half_size));           // move to (x, y, z)
                    *boxes += *box;
                }
                auto n = int(node_border->points_.size());
                node_border->points_.push_back(Eigen::Vector3d(x - half_size, y - half_size, z - half_size));
                node_border->points_.push_back(Eigen::Vector3d(x + half_size, y - half_size, z - half_size));
                node_border->points_.push_back(Eigen::Vector3d(x + half_size, y + half_size, z - half_size));
                node_border->points_.push_back(Eigen::Vector3d(x - half_size, y + half_size, z - half_size));
                node_border->points_.push_back(Eigen::Vector3d(x - half_size, y - half_size, z + half_size));
                node_border->points_.push_back(Eigen::Vector3d(x + half_size, y - half_size, z + half_size));
                node_border->points_.push_back(Eigen::Vector3d(x + half_size, y + half_size, z + half_size));
                node_border->points_.push_back(Eigen::Vector3d(x - half_size, y + half_size, z + half_size));
                node_border->lines_.push_back({n + 0, n + 1});
                node_border->lines_.push_back({n + 1, n + 2});
                node_border->lines_.push_back({n + 2, n + 3});
                node_border->lines_.push_back({n + 3, n + 0});
                node_border->lines_.push_back({n + 4, n + 5});
                node_border->lines_.push_back({n + 5, n + 6});
                node_border->lines_.push_back({n + 6, n + 7});
                node_border->lines_.push_back({n + 7, n + 4});
                node_border->lines_.push_back({n + 0, n + 4});
                node_border->lines_.push_back({n + 1, n + 5});
                node_border->lines_.push_back({n + 2, n + 6});
                node_border->lines_.push_back({n + 3, n + 7});

                if (m_draw_tree_) { m_draw_tree_(this, visualizer, it); }
            }
            boxes->PaintUniformColor(m_setting_->occupied_color);
            node_border->PaintUniformColor(m_setting_->border_color);
            visualizer->AddGeometry(boxes);
            visualizer->AddGeometry(node_border);
        }

        void
        DrawLeaves(open3d::visualization::Visualizer *visualizer) const override {
            if (m_octree_ == nullptr) { return; }
            ERL_ASSERTM(visualizer, "visualizer is nullptr.");

            visualizer->GetRenderOption().background_color_ = m_setting_->bg_color;
            std::shared_ptr<const OccupancyOctreeType> octree = std::dynamic_pointer_cast<const OccupancyOctreeType>(m_octree_);
            if (octree == nullptr) {
                ERL_WARN("octree is not an occupancy octree.");
                return;
            }

            auto it = octree->BeginLeafInAabb(
                m_setting_->area_min[0],
                m_setting_->area_min[1],
                m_setting_->area_min[2],
                m_setting_->area_max[0],
                m_setting_->area_max[1],
                m_setting_->area_max[2]);
            auto end = octree->EndLeafInAabb();
            auto boxes = std::make_shared<open3d::geometry::TriangleMesh>();
            auto node_border = std::make_shared<open3d::geometry::LineSet>();
            for (; it != end; ++it) {
                ERL_DEBUG_ASSERT(!it->HasAnyChild(), "the iterator visits an inner node!");

                double node_size = it.GetNodeSize();
                double half_size = node_size / 2.0;
                double x = it.GetX();
                double y = it.GetY();
                double z = it.GetZ();

                if (octree->IsNodeOccupied(*it)) {
                    auto box = open3d::geometry::TriangleMesh::CreateBox(node_size, node_size, node_size);  // min is (0, 0, 0)
                    box->Translate(Eigen::Vector3d(x - half_size, y - half_size, z - half_size));           // move to (x, y, z)
                    *boxes += *box;
                }

                auto n = int(node_border->points_.size());
                node_border->points_.push_back(Eigen::Vector3d(x - half_size, y - half_size, z - half_size));
                node_border->points_.push_back(Eigen::Vector3d(x + half_size, y - half_size, z - half_size));
                node_border->points_.push_back(Eigen::Vector3d(x + half_size, y + half_size, z - half_size));
                node_border->points_.push_back(Eigen::Vector3d(x - half_size, y + half_size, z - half_size));
                node_border->points_.push_back(Eigen::Vector3d(x - half_size, y - half_size, z + half_size));
                node_border->points_.push_back(Eigen::Vector3d(x + half_size, y - half_size, z + half_size));
                node_border->points_.push_back(Eigen::Vector3d(x + half_size, y + half_size, z + half_size));
                node_border->points_.push_back(Eigen::Vector3d(x - half_size, y + half_size, z + half_size));
                node_border->lines_.push_back({n + 0, n + 1});
                node_border->lines_.push_back({n + 1, n + 2});
                node_border->lines_.push_back({n + 2, n + 3});
                node_border->lines_.push_back({n + 3, n + 0});
                node_border->lines_.push_back({n + 4, n + 5});
                node_border->lines_.push_back({n + 5, n + 6});
                node_border->lines_.push_back({n + 6, n + 7});
                node_border->lines_.push_back({n + 7, n + 4});
                node_border->lines_.push_back({n + 0, n + 4});
                node_border->lines_.push_back({n + 1, n + 5});
                node_border->lines_.push_back({n + 2, n + 6});
                node_border->lines_.push_back({n + 3, n + 7});
                if (m_draw_leaf_) { m_draw_leaf_(this, visualizer, it); }
            }
            boxes->PaintUniformColor(m_setting_->occupied_color);
            node_border->PaintUniformColor(m_setting_->border_color);
            visualizer->AddGeometry(boxes);
            visualizer->AddGeometry(node_border);
        }
    };
}  // namespace erl::geometry

namespace YAML {
    template<typename Setting>
    struct ConvertOccupancyOctreeDrawerSetting {
        inline static Node
        encode(const Setting &rhs) {
            Node node = convert<erl::geometry::AbstractOctreeDrawer::Setting>::encode(rhs);
            node["occupied_color"] = rhs.occupied_color;
            node["free_color"] = rhs.free_color;
            return node;
        }

        inline static bool
        decode(const Node &node, Setting &rhs) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::geometry::AbstractOctreeDrawer::Setting>::decode(node, rhs)) { return false; }
            rhs.occupied_color = node["occupied_color"].template as<Eigen::Vector3d>();
            rhs.free_color = node["free_color"].template as<Eigen::Vector3d>();
            return true;
        }
    };

    template<typename Setting>
    Emitter &
    PrintOccupancyOctreeDrawerSetting(Emitter &out, const Setting &rhs) {
        out << BeginMap;
        out << Key << "bg_color" << Value << rhs.bg_color;
        out << Key << "border_color" << Value << rhs.border_color;
        out << Key << "occupied_color" << Value << rhs.occupied_color;
        out << Key << "free_color" << Value << rhs.free_color;
        out << EndMap;
        return out;
    }
}  // namespace YAML
