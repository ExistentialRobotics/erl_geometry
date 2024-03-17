#pragma once

#include <functional>
#include "erl_common/yaml.hpp"
#include "abstract_octree_drawer.hpp"

namespace erl::geometry {

    template<typename OccupancyOctreeType>
    class OccupancyOctreeDrawer : public AbstractOctreeDrawer {
    public:
        struct Setting : public common::OverrideYamlable<AbstractOctreeDrawer::Setting, Setting> {
            bool occupied_only = false;
            Eigen::Vector3d occupied_color = {0.5, 0.5, 0.5};  // gray
        };

        typedef std::function<void(
            const OccupancyOctreeDrawer *,                               // this
            std::vector<std::shared_ptr<open3d::geometry::Geometry>> &,  // geometries
            typename OccupancyOctreeType::TreeInAabbIterator &)>
            DrawTreeCallback;
        typedef std::function<void(
            const OccupancyOctreeDrawer *,                               // this
            std::vector<std::shared_ptr<open3d::geometry::Geometry>> &,  // geometries
            typename OccupancyOctreeType::LeafInAabbIterator &)>
            DrawLeafCallback;

    private:
        std::shared_ptr<Setting> m_setting_ = {};
        std::shared_ptr<const OccupancyOctreeType> m_occupancy_octree_ = nullptr;
        DrawTreeCallback m_draw_tree_ = {};
        DrawLeafCallback m_draw_leaf_ = {};

    public:
        explicit OccupancyOctreeDrawer(std::shared_ptr<Setting> setting, std::shared_ptr<const OccupancyOctreeType> octree = nullptr)
            : AbstractOctreeDrawer(std::static_pointer_cast<AbstractOctreeDrawer::Setting>(setting), octree),
              m_setting_(std::move(setting)),
              m_occupancy_octree_(std::move(octree)) {
            ERL_ASSERTM(m_setting_, "setting is nullptr.");
        }

        using AbstractOctreeDrawer::DrawLeaves;
        using AbstractOctreeDrawer::DrawTree;

        void
        SetOctree(std::shared_ptr<const AbstractOctree> octree) override {
            m_octree_ = std::move(octree);
            m_occupancy_octree_ = std::dynamic_pointer_cast<const OccupancyOctreeType>(m_octree_);
            ERL_ASSERTM(m_occupancy_octree_, "octree is not an occupancy octree.");
        }

        void
        SetDrawTreeCallback(DrawTreeCallback draw_tree) {
            m_draw_tree_ = std::move(draw_tree);
        }

        void
        SetDrawLeafCallback(DrawLeafCallback draw_leaf) {
            m_draw_leaf_ = std::move(draw_leaf);
        }

        void
        DrawTree(std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const override {
            std::shared_ptr<open3d::geometry::TriangleMesh> boxes;
            std::shared_ptr<open3d::geometry::LineSet> node_border;
            if (geometries.empty()) {
                boxes = std::make_shared<open3d::geometry::TriangleMesh>();
                node_border = std::make_shared<open3d::geometry::LineSet>();
                geometries.push_back(boxes);
                geometries.push_back(node_border);
            } else {
                ERL_ASSERTM(geometries.size() >= 2, "geometries should be empty or contain at least 2 elements: triangle mesh and line set.");
                boxes = std::dynamic_pointer_cast<open3d::geometry::TriangleMesh>(geometries[0]);
                ERL_ASSERTM(boxes, "the first element of geometries should be a triangle mesh.");
                node_border = std::dynamic_pointer_cast<open3d::geometry::LineSet>(geometries[1]);
                ERL_ASSERTM(node_border, "the second element of geometries should be a line set.");
            }

            if (m_occupancy_octree_ == nullptr) {
                ERL_WARN("no occupancy octree is set.");
                return;
            }

            // draw
            boxes->Clear();
            node_border->Clear();
            auto it = m_occupancy_octree_->BeginTreeInAabb(
                m_setting_->area_min[0],
                m_setting_->area_min[1],
                m_setting_->area_min[2],
                m_setting_->area_max[0],
                m_setting_->area_max[1],
                m_setting_->area_max[2]);
            auto end = m_occupancy_octree_->EndTreeInAabb();

            node_border->Clear();
            double area_size = (m_setting_->area_max - m_setting_->area_min).maxCoeff();
            for (; it != end; ++it) {
                double node_size = it.GetNodeSize();
                if (node_size > area_size) { continue; }  // skip nodes that are too large
                double half_size = node_size / 2.0;
                double x = it.GetX();
                double y = it.GetY();
                double z = it.GetZ();
                bool occupied = m_occupancy_octree_->IsNodeOccupied(*it);

                if (!it->HasAnyChild() && occupied) {                                                       // occupied leaf node
                    auto box = open3d::geometry::TriangleMesh::CreateBox(node_size, node_size, node_size);  // min is (0, 0, 0)
                    box->Translate(Eigen::Vector3d(x - half_size, y - half_size, z - half_size));           // move to (x, y, z)
                    *boxes += *box;
                }

                if (m_setting_->occupied_only) {
                    if (m_draw_tree_) { m_draw_tree_(this, geometries, it); }
                    continue;
                }

                auto n = int(node_border->points_.size());
                node_border->points_.emplace_back(x - half_size, y - half_size, z - half_size);
                node_border->points_.emplace_back(x + half_size, y - half_size, z - half_size);
                node_border->points_.emplace_back(x + half_size, y + half_size, z - half_size);
                node_border->points_.emplace_back(x - half_size, y + half_size, z - half_size);
                node_border->points_.emplace_back(x - half_size, y - half_size, z + half_size);
                node_border->points_.emplace_back(x + half_size, y - half_size, z + half_size);
                node_border->points_.emplace_back(x + half_size, y + half_size, z + half_size);
                node_border->points_.emplace_back(x - half_size, y + half_size, z + half_size);
                node_border->lines_.emplace_back(n + 0, n + 1);
                node_border->lines_.emplace_back(n + 1, n + 2);
                node_border->lines_.emplace_back(n + 2, n + 3);
                node_border->lines_.emplace_back(n + 3, n + 0);
                node_border->lines_.emplace_back(n + 4, n + 5);
                node_border->lines_.emplace_back(n + 5, n + 6);
                node_border->lines_.emplace_back(n + 6, n + 7);
                node_border->lines_.emplace_back(n + 7, n + 4);
                node_border->lines_.emplace_back(n + 0, n + 4);
                node_border->lines_.emplace_back(n + 1, n + 5);
                node_border->lines_.emplace_back(n + 2, n + 6);
                node_border->lines_.emplace_back(n + 3, n + 7);

                if (m_draw_tree_) { m_draw_tree_(this, geometries, it); }
            }
            boxes->PaintUniformColor(m_setting_->occupied_color);
            node_border->PaintUniformColor(m_setting_->border_color);
        }

        void
        DrawLeaves(std::vector<std::shared_ptr<open3d::geometry::Geometry>> &geometries) const override {
            std::shared_ptr<open3d::geometry::TriangleMesh> boxes;
            std::shared_ptr<open3d::geometry::LineSet> node_border;
            if (geometries.empty()) {
                boxes = std::make_shared<open3d::geometry::TriangleMesh>();
                node_border = std::make_shared<open3d::geometry::LineSet>();
                geometries.push_back(boxes);
                geometries.push_back(node_border);
            } else {
                ERL_ASSERTM(geometries.size() >= 2, "geometries should be empty or contain at least 2 elements: triangle mesh and line set.");
                boxes = std::dynamic_pointer_cast<open3d::geometry::TriangleMesh>(geometries[0]);
                ERL_ASSERTM(boxes, "the first element of geometries should be a triangle mesh.");
                node_border = std::dynamic_pointer_cast<open3d::geometry::LineSet>(geometries[1]);
                ERL_ASSERTM(node_border, "the second element of geometries should be a line set.");
            }

            std::shared_ptr<const OccupancyOctreeType> octree = std::static_pointer_cast<const OccupancyOctreeType>(m_octree_);
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

            boxes->Clear();
            node_border->Clear();
            for (; it != end; ++it) {
                ERL_DEBUG_ASSERT(!it->HasAnyChild(), "the iterator visits an inner node!");

                double node_size = it.GetNodeSize();
                double half_size = node_size / 2.0;
                double x = it.GetX();
                double y = it.GetY();
                double z = it.GetZ();
                bool occupied = octree->IsNodeOccupied(*it);

                if (occupied) {
                    auto box = open3d::geometry::TriangleMesh::CreateBox(node_size, node_size, node_size);  // min is (0, 0, 0)
                    box->Translate(Eigen::Vector3d(x - half_size, y - half_size, z - half_size));           // move to (x, y, z)
                    *boxes += *box;
                }

                if (m_setting_->occupied_only) {
                    if (m_draw_tree_) { m_draw_leaf_(this, geometries, it); }
                    continue;
                }

                auto n = int(node_border->points_.size());
                node_border->points_.emplace_back(x - half_size, y - half_size, z - half_size);
                node_border->points_.emplace_back(x + half_size, y - half_size, z - half_size);
                node_border->points_.emplace_back(x + half_size, y + half_size, z - half_size);
                node_border->points_.emplace_back(x - half_size, y + half_size, z - half_size);
                node_border->points_.emplace_back(x - half_size, y - half_size, z + half_size);
                node_border->points_.emplace_back(x + half_size, y - half_size, z + half_size);
                node_border->points_.emplace_back(x + half_size, y + half_size, z + half_size);
                node_border->points_.emplace_back(x - half_size, y + half_size, z + half_size);
                node_border->lines_.emplace_back(n + 0, n + 1);
                node_border->lines_.emplace_back(n + 1, n + 2);
                node_border->lines_.emplace_back(n + 2, n + 3);
                node_border->lines_.emplace_back(n + 3, n + 0);
                node_border->lines_.emplace_back(n + 4, n + 5);
                node_border->lines_.emplace_back(n + 5, n + 6);
                node_border->lines_.emplace_back(n + 6, n + 7);
                node_border->lines_.emplace_back(n + 7, n + 4);
                node_border->lines_.emplace_back(n + 0, n + 4);
                node_border->lines_.emplace_back(n + 1, n + 5);
                node_border->lines_.emplace_back(n + 2, n + 6);
                node_border->lines_.emplace_back(n + 3, n + 7);
                if (m_draw_leaf_) { m_draw_leaf_(this, geometries, it); }
            }
            boxes->PaintUniformColor(m_setting_->occupied_color);
            node_border->PaintUniformColor(m_setting_->border_color);
        }
    };
}  // namespace erl::geometry

namespace YAML {
    template<typename Setting>
    struct ConvertOccupancyOctreeDrawerSetting {
        inline static Node
        encode(const Setting &rhs) {
            Node node = convert<erl::geometry::AbstractOctreeDrawer::Setting>::encode(rhs);
            node["occupied_only"] = rhs.occupied_only;
            node["occupied_color"] = rhs.occupied_color;
            return node;
        }

        inline static bool
        decode(const Node &node, Setting &rhs) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::geometry::AbstractOctreeDrawer::Setting>::decode(node, rhs)) { return false; }
            rhs.occupied_only = node["occupied_only"].template as<bool>();
            rhs.occupied_color = node["occupied_color"].template as<Eigen::Vector3d>();
            return true;
        }
    };

    template<typename Setting>
    Emitter &
    PrintOccupancyOctreeDrawerSetting(Emitter &out, const Setting &rhs) {
        out << BeginMap;
        out << Key << "area_min" << Value << rhs.area_min;
        out << Key << "area_max" << Value << rhs.area_max;
        out << Key << "border_color" << Value << rhs.border_color;
        out << Key << "occupied_only" << Value << rhs.occupied_only;
        out << Key << "occupied_color" << Value << rhs.occupied_color;
        out << EndMap;
        return out;
    }
}  // namespace YAML
