#pragma once

#include "occupancy_octree_node.hpp"

namespace erl::geometry {

    class ColoredOccupancyOctreeNode : public OccupancyOctreeNode {
    protected:
        std::array<uint8_t, 4> m_color_ = {0, 0, 0, 0};  // RGBA color

    public:
        explicit ColoredOccupancyOctreeNode(
            const uint32_t depth = 0,
            const int child_index = -1,
            const float log_odds = 0)
            : OccupancyOctreeNode(depth, child_index, log_odds) {}

        ColoredOccupancyOctreeNode(const ColoredOccupancyOctreeNode &other) = default;
        ColoredOccupancyOctreeNode &
        operator=(const ColoredOccupancyOctreeNode &other) = default;
        ColoredOccupancyOctreeNode(ColoredOccupancyOctreeNode &&other) = default;
        ColoredOccupancyOctreeNode &
        operator=(ColoredOccupancyOctreeNode &&other) = default;

        bool
        operator==(const AbstractOctreeNode &other) const override {
            if (OccupancyOctreeNode::operator==(other)) {
                auto &other_node = reinterpret_cast<const ColoredOccupancyOctreeNode &>(other);
                return m_color_ == other_node.m_color_;
            }
            return false;
        }

        [[nodiscard]] AbstractOctreeNode *
        Create(const uint32_t depth, const int child_index) const override {
            CheckRuntimeType<ColoredOccupancyOctreeNode>(this, /*debug_only*/ true);
            const auto node = new ColoredOccupancyOctreeNode(depth, child_index, /*log_odds*/ 0);
            return node;
        }

        [[nodiscard]] AbstractOctreeNode *
        Clone() const override {
            CheckRuntimeType<ColoredOccupancyOctreeNode>(this, /*debug_only*/ true);
            const auto node = new ColoredOccupancyOctreeNode(*this);
            return node;
        }

        const std::array<uint8_t, 4> &
        GetColor() const {
            return m_color_;
        }

        void
        SetColor(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a) {
            m_color_[0] = r;
            m_color_[1] = g;
            m_color_[2] = b;
            m_color_[3] = a;
        }

        //-- file IO
        std::istream &
        ReadData(std::istream &s) override {
            OccupancyOctreeNode::ReadData(s);
            s.read(reinterpret_cast<char *>(m_color_.data()), m_color_.size());
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            OccupancyOctreeNode::WriteData(s);
            s.write(reinterpret_cast<const char *>(m_color_.data()), m_color_.size());
            return s;
        }

        //-- pruning and expanding

        [[nodiscard]] bool
        AllowMerge(const AbstractOctreeNode *other) const override {
            if (!OccupancyOctreeNode::AllowMerge(other)) { return false; }
            ERL_DEBUG_ASSERT(
                dynamic_cast<const ColoredOccupancyOctreeNode *>(other) != nullptr,
                "other node is not ColoredOccupancyOctreeNode.");
            const auto *other_node = reinterpret_cast<const ColoredOccupancyOctreeNode *>(other);
            return m_color_ == other_node->m_color_;
        }

        void
        Prune() override {
            m_color_ = reinterpret_cast<ColoredOccupancyOctreeNode *>(m_children_[0])->m_color_;
            OccupancyOctreeNode::Prune();
        }

        void
        Expand() override {
            if (m_children_ == nullptr) { m_children_ = new AbstractOctreeNode *[8]; }
            for (int i = 0; i < 8; ++i) {
                // make sure the child type is correct if this class is inherited
                AbstractOctreeNode *child = this->Create(m_depth_ + 1, i);
                m_children_[i] = child;
                auto *colored_child = reinterpret_cast<ColoredOccupancyOctreeNode *>(child);
                colored_child->m_log_odds_ = m_log_odds_;
                colored_child->m_color_ = m_color_;
            }
            m_num_children_ = 8;
        }
    };

}  // namespace erl::geometry
