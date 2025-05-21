#pragma once

#include "occupancy_quadtree_node.hpp"

namespace erl::geometry {
    class ColoredOccupancyQuadtreeNode : public OccupancyQuadtreeNode {
    protected:
        std::array<uint8_t, 4> m_color_ = {0, 0, 0, 0};  // RGBA color

    public:
        explicit ColoredOccupancyQuadtreeNode(
            const uint32_t depth = 0,
            const int child_index = -1,
            const float log_odds = 0)
            : OccupancyQuadtreeNode(depth, child_index, log_odds) {}

        ColoredOccupancyQuadtreeNode(const ColoredOccupancyQuadtreeNode &other) = default;
        ColoredOccupancyQuadtreeNode &
        operator=(const ColoredOccupancyQuadtreeNode &other) = default;
        ColoredOccupancyQuadtreeNode(ColoredOccupancyQuadtreeNode &&other) = default;
        ColoredOccupancyQuadtreeNode &
        operator=(ColoredOccupancyQuadtreeNode &&other) = default;

        bool
        operator==(const AbstractQuadtreeNode &other) const override {
            if (OccupancyQuadtreeNode::operator==(other)) {
                auto &other_node = reinterpret_cast<const ColoredOccupancyQuadtreeNode &>(other);
                return m_color_ == other_node.m_color_;
            }
            return false;
        }

        [[nodiscard]] AbstractQuadtreeNode *
        Create(const uint32_t depth, const int child_index) const override {
            CheckRuntimeType<ColoredOccupancyQuadtreeNode>(this, /*debug_only*/ true);
            const auto node = new ColoredOccupancyQuadtreeNode(depth, child_index, /*log_odds*/ 0);
            return node;
        }

        [[nodiscard]] AbstractQuadtreeNode *
        Clone() const override {
            CheckRuntimeType<ColoredOccupancyQuadtreeNode>(this, /*debug_only*/ true);
            const auto node = new ColoredOccupancyQuadtreeNode(*this);
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
            OccupancyQuadtreeNode::ReadData(s);
            s.read(reinterpret_cast<char *>(m_color_.data()), m_color_.size());
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            OccupancyQuadtreeNode::WriteData(s);
            s.write(reinterpret_cast<const char *>(m_color_.data()), m_color_.size());
            return s;
        }

        //-- pruning and expanding

        [[nodiscard]] bool
        AllowMerge(const AbstractQuadtreeNode *other) const override {
            if (!OccupancyQuadtreeNode::AllowMerge(other)) { return false; }
            ERL_DEBUG_ASSERT(
                dynamic_cast<const ColoredOccupancyQuadtreeNode *>(other) != nullptr,
                "other node is not ColoredOccupancyQuadtreeNode.");
            const auto *other_node = reinterpret_cast<const ColoredOccupancyQuadtreeNode *>(other);
            return m_color_ == other_node->m_color_;
        }

        void
        Prune() override {
            m_color_ = reinterpret_cast<ColoredOccupancyQuadtreeNode *>(m_children_[0])->m_color_;
            OccupancyQuadtreeNode::Prune();
        }

        void
        Expand() override {
            if (m_children_ == nullptr) { m_children_ = new AbstractQuadtreeNode *[4]; }
            for (int i = 0; i < 4; ++i) {
                // call the virtual method `Create` to make sure the child type is correct if this
                // class is inherited
                AbstractQuadtreeNode *child = this->Create(m_depth_ + 1, i);
                m_children_[i] = child;
                auto *colored_child = reinterpret_cast<ColoredOccupancyQuadtreeNode *>(child);
                colored_child->m_log_odds_ = m_log_odds_;
                colored_child->m_color_ = m_color_;
            }
            m_num_children_ = 4;
        }
    };
}  // namespace erl::geometry
