#pragma once

#include "abstract_octree_node.hpp"
#include "logodd.hpp"

#include "erl_common/template_helper.hpp"

#include <cstdint>

namespace erl::geometry {

    class OccupancyOctreeNode : public AbstractOctreeNode {
    protected:
        float m_log_odds_ = 0;

    public:
        explicit OccupancyOctreeNode(
            const uint32_t depth = 0,
            const int child_index = -1,
            const float log_odds = 0)
            : AbstractOctreeNode(depth, child_index),
              m_log_odds_(log_odds) {}

        OccupancyOctreeNode(const OccupancyOctreeNode &other) = default;
        OccupancyOctreeNode &
        operator=(const OccupancyOctreeNode &other) = default;
        OccupancyOctreeNode(OccupancyOctreeNode &&other) = default;
        OccupancyOctreeNode &
        operator=(OccupancyOctreeNode &&other) = default;

        bool
        operator==(const AbstractOctreeNode &other) const override {
            if (AbstractOctreeNode::operator==(other)) {
                const auto &other_node = reinterpret_cast<const OccupancyOctreeNode &>(other);
                return m_log_odds_ == other_node.m_log_odds_;
            }
            return false;
        }

        [[nodiscard]] AbstractOctreeNode *
        Create(const uint32_t depth, const int child_index) const override {
            CheckRuntimeType<OccupancyOctreeNode>(this, /*debug_only*/ true);
            const auto node = new OccupancyOctreeNode(depth, child_index, /*log_odds*/ 0);
            return node;
        }

        [[nodiscard]] AbstractOctreeNode *
        Clone() const override {
            CheckRuntimeType<OccupancyOctreeNode>(this, /*debug_only*/ true);
            const auto node = new OccupancyOctreeNode(*this);
            return node;
        }

        //-- file IO
        std::istream &
        ReadData(std::istream &s) override {
            s.read(reinterpret_cast<char *>(&m_log_odds_), sizeof(float));
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            s.write(reinterpret_cast<const char *>(&m_log_odds_), sizeof(float));
            return s;
        }

        //-- pruning and expanding

        [[nodiscard]] bool
        AllowMerge(const AbstractOctreeNode *other) const override {
            ERL_DEBUG_ASSERT(other != nullptr, "other node is nullptr.");
            ERL_DEBUG_ASSERT(
                dynamic_cast<const OccupancyOctreeNode *>(other) != nullptr,
                "other node is not OccupancyOctreeNode.");
            const auto *other_node = reinterpret_cast<const OccupancyOctreeNode *>(other);
            if (m_num_children_ > 0 || other_node->m_num_children_ > 0) { return false; }
            return m_log_odds_ == other_node->m_log_odds_;
        }

        void
        Prune() override {
            m_log_odds_ = reinterpret_cast<OccupancyOctreeNode *>(m_children_[0])->m_log_odds_;
            AbstractOctreeNode::Prune();
        }

        void
        Expand() override {
            if (m_children_ == nullptr) { m_children_ = new AbstractOctreeNode *[8]; }
            for (int i = 0; i < 8; ++i) {
                // make sure the child type is correct if this class is inherited
                AbstractOctreeNode *child = this->Create(m_depth_ + 1, i);
                m_children_[i] = child;
                reinterpret_cast<OccupancyOctreeNode *>(child)->m_log_odds_ = m_log_odds_;
            }
            m_num_children_ = 8;
        }

        //-- node occupancy
        [[nodiscard]] float
        GetOccupancy() const {
            return logodd::Probability(m_log_odds_);
        }

        [[nodiscard]] const float &
        GetLogOdds() const {
            return m_log_odds_;
        }

        void
        SetLogOdds(const float log_odds) {
            m_log_odds_ = log_odds;
        }

        virtual bool
        AllowUpdateLogOdds(float &delta) const {
            (void) delta;
            return true;
        }

        [[maybe_unused]] [[nodiscard]] float
        GetMeanChildLogOdds() const {
            if (!HasAnyChild()) { return -std::numeric_limits<float>::infinity(); }  // log(0)

            float mean = 0;
            for (int i = 0; i < 8; ++i) {
                const auto *child = reinterpret_cast<OccupancyOctreeNode *>(m_children_[i]);
                if (child == nullptr) { continue; }
                mean += child->GetOccupancy();
            }
            mean /= static_cast<float>(m_num_children_);

            return logodd::LogOdd(mean);
        }

        [[nodiscard]] float
        GetMaxChildLogOdds() const {
            float max = -std::numeric_limits<float>::max();

            if (m_num_children_ > 0) {
                for (int i = 0; i < 8; ++i) {
                    const auto *child = reinterpret_cast<OccupancyOctreeNode *>(
                        m_children_[i]);  // dynamic_cast causes high overhead
                    if (child == nullptr) { continue; }
                    const float l = child->GetLogOdds();
                    if (l > max) { max = l; }
                }
            }
            return max;
        }

        void
        AddLogOdds(const float log_odds) {
            m_log_odds_ += log_odds;
        }
    };
}  // namespace erl::geometry
