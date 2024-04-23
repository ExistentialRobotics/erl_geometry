#pragma once

#include "logodd.hpp"
#include "abstract_octree_node.hpp"

#include <cstdint>

namespace erl::geometry {

    class OccupancyOctreeNode : public AbstractOctreeNode {
    protected:
        float m_log_odds_ = 0;

    public:
        explicit OccupancyOctreeNode(uint32_t depth = 0, int child_index = -1, float log_odds = 0)
            : AbstractOctreeNode(depth, child_index),
              m_log_odds_(log_odds) {}

        OccupancyOctreeNode(const OccupancyOctreeNode &other) = default;
        OccupancyOctreeNode &
        operator=(const OccupancyOctreeNode &other) = default;
        OccupancyOctreeNode(OccupancyOctreeNode &&other) = default;
        OccupancyOctreeNode &
        operator=(OccupancyOctreeNode &&other) = default;

        [[nodiscard]] inline AbstractOctreeNode *
        Clone() const override {
            return new OccupancyOctreeNode(*this);
        }

        inline bool
        operator==(const AbstractOctreeNode &other) const override {
            if (AbstractOctreeNode::operator==(other)) {
                const auto &other_node = reinterpret_cast<const OccupancyOctreeNode &>(other);
                return m_log_odds_ == other_node.m_log_odds_;
            }
            return false;
        }

        //-- file IO
        inline std::istream &
        ReadData(std::istream &s) override {
            s.read(reinterpret_cast<char *>(&m_log_odds_), sizeof(float));
            return s;
        }

        inline std::ostream &
        WriteData(std::ostream &s) const override {
            s.write(reinterpret_cast<const char *>(&m_log_odds_), sizeof(float));
            return s;
        }

        [[nodiscard]] inline bool
        AllowMerge(const AbstractOctreeNode *other) const override {
            ERL_DEBUG_ASSERT(other != nullptr, "other node is nullptr.");
            const auto *other_node = reinterpret_cast<const OccupancyOctreeNode *>(other);
            if (m_num_children_ > 0 || other_node->m_num_children_ > 0) { return false; }
            return m_log_odds_ == other_node->m_log_odds_;
        }

        inline void
        Prune() override {
            m_log_odds_ = reinterpret_cast<OccupancyOctreeNode *>(m_children_[0])->m_log_odds_;
            AbstractOctreeNode::Prune();
        }

        inline void
        Expand() override {
            if (m_children_ == nullptr) { m_children_ = new AbstractOctreeNode *[8]; }
            for (int i = 0; i < 8; ++i) {
                auto child = this->AllocateChildPtr(i);  // make sure child type is correct if this class is inherited
                m_children_[i] = child;
                reinterpret_cast<OccupancyOctreeNode *>(child)->m_log_odds_ = m_log_odds_;
            }
            m_num_children_ = 8;
        }

        //-- node occupancy
        [[nodiscard]] inline double
        GetOccupancy() const {
            return logodd::Probability(m_log_odds_);
        }

        [[nodiscard]] inline const float &
        GetLogOdds() const {
            return m_log_odds_;
        }

        inline void
        SetLogOdds(float log_odds) {
            m_log_odds_ = log_odds;
        }

        virtual inline bool
        AllowUpdateLogOdds(double &delta) const {
            (void) delta;
            return true;
        }

        [[maybe_unused]] [[nodiscard]] inline double
        GetMeanChildLogOdds() const {
            if (!HasAnyChild()) { return -std::numeric_limits<double>::infinity(); }  // log(0)

            double mean = 0;
            for (int i = 0; i < 8; ++i) {
                const auto *child = reinterpret_cast<OccupancyOctreeNode *>(m_children_[i]);
                if (child == nullptr) { continue; }
                mean += child->GetOccupancy();
            }
            mean /= double(m_num_children_);

            return logodd::LogOdd(mean);
        }

        [[nodiscard]] inline float
        GetMaxChildLogOdds() const {
            float max = -std::numeric_limits<float>::max();

            if (m_num_children_ > 0) {
                for (int i = 0; i < 8; ++i) {
                    const auto *child = reinterpret_cast<OccupancyOctreeNode *>(m_children_[i]);  // dynamic_cast causes high overhead
                    if (child == nullptr) { continue; }
                    float l = child->GetLogOdds();
                    if (l > max) { max = l; }
                }
            }
            return max;
        }

        inline void
        AddLogOdds(float log_odds) {
            m_log_odds_ += log_odds;
        }

    private:
        inline AbstractOctreeNode *
        AllocateChildPtr(uint32_t child_index) override {
            return new OccupancyOctreeNode(m_depth_ + 1, int(child_index), 0);
        }
    };
}  // namespace erl::geometry
