#pragma once

#include "quadtree_node.hpp"
#include "logodd.hpp"

namespace erl::geometry {

    class OccupancyQuadtreeNode : public QuadtreeDataNode<float> {
    public:
        OccupancyQuadtreeNode()
            : QuadtreeDataNode<float>(0) {}

        //-- node occupancy
        [[nodiscard]] inline double
        GetOccupancy() const {
            return logodd::Probability(m_value_);
        }

        [[nodiscard]] inline float
        GetLogOdds() const {
            return m_value_;
        }

        inline void
        SetLogOdds(float log_odds) {
            m_value_ = log_odds;
        }

        virtual bool
        AllowUpdateLogOdds(double delta) const {
            (void) delta;
            return true;
        }

        [[nodiscard]] inline double
        ComputeMeanChildLogOdds() const {
            if (!HasAnyChild()) { return -std::numeric_limits<double>::infinity(); }  // log(0)

            double mean = 0;
            uint8_t c = 0;
            for (unsigned int i = 0; i < 4; ++i) {
                auto &child = m_children_[i];
                if (child == nullptr) { continue; }
                mean += std::static_pointer_cast<OccupancyQuadtreeNode>(child)->GetOccupancy();
                ++c;
            }

            mean /= double(c);

            return std::log(mean / (1 - mean));
        }

        [[nodiscard]] inline float
        GetMaxChildLogOdds() const {
            float max = -std::numeric_limits<float>::max();
            if (!HasAnyChild()) { return max; }

            for (unsigned int i = 0; i < 4; ++i) {
                auto &child = m_children_[i];
                if (child == nullptr) { continue; }
                float l = std::static_pointer_cast<OccupancyQuadtreeNode>(child)->GetLogOdds();
                if (l > max) { max = l; }
            }
            return max;
        }

        inline void
        AddLogOdds(float log_odds) {
            m_value_ += log_odds;
        }
    };
}  // namespace erl::geometry
