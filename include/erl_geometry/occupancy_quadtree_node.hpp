#pragma once

#include <cstdint>
#include "quadtree_data_node.hpp"
#include "logodd.hpp"

namespace erl::geometry {

    class OccupancyQuadtreeNode : public QuadtreeDataNode<float> {
    public:
        explicit OccupancyQuadtreeNode(float log_odds = 0)
            : QuadtreeDataNode<float>(log_odds) {}

        OccupancyQuadtreeNode(const OccupancyQuadtreeNode &other)
            : QuadtreeDataNode<float>(other.m_value_) {
            CopyChildren<OccupancyQuadtreeNode>(other);
        }

        //-- node occupancy
        [[nodiscard]] inline double
        GetOccupancy() const {
            return logodd::Probability(m_value_);
        }

        [[nodiscard]] inline const float &
        GetLogOdds() const {
            return m_value_;
        }

        inline void
        SetLogOdds(float log_odds) {
            m_value_ = log_odds;
        }

        virtual inline bool
        AllowUpdateLogOdds(double &delta) const {
            (void) delta;
            return true;
        }

        [[nodiscard]] inline double
        GetMeanChildLogOdds() const {
            if (!HasAnyChild()) { return -std::numeric_limits<double>::infinity(); }  // log(0)

            double mean = 0;
            for (int i = 0; i < 4; ++i) {
                auto child = static_cast<OccupancyQuadtreeNode *>(m_children_[i]);
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
                for (int i = 0; i < 4; ++i) {
                    auto child = static_cast<OccupancyQuadtreeNode *>(m_children_[i]);
                    if (child == nullptr) { continue; }
                    float l = static_cast<OccupancyQuadtreeNode *>(child)->GetLogOdds();
                    if (l > max) { max = l; }
                }
            }
            return max;
        }

        inline void
        AddLogOdds(float log_odds) {
            m_value_ += log_odds;
        }
    };
}  // namespace erl::geometry
