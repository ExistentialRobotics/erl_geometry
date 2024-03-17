#pragma once

#include "erl_common/assert.hpp"
#include "abstract_quadtree.hpp"
#include "quadtree_key.hpp"
#include "occupancy_quadtree_node.hpp"
#include "logodd.hpp"

namespace erl::geometry {

    /**
     * AbstractOccupancyQuadtree is a base class that implements generic occupancy quadtree functionality.
     */
    class AbstractOccupancyQuadtree : public AbstractQuadtree {

    protected:
        // occupancy parameters, stored in log-odds
        float m_log_odd_min_ = -2;           // minimum log-odd value, =0.12 in probability
        float m_log_odd_max_ = 3.5;          // maximum log-odd value, = 0.97 in probability
        float m_log_odd_hit_ = 0.85;         // log-odd value to add when a cell is hit by a ray, =0.7 in probability
        float m_log_odd_miss_ = -0.4;        // log-odd value to add when a cell is gone through by a ray, =0.4 in probability
        float m_log_odd_occ_threshold_ = 0;  // threshold that is used to decide whether a cell is occupied or not, =0.5 in probability

        // binary file header identifier
        inline static const std::string sk_BinaryFileHeader_ = "# OccupancyQuadtree binary file";

    public:
        AbstractOccupancyQuadtree() = default;

        //--IO
        /**
         * Write the tree to a binary file. Before writing, the tree is pruned.
         * @param filename
         * @return
         */
        bool
        WriteBinary(const std::string& filename);
        /**
         * Write the tree to a binary stream. Before writing, the tree is pruned.
         * @param s
         * @return
         */
        bool
        WriteBinary(std::ostream& s);
        /**
         * Write the tree to a binary file. The tree is not pruned before writing.
         * @param filename
         * @return
         */
        [[nodiscard]] bool
        WriteBinary(const std::string& filename) const;
        /**
         * Write the tree to a binary stream. The tree is not pruned before writing.
         * @param s
         * @return
         */
        bool
        WriteBinary(std::ostream& s) const;

        /**
         * Write the actual tree data to a binary stream.
         * @param s
         * @return
         */
        virtual std::ostream&
        WriteBinaryData(std::ostream& s) const = 0;

        bool
        ReadBinary(const std::string& filename);

        bool
        ReadBinary(std::istream& s);

        virtual std::istream&
        ReadBinaryData(std::istream& s) = 0;

        //-- occupancy queries
        [[nodiscard]] inline bool
        IsNodeOccupied(const OccupancyQuadtreeNode* node) const {
            return node->GetLogOdds() >= m_log_odd_occ_threshold_;
        }

        [[nodiscard]] inline bool
        IsNodeAtThreshold(const OccupancyQuadtreeNode* node) const {
            float log_odds = node->GetLogOdds();
            return log_odds >= m_log_odd_max_ || log_odds <= m_log_odd_min_;
        }

        //-- update functions
        virtual void
        ToMaxLikelihood() = 0;

        //-- parameters for occupancy and sensor model
        inline void
        SetLogOddMin(float val) {
            m_log_odd_min_ = val;
        }

        [[nodiscard]] inline float
        GetLogOddMin() const {
            return m_log_odd_min_;
        }

        inline void
        SetLogOddMax(float val) {
            m_log_odd_max_ = val;
        }

        [[nodiscard]] inline float
        GetLogOddMax() const {
            return m_log_odd_max_;
        }

        inline void
        SetProbabilityHit(double p) {
            m_log_odd_hit_ = logodd::LogOdd(p);
            ERL_ASSERTM(m_log_odd_hit_ > 0, "ProbabilityHit must be > 0, but is %f", m_log_odd_hit_);
        }

        [[nodiscard]] inline double
        GetProbabilityHit() const {
            return logodd::Probability(m_log_odd_hit_);
        }

        inline void
        SetProbabilityMiss(double p) {
            m_log_odd_miss_ = logodd::LogOdd(p);
            ERL_ASSERTM(m_log_odd_miss_ < 0, "ProbabilityMiss must be < 0, but is %f", m_log_odd_miss_);
        }

        [[nodiscard]] inline double
        GetProbabilityMiss() const {
            return logodd::Probability(m_log_odd_miss_);
        }

        inline void
        SetOccupancyThreshold(double p) {
            m_log_odd_occ_threshold_ = logodd::LogOdd(p);
        }

        [[nodiscard]] inline double
        GetOccupancyThreshold() const {
            return logodd::Probability(m_log_odd_occ_threshold_);
        }
    };
}  // namespace erl::geometry
