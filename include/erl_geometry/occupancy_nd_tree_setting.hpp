#pragma once

#include "logodd.hpp"
#include "nd_tree_setting.hpp"

namespace erl::geometry {
    /**
     * OccupancyNdTreeSetting is a base class for all occupancy n-d tree settings.
     */
    struct OccupancyNdTreeSetting : public common::Yamlable<OccupancyNdTreeSetting, NdTreeSetting> {
        // minimum log-odd value, default: -2 in log odd = 0.12 in probability
        float log_odd_min = -2;
        // maximum log-odd value, default: 3.5 in log odd = 0.97 in probability
        float log_odd_max = 3.5;
        // log-odd value to add when a ray hits a cell, default: 0.85 in log odd = 0.7 in
        // probability
        float log_odd_hit = 0.85;
        // log-odd value to add when a ray goes through a cell, default: -0.4 in log odd = 0.4 in
        // probability
        float log_odd_miss = -0.4;
        // threshold used to decide whether a cell is occupied or not, default: 0 in log odd = 0.5
        // in probability
        float log_odd_occ_threshold = 0;

        ERL_REFLECT_SCHEMA(
            OccupancyNdTreeSetting,
            ERL_REFLECT_MEMBER(OccupancyNdTreeSetting, log_odd_min),
            ERL_REFLECT_MEMBER(OccupancyNdTreeSetting, log_odd_max),
            ERL_REFLECT_MEMBER(OccupancyNdTreeSetting, log_odd_hit),
            ERL_REFLECT_MEMBER(OccupancyNdTreeSetting, log_odd_miss),
            ERL_REFLECT_MEMBER(OccupancyNdTreeSetting, log_odd_occ_threshold));

        void
        SetProbabilityHit(const float p) {
            log_odd_hit = logodd::LogOdd(p);
            ERL_WARN_COND(log_odd_hit <= 0, "ProbabilityHit should be > 0, but is {}", log_odd_hit);
        }

        [[nodiscard]] double
        GetProbabilityHit() const {
            return logodd::Probability(log_odd_hit);
        }

        void
        SetProbabilityMiss(const float p) {
            log_odd_miss = logodd::LogOdd(p);
            ERL_WARN_COND(
                log_odd_miss >= 0,
                "ProbabilityMiss should be < 0, but is {}",
                log_odd_miss);
        }

        [[nodiscard]] double
        GetProbabilityMiss() const {
            return logodd::Probability(log_odd_miss);
        }

        void
        SetProbabilityOccupiedThreshold(const float p) {
            log_odd_occ_threshold = logodd::LogOdd(p);
        }

        [[nodiscard]] float
        GetProbabilityOccupiedThreshold() const {
            return logodd::Probability(log_odd_occ_threshold);
        }

        bool
        operator==(const NdTreeSetting &other) const override;
    };
}  // namespace erl::geometry
