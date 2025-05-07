#pragma once

#include "erl_common/logging.hpp"
#include "erl_common/yaml.hpp"

#include <boost/geometry/index/rtree.hpp>

namespace erl::geometry {

    /**
     * wrapper of boost::geometry::index::rtree. Inspired by
     * https://github.com/mloskot/spatial_index_benchmark
     * @tparam T data type of point
     * @tparam Dim dimension of point
     */
    template<typename T, int Dim, typename SplitModeType = boost::geometry::index::dynamic_rstar>
    class RTree {
    public:
        struct Setting : public common::Yamlable<Setting> {
            std::size_t min_capacity = 512;
            std::size_t max_capacity = 1024;
        };

        using Point = boost::geometry::model::point<T, Dim, boost::geometry::cs::cartesian>;
        using Box = boost::geometry::model::box<Point>;
        using SplitMode = SplitModeType;
        using BoostRTree = boost::geometry::index::rtree<Box, SplitMode>;

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::shared_ptr<BoostRTree> m_rtree_ = nullptr;

    public:
        explicit RTree(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            if (m_setting_ == nullptr) { m_setting_ = std::make_shared<Setting>(); }
            ERL_ASSERTM(
                m_setting_->min_capacity > m_setting_->max_capacity,
                "min_capacity should be less than or equal to max_capacity");
            if (!std::is_same_v<boost::geometry::index::dynamic_linear, SplitMode> &&     //
                !std::is_same_v<boost::geometry::index::dynamic_quadratic, SplitMode> &&  //
                !std::is_same_v<boost::geometry::index::dynamic_rstar, SplitMode>) {
                // split mode is static, check min_capacity and max_capacity
                ERL_ASSERTM(
                    SplitMode::get_max_elements() == m_setting_->max_capacity,
                    "max_capacity should be {}.",
                    SplitMode::get_max_elements());
                ERL_ASSERTM(
                    SplitMode::get_min_elements() == m_setting_->min_capacity,
                    "min_capacity should be {}.",
                    SplitMode::get_min_elements());
                m_rtree_ = std::make_shared<BoostRTree>();
            } else {
                SplitMode split_mode(m_setting_->max_capacity, m_setting_->min_capacity);
                m_rtree_ = std::make_shared<BoostRTree>(split_mode);
            }
        }

        // TODO: finish implementation
        // ref:
        // https://github.com/mloskot/spatial_index_benchmark/blob/master/benchmark_boost_geometry.cpp
    };

}  // namespace erl::geometry
