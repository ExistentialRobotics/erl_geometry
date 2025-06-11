#include "erl_geometry/abstract_occupancy_quadtree.hpp"

#include "erl_common/serialization.hpp"

#include <fstream>

namespace erl::geometry {

    template<typename Dtype>
    AbstractOccupancyQuadtree<Dtype>::AbstractOccupancyQuadtree(
        std::shared_ptr<OccupancyNdTreeSetting> setting)
        : AbstractQuadtree<Dtype>(setting),
          m_setting_(std::move(setting)) {}

    template<typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::WriteBinary(std::ostream &s, const bool prune) {
        if (prune) {
            this->ToMaxLikelihood();
            this->Prune();
            s << "# Pruned quadtree\n";  // add a comment line
        }
        return const_cast<const AbstractOccupancyQuadtree *>(this)->WriteBinary(s);
    }

    template<typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::WriteBinary(std::ostream &s) const {
        s << "# Binary file\n";  // add a comment line
        static const common::TokenWriteFunctionPairs<AbstractOccupancyQuadtree>
            token_function_pairs = {
                {
                    "setting",
                    [](const AbstractOccupancyQuadtree *self, std::ostream &stream) {
                        self->WriteSetting(stream);
                        return stream.good();
                    },
                },
                {
                    "data",
                    [](const AbstractOccupancyQuadtree *self, std::ostream &stream) {
                        const std::size_t size = self->GetSize();
                        stream << size << '\n';
                        if (size > 0) { return self->WriteBinaryData(stream) && stream.good(); }
                        return stream.good();
                    },
                },
            };
        return common::WriteTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::ReadBinary(std::istream &s) {
        static const common::TokenReadFunctionPairs<AbstractOccupancyQuadtree>
            token_function_pairs = {
                {
                    "setting",
                    [](AbstractOccupancyQuadtree *self, std::istream &stream) {
                        self->Clear();  // clear the tree before reading the setting
                        if (!self->ReadSetting(stream)) { return false; }
                        self->ApplySetting();
                        return stream.good();
                    },
                },
                {
                    "data",
                    [](AbstractOccupancyQuadtree *self, std::istream &stream) {
                        std::size_t size;
                        stream >> size;
                        common::SkipLine(stream);
                        if (size > 0) { return self->ReadBinaryData(stream) && stream.good(); }
                        ERL_DEBUG("Load {} nodes", size);
                        return size == self->GetSize() && stream.good();
                    },
                },
            };
        return common::ReadTokens(s, this, token_function_pairs);
    }

    template<typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::IsNodeOccupied(const OccupancyQuadtreeNode *node) const {
        return node->GetLogOdds() >= m_setting_->log_odd_occ_threshold;
    }

    template<typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::IsNodeAtThreshold(const OccupancyQuadtreeNode *node) const {
        const float log_odds = node->GetLogOdds();
        return log_odds >= m_setting_->log_odd_max || log_odds <= m_setting_->log_odd_min;
    }

    template<typename Dtype>
    const OccupancyQuadtreeNode *
    AbstractOccupancyQuadtree<Dtype>::GetHitOccupiedNode(
        const Eigen::Ref<typename Super::Vector2> &p,
        const Eigen::Ref<typename Super::Vector2> &v,
        const bool ignore_unknown,
        const Dtype max_range,
        typename Super::Vector2 &hit_position) {
        return GetHitOccupiedNode(
            p.x(),
            p.y(),
            v.x(),
            v.y(),
            ignore_unknown,
            max_range,
            hit_position.x(),
            hit_position.y());
    }

    template class AbstractOccupancyQuadtree<double>;
    template class AbstractOccupancyQuadtree<float>;
}  // namespace erl::geometry
