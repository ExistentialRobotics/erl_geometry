#pragma once

#include <fstream>

namespace erl::geometry {

    template<typename Dtype>
    AbstractOccupancyOctree<Dtype>::AbstractOccupancyOctree(
        std::shared_ptr<OccupancyNdTreeSetting> setting)
        : Super(setting),
          m_setting_(std::move(setting)) {}

    template<typename Dtype>
    bool
    AbstractOccupancyOctree<Dtype>::WriteBinary(std::ostream &s, const bool prune) {
        if (prune) {
            ToMaxLikelihood();
            this->Prune();
            s << "# Pruned octree\n";  // add a comment line
        }
        return const_cast<const AbstractOccupancyOctree *>(this)->WriteBinary(s);
    }

    template<typename Dtype>
    bool
    AbstractOccupancyOctree<Dtype>::WriteBinary(std::ostream &s) const {
        s << "# Binary file\n";  // add a comment line
        static const common::TokenWriteFunctionPairs<AbstractOccupancyOctree> token_function_pairs =
            {
                {
                    "setting",
                    [](const AbstractOccupancyOctree *self, std::ostream &stream) {
                        self->WriteSetting(stream);
                        return stream.good();
                    },
                },
                {
                    "data",
                    [](const AbstractOccupancyOctree *self, std::ostream &stream) {
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
    AbstractOccupancyOctree<Dtype>::ReadBinary(std::istream &s) {
        static const common::TokenReadFunctionPairs<AbstractOccupancyOctree> token_function_pairs =
            {
                {
                    "setting",
                    [](AbstractOccupancyOctree *self, std::istream &stream) {
                        self->Clear();  // clear the tree before reading the setting
                        if (!self->ReadSetting(stream)) { return false; }
                        self->ApplySetting();
                        return stream.good();
                    },
                },
                {
                    "data",
                    [](AbstractOccupancyOctree *self, std::istream &stream) {
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
    AbstractOccupancyOctree<Dtype>::IsNodeOccupied(const OccupancyOctreeNode *node) const {
        return node->GetLogOdds() > m_setting_->log_odd_occ_threshold;
    }

    template<typename Dtype>
    bool
    AbstractOccupancyOctree<Dtype>::IsNodeAtThreshold(const OccupancyOctreeNode *node) const {
        const float log_odds = node->GetLogOdds();
        return log_odds >= m_setting_->log_odd_max || log_odds <= m_setting_->log_odd_min;
    }

    template<typename Dtype>
    const OccupancyOctreeNode *
    AbstractOccupancyOctree<Dtype>::GetHitOccupiedNode(
        const Eigen::Ref<typename Super::Vector3> &p,
        const Eigen::Ref<typename Super::Vector3> &v,
        const bool ignore_unknown,
        const Dtype max_range,
        typename Super::Vector3 &hit_position) {
        return GetHitOccupiedNode(
            p.x(),
            p.y(),
            p.z(),
            v.x(),
            v.y(),
            v.z(),
            ignore_unknown,
            max_range,
            hit_position.x(),
            hit_position.y(),
            hit_position.z());
    }

}  // namespace erl::geometry
