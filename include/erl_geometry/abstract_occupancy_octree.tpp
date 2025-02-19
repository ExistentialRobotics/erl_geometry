#pragma once

#include <fstream>

namespace erl::geometry {

    template<typename Dtype>
    AbstractOccupancyOctree<Dtype>::AbstractOccupancyOctree(std::shared_ptr<OccupancyNdTreeSetting> setting)
        : Super(setting),
          m_setting_(std::move(setting)) {}

    template<typename Dtype>
    bool
    AbstractOccupancyOctree<Dtype>::WriteBinary(const std::string &filename) {
        ERL_DEBUG("Writing Octree to file: {}", std::filesystem::absolute(filename));
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename);
            return false;
        }

        WriteBinary(file);
        const bool success = file.good();
        file.close();
        if (success) {
            ERL_DEBUG("Successfully wrote Octree of type {}, size {}", Super::GetTreeType(), this->GetSize());
        } else {
            ERL_WARN("Failed to write Octree of type {}, size {}", Super::GetTreeType(), this->GetSize());
        }
        return success;
    }

    template<typename Dtype>
    std::ostream &
    AbstractOccupancyOctree<Dtype>::WriteBinary(std::ostream &s) {
        ToMaxLikelihood();
        this->Prune();
        return const_cast<const AbstractOccupancyOctree *>(this)->WriteBinary(s);
    }

    template<typename Dtype>
    bool
    AbstractOccupancyOctree<Dtype>::WriteBinary(const std::string &filename) const {
        ERL_DEBUG("Writing Octree to file: {}", std::filesystem::absolute(filename));
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename);
            return false;
        }

        WriteBinary(file);
        const bool success = file.good();
        file.close();
        if (success) {
            ERL_DEBUG("Successfully wrote Octree of type {}, size {}", Super::GetTreeType(), this->GetSize());
        } else {
            ERL_WARN("Failed to write Octree of type {}, size {}", Super::GetTreeType(), this->GetSize());
        }
        return success;
    }

    template<typename Dtype>
    std::ostream &
    AbstractOccupancyOctree<Dtype>::WriteBinary(std::ostream &s) const {
        s << sk_BinaryFileHeader_ << std::endl
          << "# (feel free to add / change comments, but leave the first line as it is!)\n#" << std::endl
          << "id " << Super::GetTreeType() << std::endl
          << "size " << this->GetSize() << std::endl
          << "setting" << std::endl;
        Super::WriteSetting(s);
        s << "data" << std::endl;
        return WriteBinaryData(s);
    }

    template<typename Dtype>
    bool
    AbstractOccupancyOctree<Dtype>::ReadBinary(const std::string &filename) {
        ERL_DEBUG("Reading Octree from file: {}", std::filesystem::absolute(filename).c_str());
        std::ifstream file(filename.c_str(), std::ios::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename.c_str());
            return false;
        }

        const bool success = ReadBinary(file);
        file.close();
        return success;
    }

    template<typename Dtype>
    bool
    AbstractOccupancyOctree<Dtype>::ReadBinary(std::istream &s) {
        if (!s.good()) {
            ERL_WARN("Input stream is not ready for reading");
            return false;
        }

        // check if the first line is valid
        std::string line;
        std::getline(s, line);
        if (line.compare(0, sk_BinaryFileHeader_.length(), sk_BinaryFileHeader_) != 0) {
            ERL_WARN("First line of Octree file header does not start with \"{}\"", sk_BinaryFileHeader_.c_str());
            return false;
        }

        // read header
        std::string tree_id;
        uint32_t size;
        if (!Super::ReadHeader(s, tree_id, size)) { return false; }
        if (tree_id != Super::GetTreeType()) {
            ERL_WARN("Error reading Octree header, ID does not match: {} != {}", tree_id.c_str(), Super::GetTreeType().c_str());
            return false;
        }

        // clear and read setting
        this->Clear();
        if (!Super::ReadSetting(s)) {
            ERL_WARN("Failed to read setting");
            return false;
        }
        this->ApplySetting();

        // check if the next line is "data"
        std::getline(s, line);
        if (line.compare(0, 4, "data") != 0) {
            ERL_WARN("Expected 'data' keyword, got: {}", line.c_str());
            return false;
        }

        // read binary data
        if (size > 0) { ReadBinaryData(s); }
        ERL_DEBUG("Done ({} nodes).", this->GetSize());
        return this->GetSize() == size;
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
        return GetHitOccupiedNode(p.x(), p.y(), p.z(), v.x(), v.y(), v.z(), ignore_unknown, max_range, hit_position.x(), hit_position.y(), hit_position.z());
    }

}  // namespace erl::geometry
