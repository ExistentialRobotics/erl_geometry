#include "erl_geometry/abstract_occupancy_quadtree.hpp"

#include <fstream>

namespace erl::geometry {

    template <typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::WriteBinary(const std::string &filename) {
        ERL_DEBUG("Writing Quadtree to file: {}", std::filesystem::absolute(filename));
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename);
            return false;
        }

        WriteBinary(file);
        const bool success = file.good();
        file.close();
        if (success) {
            ERL_DEBUG("Successfully wrote Quadtree of type {}, size {}", this->GetTreeType(), this->GetSize());
        } else {
            ERL_WARN("Failed to write Quadtree of type {}, size {}", this->GetTreeType(), this->GetSize());
        }
        return success;
    }

    template <typename Dtype>
    std::ostream &
    AbstractOccupancyQuadtree<Dtype>::WriteBinary(std::ostream &s) {
        this->ToMaxLikelihood();
        this->Prune();
        return const_cast<const AbstractOccupancyQuadtree *>(this)->WriteBinary(s);
    }

    template <typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::WriteBinary(const std::string &filename) const {
        ERL_DEBUG("Writing Quadtree to file: {}", std::filesystem::absolute(filename));
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename);
            return false;
        }

        WriteBinary(file);
        const bool success = file.good();
        file.close();
        if (success) {
            ERL_DEBUG("Successfully wrote Quadtree of type {}, size {}", this->GetTreeType(), this->GetSize());
        } else {
            ERL_WARN("Failed to write Quadtree of type {}, size {}", this->GetTreeType(), this->GetSize());
        }
        return success;
    }

    template <typename Dtype>
    std::ostream &
    AbstractOccupancyQuadtree<Dtype>::WriteBinary(std::ostream &s) const {
        s << sk_BinaryFileHeader_ << std::endl
          << "# (feel free to add / change comments, but leave the first line as it is!)\n#" << std::endl
          << "id " << this->GetTreeType() << std::endl
          << "size " << this->GetSize() << std::endl
          << "setting" << std::endl;
        this->WriteSetting(s);
        s << "data" << std::endl;
        return this->WriteBinaryData(s);
    }

    template <typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::ReadBinary(const std::string &filename) {
        ERL_DEBUG("Reading Quadtree from file: {}", std::filesystem::absolute(filename));
        std::ifstream file(filename.c_str(), std::ios::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: {}", filename.c_str());
            return false;
        }

        const bool success = ReadBinary(file);
        file.close();
        return success;
    }

    template <typename Dtype>
    bool
    AbstractOccupancyQuadtree<Dtype>::ReadBinary(std::istream &s) {
        if (!s.good()) {
            ERL_WARN("Input stream is not ready for reading");
            return false;
        }

        // check if the first line is valid
        std::string line;
        std::getline(s, line);
        if (line.compare(0, sk_BinaryFileHeader_.length(), sk_BinaryFileHeader_) != 0) {
            ERL_WARN("First line of Quadtree file header does not start with \"{}\"", sk_BinaryFileHeader_);
            return false;
        }

        // read header
        std::string tree_id;
        uint32_t size;
        if (!this->ReadHeader(s, tree_id, size)) { return false; }
        if (tree_id != this->GetTreeType()) {
            ERL_WARN("Error reading Quadtree header, ID does not match: {} != {}", tree_id, this->GetTreeType());
            return false;
        }

        // clear and read setting
        this->Clear();
        if (!this->ReadSetting(s)) {
            ERL_WARN("Failed to read setting");
            return false;
        }
        this->ApplySetting();

        // check if the next line is "data"
        std::getline(s, line);
        if (line.compare(0, 4, "data") != 0) {
            ERL_WARN("Expected 'data' keyword, got: {}", line);
            return false;
        }

        // read binary data
        if (size > 0) { this->ReadBinaryData(s); }
        ERL_DEBUG("Done ({} nodes).", this->GetSize());
        return this->GetSize() == size;
    }
}  // namespace erl::geometry
