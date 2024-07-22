#include "erl_geometry/abstract_occupancy_octree.hpp"

#include <fstream>

namespace erl::geometry {

    bool
    AbstractOccupancyOctree::WriteBinary(const std::string &filename) {
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
            ERL_DEBUG("Successfully wrote Octree of type {}, size {}", GetTreeType(), GetSize());
        } else {
            ERL_WARN("Failed to write Octree of type {}, size {}", GetTreeType(), GetSize());
        }
        return success;
    }

    std::ostream &
    AbstractOccupancyOctree::WriteBinary(std::ostream &s) {
        ToMaxLikelihood();
        Prune();
        return const_cast<const AbstractOccupancyOctree *>(this)->WriteBinary(s);
    }

    bool
    AbstractOccupancyOctree::WriteBinary(const std::string &filename) const {
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
            ERL_DEBUG("Successfully wrote Octree of type {}, size {}", GetTreeType(), GetSize());
        } else {
            ERL_WARN("Failed to write Octree of type {}, size {}", GetTreeType(), GetSize());
        }
        return success;
    }

    std::ostream &
    AbstractOccupancyOctree::WriteBinary(std::ostream &s) const {
        s << sk_BinaryFileHeader_ << std::endl
          << "# (feel free to add / change comments, but leave the first line as it is!)\n#" << std::endl
          << "id " << GetTreeType() << std::endl
          << "size " << GetSize() << std::endl
          << "setting" << std::endl;
        WriteSetting(s);
        s << "data" << std::endl;
        return WriteBinaryData(s);
    }

    bool
    AbstractOccupancyOctree::ReadBinary(const std::string &filename) {
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

    bool
    AbstractOccupancyOctree::ReadBinary(std::istream &s) {
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
        if (!ReadHeader(s, tree_id, size)) { return false; }
        if (tree_id != GetTreeType()) {
            ERL_WARN("Error reading Octree header, ID does not match: {} != {}", tree_id.c_str(), GetTreeType().c_str());
            return false;
        }

        // clear and read setting
        Clear();
        if (!ReadSetting(s)) {
            ERL_WARN("Failed to read setting");
            return false;
        }
        ApplySetting();

        // check if the next line is "data"
        std::getline(s, line);
        if (line.compare(0, 4, "data") != 0) {
            ERL_WARN("Expected 'data' keyword, got: {}", line.c_str());
            return false;
        }

        // read binary data
        if (size > 0) { ReadBinaryData(s); }
        ERL_DEBUG("Done ({} nodes).", GetSize());
        return GetSize() == size;
    }
}  // namespace erl::geometry
