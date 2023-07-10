#include "erl_geometry/abstract_occupancy_quadtree.hpp"
#include <fstream>

namespace erl::geometry {

    bool
    AbstractOccupancyQuadtree::WriteBinary(const std::string &filename) {
        std::ofstream file(filename.c_str(), std::ios::binary);
        if (!file.is_open()) {
            ERL_WARNING("Failed to open file: %s\n", filename.c_str());
            return false;
        }

        bool success = WriteBinary(file);
        file.close();
        return success;
    }

    bool
    AbstractOccupancyQuadtree::WriteBinary(std::ostream &s) {
        ToMaxLikelihood();
        Prune();
        return WriteBinaryConst(s);
    }

    bool
    AbstractOccupancyQuadtree::WriteBinaryConst(const std::string &filename) const {
        std::ofstream file(filename.c_str(), std::ios::binary);
        if (!file.is_open()) {
            ERL_WARNING("Failed to open file: %s\n", filename.c_str());
            return false;
        }

        bool success = WriteBinaryConst(file);
        file.close();
        return success;
    }

    bool
    AbstractOccupancyQuadtree::WriteBinaryConst(std::ostream &s) const {
        // write header
        s << sk_BinaryFileHeader_ << "\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n"
          << "id " << GetTreeType() << std::endl
          << "size " << GetSize() << std::endl
          << "res " << GetResolution() << std::endl
          << "data" << std::endl;

        WriteBinaryData(s);
        if (s.good()) {
            ERL_DEBUG("Successfully wrote Quadtree of type %s, size %zu, resolution %f\n", GetTreeType().c_str(), GetSize(), GetResolution());
            return true;
        } else {
            ERL_WARNING("Failed to write Quadtree of type %s, size %zu, resolution %f\n", GetTreeType().c_str(), GetSize(), GetResolution());
            return false;
        }
    }

    bool
    AbstractOccupancyQuadtree::ReadBinary(const std::string &filename) {
        std::ifstream file(filename.c_str(), std::ios::binary);
        if (!file.is_open()) {
            ERL_WARNING("Failed to open file: %s\n", filename.c_str());
            return false;
        }

        bool success = ReadBinary(file);
        file.close();
        return success;
    }

    bool
    AbstractOccupancyQuadtree::ReadBinary(std::istream &s) {
        if (!s.good()) {
            ERL_WARNING("Input stream is not ready for reading\n");
            return false;
        }

        // check if the first line is valid
        std::string line;
        std::getline(s, line);
        if (line.compare(0, sk_BinaryFileHeader_.length(), sk_BinaryFileHeader_) != 0) {
            ERL_WARNING("First line of Quadtree file header does not start with \"%s\"\n", sk_FileHeader_.c_str());
            return false;
        }

        // read header
        std::string id;
        unsigned int size;
        double res;
        if (!ReadHeader(s, id, size, res)) { return false; }

        // read binary data
        Clear();
        SetResolution(res);
        if (size > 0) { ReadBinaryData(s); }
        if (size != GetSize()) {
            ERL_WARNING("GetSize mismatch: %u != %zu\n", size, GetSize());
            return false;
        }

        ERL_DEBUG("Successfully read Quadtree of type %s, size %zu, resolution %f\n", GetTreeType().c_str(), GetSize(), GetResolution());
        return true;
    }
}  // namespace erl::geometry
