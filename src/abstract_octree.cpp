#include "erl_geometry/abstract_octree.hpp"
#include <fstream>
#include "erl_common/assert.hpp"

namespace erl::geometry {

    bool
    AbstractOctree::Write(const std::string &filename) const {
        std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: %s", filename.c_str());
            return false;
        }

        (void) Write(file);
        file.close();
        ERL_DEBUG("Successfully wrote Octree of type %s, size %zu, resolution %f", GetTreeType().c_str(), GetSize(), GetResolution());
        return true;
    }

    std::ostream &
    AbstractOctree::Write(std::ostream &s) const {
        s << sk_FileHeader_ << std::endl
          << "# (feel free to add / change comments, but leave the first line as it is!)\n#" << std::endl
          << "id " << GetTreeType() << std::endl
          << "size " << GetSize() << std::endl
          << "res " << GetResolution() << std::endl
          << "data" << std::endl;
        return WriteData(s);
    }

    std::shared_ptr<AbstractOctree>
    AbstractOctree::Read(const std::string &filename) {
        std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: %s", filename.c_str());
            return nullptr;
        }

        auto tree = Read(file);
        file.close();
        return tree;
    }

    std::shared_ptr<AbstractOctree>
    AbstractOctree::Read(std::istream &s) {
        if (!s.good()) {
            ERL_WARN("Input stream is not ready for reading");
            return nullptr;
        }

        // check if the first line is valid
        std::string line;
        std::getline(s, line);
        if (line.compare(0, sk_FileHeader_.length(), sk_FileHeader_) != 0) {
            ERL_WARN("First line of Octree file header does not start with \"%s\"", sk_FileHeader_.c_str());
            return nullptr;
        }

        std::string id;
        uint32_t size;
        double res;
        if (!ReadHeader(s, id, size, res)) { return nullptr; }

        ERL_DEBUG("Reading Octree of type %s, size %u, resolution %f", id.c_str(), size, res);
        auto tree = CreateTree(id, res);
        if (!tree) { return nullptr; }
        if (size > 0) { tree->ReadData(s); }
        ERL_DEBUG("Done (%zu nodes).", tree->GetSize());
        return tree;
    }

    bool
    AbstractOctree::LoadData(const std::string &filename) {
        std::ifstream s(filename.c_str(), std::ios_base::in | std::ios_base::binary);
        if (!s.is_open()) {
            ERL_WARN("Failed to open file: %s", filename.c_str());
            return false;
        }
        bool success = LoadData(s);
        s.close();
        return success;
    }

    bool
    AbstractOctree::LoadData(std::istream &s) {
        std::string id;
        uint32_t size;
        double res;
        if (!ReadHeader(s, id, size, res)) { return false; }
        if (id != GetTreeType()) {
            ERL_WARN("Error reading Octree header, ID does not match: %s != %s", id.c_str(), GetTreeType().c_str());
            return false;
        }
        Clear();
        SetResolution(res);
        if (size > 0) { this->ReadData(s); }
        ERL_DEBUG("Done (%zu nodes).", GetSize());
        return GetSize() == size;
    }

    std::shared_ptr<AbstractOctree>
    AbstractOctree::CreateTree(const std::string &id, double res) {
        auto it = s_class_id_mapping_.find(id);
        if (it == s_class_id_mapping_.end()) {
            ERL_WARN("Unknown Octree type: %s", id.c_str());
            return nullptr;
        }

        auto tree = it->second->Create();
        tree->SetResolution(res);
        return tree;
    }

    bool
    AbstractOctree::ReadHeader(std::istream &s, std::string &id, uint32_t &size, double &res) {
        // initialize output variables
        id = "";
        size = 0;
        res = 0.0;

        // read header
        std::string token;
        bool header_read = false;
        while (s.good() && !header_read) {
            s >> token;
            if (token == "data") {   // reach the data section
                header_read = true;  // header read successfully
                // skip forward until end of line
                char c;
                do { c = (char) s.get(); } while (s.good() && (c != '\n'));
            } else if (token.compare(0, 1, "#") == 0) {
                // comment line, skip forward until end of line
                char c;
                do { c = (char) s.get(); } while (s.good() && (c != '\n'));
            } else if (token == "id") {
                s >> id;
            } else if (token == "size") {
                s >> size;
            } else if (token == "res") {
                s >> res;
            } else {
                ERL_WARN("Unknown keyword in Octree header, skipping: %s", token.c_str());
                char c;
                do { c = (char) s.get(); } while (s.good() && (c != '\n'));
            }
        }

        if (!header_read) {
            ERL_WARN("Error reading Octree header");
            return false;
        }

        if (id.empty()) {
            ERL_WARN("Error reading Octree header, ID not set");
            return false;
        }

        if (res <= 0.0) {
            ERL_WARN("Error reading Octree header, res <= 0.0");
            return false;
        }

        return true;
    }

    void
    AbstractOctree::RegisterTreeType(const std::shared_ptr<AbstractOctree> &tree) {
        s_class_id_mapping_[tree->GetTreeType()] = tree;
    }

}  // namespace erl::geometry
