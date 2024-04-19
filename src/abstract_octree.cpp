#include <fstream>
#include "erl_geometry/abstract_octree.hpp"
#include "erl_common/assert.hpp"

namespace erl::geometry {

    std::shared_ptr<AbstractOctree>
    AbstractOctree::CreateTree(const std::string &tree_id) {
        auto it = s_class_id_mapping_.find(tree_id);
        if (it == s_class_id_mapping_.end()) {
            ERL_WARN("Unknown Octree type: %s", tree_id.c_str());
            return nullptr;
        }

        auto tree = it->second->Create();
        return tree;
    }

    bool
    AbstractOctree::Write(const std::string &filename) const {
        ERL_INFO("Writing octree to file: %s", std::filesystem::absolute(filename).string().c_str());
        std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARN("Failed to open file: %s", filename.c_str());
            return false;
        }

        (void) Write(file);
        file.close();
        ERL_INFO("Successfully wrote Octree of type %s, size %zu", GetTreeType().c_str(), GetSize());
        return true;
    }

    std::ostream &
    AbstractOctree::Write(std::ostream &s) const {
        // write header
        s << sk_FileHeader_ << std::endl
          << "# (feel free to add / change comments, but leave the first line as it is!)\n#" << std::endl
          << "id " << GetTreeType() << std::endl
          << "size " << GetSize() << std::endl
          << "setting" << std::endl;
        WriteSetting(s);
        s << "data" << std::endl;
        // write the actual tree data
        return WriteData(s);
    }

    std::shared_ptr<AbstractOctree>
    AbstractOctree::Read(const std::string &filename) {
        ERL_INFO("Reading octree from file: %s", std::filesystem::absolute(filename).string().c_str());
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
        if (line.compare(0, sk_FileHeader_.length(), sk_FileHeader_) != 0) {  // check if the first line is valid
            ERL_WARN("First line of Octree file header does not start with \"%s\"", sk_FileHeader_.c_str());
            return nullptr;
        }

        std::string tree_id;
        uint32_t size;
        if (!ReadHeader(s, tree_id, size)) { return nullptr; }

        ERL_DEBUG("Reading Octree of type %s, size %u", tree_id.c_str(), size);
        auto tree = CreateTree(tree_id);
        if (!tree) { return nullptr; }
        tree->ReadSetting(s);
        tree->ApplySetting();

        std::getline(s, line);  // check if the next line is "data"
        if (line.compare(0, 4, "data") != 0) {
            ERL_WARN("Expected 'data' keyword, got: %s", line.c_str());
            return nullptr;
        }

        // read the actual tree data
        if (size > 0) { tree->ReadData(s); }
        ERL_INFO("Done (%zu nodes).", tree->GetSize());
        return tree;
    }

    bool
    AbstractOctree::LoadData(const std::string &filename) {
        ERL_INFO("Loading data from file: %s", std::filesystem::absolute(filename).string().c_str());
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
        if (!s.good()) {
            ERL_WARN("Input stream is not ready for reading");
            return false;
        }

        std::string line;
        std::getline(s, line);
        if (line.compare(0, sk_FileHeader_.length(), sk_FileHeader_) != 0) {  // check if the first line is valid
            ERL_WARN("First line of Octree file header does not start with \"%s\"", sk_FileHeader_.c_str());
            return false;
        }

        std::string tree_id;
        uint32_t size;
        if (!ReadHeader(s, tree_id, size)) { return false; }

        if (tree_id != GetTreeType()) {
            ERL_WARN("Error reading Octree header, ID does not match: %s != %s", tree_id.c_str(), GetTreeType().c_str());
            return false;
        }
        Clear();
        ReadSetting(s);
        ApplySetting();

        std::getline(s, line);  // check if the next line is "data"
        if (line.compare(0, 4, "data") != 0) {
            ERL_WARN("Expected 'data' keyword, got: %s", line.c_str());
            return false;
        }
        if (size > 0) { ReadData(s); }
        ERL_DEBUG("Done (%zu nodes).", GetSize());
        return GetSize() == size;
    }

    bool
    AbstractOctree::ReadHeader(std::istream &s, std::string &tree_id, uint32_t &size) {
        // initialize output variables
        tree_id = "";
        size = 0;
        // read header
        std::string token;
        bool header_read = false;
        while (s.good() && !header_read) {
            s >> token;
            if (token == "setting") {  // reach the setting section
                header_read = true;    // header read successfully
                // skip forward until end of line
                char c;
                do { c = (char) s.get(); } while (s.good() && (c != '\n'));
            } else if (token.compare(0, 1, "#") == 0) {
                // comment line, skip forward until end of line
                char c;
                do { c = (char) s.get(); } while (s.good() && (c != '\n'));
            } else if (token == "id") {
                s >> tree_id;
            } else if (token == "size") {
                s >> size;
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

        if (tree_id.empty()) {
            ERL_WARN("Error reading Octree header, ID not set");
            return false;
        }

        return true;
    }

    void
    AbstractOctree::RegisterTreeType(const std::shared_ptr<AbstractOctree> &tree) {
        s_class_id_mapping_[tree->GetTreeType()] = tree;
    }

}  // namespace erl::geometry
