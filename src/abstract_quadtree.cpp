#include "erl_geometry/abstract_quadtree.hpp"
#include <fstream>
#include "erl_common/assert.hpp"

namespace erl::geometry {

    bool
    AbstractQuadtree::Write(const std::string &filename) const {
        std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARNING("Failed to open file: %s\n", filename.c_str());
            return false;
        }

        (void) Write(file);
        file.close();
        return true;
    }

    std::ostream &
    AbstractQuadtree::Write(std::ostream &s) const {
        s << sk_FileHeader_ << "\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n"
          << "id " << GetTreeType() << std::endl
          << "size " << GetSize() << std::endl
          << "res " << GetResolution() << std::endl
          << "data" << std::endl;
        return WriteData(s);
    }

    std::shared_ptr<AbstractQuadtree>
    AbstractQuadtree::Read(const std::string &filename) {
        std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
        if (!file.is_open()) {
            ERL_WARNING("Failed to open file: %s\n", filename.c_str());
            return nullptr;
        }

        auto tree = Read(file);
        file.close();
        return tree;
    }

    std::shared_ptr<AbstractQuadtree>
    AbstractQuadtree::Read(std::istream &s) {
        if (!s.good()) {
            ERL_WARNING("Input stream is not ready for reading\n");
            return nullptr;
        }

        // check if the first line is valid
        std::string line;
        std::getline(s, line);
        if (line.compare(0, sk_FileHeader_.length(), sk_FileHeader_) != 0) {
            ERL_WARNING("First line of Quadtree file header does not start with \"%s\"\n", sk_FileHeader_.c_str());
            return nullptr;
        }

        std::string id;
        unsigned int size;
        double res;
        if (!ReadHeader(s, id, size, res)) { return nullptr; }

        ERL_DEBUG("Reading Quadtree of type %s, size %u, resolution %f\n", id.c_str(), size, res);
        auto tree = CreateTree(id, res);
        if (!tree) { return nullptr; }
        if (size > 0) { tree->ReadData(s); }
        ERL_DEBUG("Done (%zu nodes).\n", tree->GetSize());
        return tree;
    }

    std::shared_ptr<AbstractQuadtree>
    AbstractQuadtree::CreateTree(const std::string &id, double res) {
        auto it = s_class_id_mapping_.find(id);
        if (it == s_class_id_mapping_.end()) {
            ERL_WARNING("Unknown Quadtree type: %s\n", id.c_str());
            return nullptr;
        }

        auto tree = it->second->Create();
        tree->SetResolution(res);
        return tree;
    }

    bool
    AbstractQuadtree::ReadHeader(std::istream &s, std::string &id, unsigned int &size, double &res) {
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
                ERL_WARNING("Unknown keyword in Quadtree header, skipping: %s\n", token.c_str());
                char c;
                do { c = (char) s.get(); } while (s.good() && (c != '\n'));
            }
        }

        if (!header_read) {
            ERL_WARNING("Error reading Quadtree header\n");
            return false;
        }

        if (id.empty()) {
            ERL_WARNING("Error reading Quadtree header, ID not set\n");
            return false;
        }

        if (res <= 0.0) {
            ERL_WARNING("Error reading Quadtree header, res <= 0.0\n");
            return false;
        }

        return true;
    }

    void
    AbstractQuadtree::RegisterTreeType(const std::shared_ptr<AbstractQuadtree> &tree) {
        s_class_id_mapping_[tree->GetTreeType()] = tree;
    }

}  // namespace erl::geometry
