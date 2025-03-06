#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_octree_node.hpp"

namespace erl::geometry {
    class PyObjectOccupancyOctreeNode : public OccupancyOctreeNode {
    protected:
        py::object m_py_object_ = py::none();  // Store the Python object

    public:
        explicit PyObjectOccupancyOctreeNode(const uint32_t depth = 0, const int child_index = -1, const float log_odds = 0)
            : OccupancyOctreeNode(depth, child_index, log_odds) {}

        PyObjectOccupancyOctreeNode(const PyObjectOccupancyOctreeNode &other) = default;
        PyObjectOccupancyOctreeNode &
        operator=(const PyObjectOccupancyOctreeNode &other) = default;
        PyObjectOccupancyOctreeNode(PyObjectOccupancyOctreeNode &&other) = default;
        PyObjectOccupancyOctreeNode &
        operator=(PyObjectOccupancyOctreeNode &&other) = default;

        bool
        operator==(const AbstractOctreeNode &other) const override {
            if (OccupancyOctreeNode::operator==(other)) {
                const auto &other_node = reinterpret_cast<const PyObjectOccupancyOctreeNode &>(other);
                return m_py_object_.is(other_node.m_py_object_);  // Compare Python objects
            }
            return false;
        }

        [[nodiscard]] AbstractOctreeNode *
        Create(const uint32_t depth, const int child_index) const override {
            const auto node = new PyObjectOccupancyOctreeNode(depth, child_index, /*log_odds*/ 0);
            ERL_TRACY_RECORD_ALLOC(node, sizeof(OccupancyOctreeNode));
            return node;
        }

        [[nodiscard]] AbstractOctreeNode *
        Clone() const override {
            const auto node = new PyObjectOccupancyOctreeNode(*this);
            ERL_TRACY_RECORD_ALLOC(node, sizeof(OccupancyOctreeNode));
            return node;
        }

        [[nodiscard]] py::object
        GetPyObject() const {
            return m_py_object_;
        }

        void
        SetPyObject(const py::object &py_object = py::none()) {
            m_py_object_ = py_object;
        }

        //-- file IO
        std::istream &
        ReadData(std::istream &s) override {
            s.read(reinterpret_cast<char *>(&m_log_odds_), sizeof(float));
            try {
                long length;
                s.read(reinterpret_cast<char *>(&length), sizeof(long));  // Read the length of the data
                std::vector<char> buffer(length);
                s.read(buffer.data(), length);                                                                // Read the data into the buffer
                m_py_object_ = py::module::import("pickle").attr("loads")(py::bytes(buffer.data(), length));  // Load the Python object
            } catch (const std::exception &e) { ERL_WARN_ONCE("Failed to read Python object: {}", e.what()); }
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            s.write(reinterpret_cast<const char *>(&m_log_odds_), sizeof(float));
            try {
                const py::bytes data = py::module::import("pickle").attr("dumps")(m_py_object_);
                const py::buffer_info info(py::buffer(data).request());          // Get buffer info
                const long length = info.size;                                   // Get the length of the data
                s.write(reinterpret_cast<const char *>(&length), sizeof(long));  // Write the length of the data
                s.write(static_cast<const char *>(info.ptr), length);            // Write the data to the stream
            } catch (const std::exception &e) { ERL_WARN_ONCE("Failed to write Python object: {}", e.what()); }
            return s;
        }
    };
}  // namespace erl::geometry
