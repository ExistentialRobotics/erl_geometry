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
            ERL_WARN_ONCE(
                "PyObjectOccupancyOctreeNode does not support reading Python objects from file. "
                "Please load the Python object separately.");
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            s.write(reinterpret_cast<const char *>(&m_log_odds_), sizeof(float));
            ERL_WARN_ONCE(
                "PyObjectOccupancyOctreeNode does not support writing Python objects to file. "
                "Please save the Python object separately.");
            return s;
        }
    };

    ERL_REGISTER_OCTREE_NODE(PyObjectOccupancyOctreeNode);  // Register the node
}  // namespace erl::geometry
