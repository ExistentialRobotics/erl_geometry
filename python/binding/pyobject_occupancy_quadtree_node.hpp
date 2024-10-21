#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_node.hpp"

namespace erl::geometry {
    class PyObjectOccupancyQuadtreeNode : public OccupancyQuadtreeNode {
    protected:
        py::object m_py_object_ = py::none();  // Store the Python object

    public:
        explicit PyObjectOccupancyQuadtreeNode(const uint32_t depth = 0, const int child_index = -1, const float log_odds = 0)
            : OccupancyQuadtreeNode(depth, child_index, log_odds) {}

        PyObjectOccupancyQuadtreeNode(const PyObjectOccupancyQuadtreeNode &other) = default;
        PyObjectOccupancyQuadtreeNode &
        operator=(const PyObjectOccupancyQuadtreeNode &other) = default;
        PyObjectOccupancyQuadtreeNode(PyObjectOccupancyQuadtreeNode &&other) = default;
        PyObjectOccupancyQuadtreeNode &
        operator=(PyObjectOccupancyQuadtreeNode &&other) = default;

        bool
        operator==(const AbstractQuadtreeNode &other) const override {
            if (OccupancyQuadtreeNode::operator==(other)) {
                const auto &other_node = reinterpret_cast<const PyObjectOccupancyQuadtreeNode &>(other);
                return m_py_object_.is(other_node.m_py_object_);  // Compare Python objects
            }
            return false;
        }

        [[nodiscard]] AbstractQuadtreeNode *
        Create(const uint32_t depth, const int child_index) const override {
            const auto node = new PyObjectOccupancyQuadtreeNode(depth, child_index, /*log_odds*/ 0);
            ERL_TRACY_RECORD_ALLOC(node, sizeof(OccupancyQuadtreeNode));
            return node;
        }

        [[nodiscard]] AbstractQuadtreeNode *
        Clone() const override {
            const auto node = new PyObjectOccupancyQuadtreeNode(*this);
            ERL_TRACY_RECORD_ALLOC(node, sizeof(OccupancyQuadtreeNode));
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
                "PyObjectOccupancyQuadtreeNode does not support reading Python objects from file. "
                "Please load the Python object separately.");
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            s.write(reinterpret_cast<const char *>(&m_log_odds_), sizeof(float));
            ERL_WARN_ONCE(
                "PyObjectOccupancyQuadtreeNode does not support writing Python objects to file. "
                "Please save the Python object separately.");
            return s;
        }
    };

    ERL_REGISTER_QUADTREE_NODE(PyObjectOccupancyQuadtreeNode);  // Register the node
}  // namespace erl::geometry
