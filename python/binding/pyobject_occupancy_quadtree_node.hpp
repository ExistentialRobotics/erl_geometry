#pragma once

#include "erl_common/pybind11.hpp"
#include "erl_geometry/occupancy_quadtree_node.hpp"

namespace erl::geometry {
    class PyObjectOccupancyQuadtreeNode : public OccupancyQuadtreeNode {
    protected:
        py::object m_py_object_ = py::none();  // Store the Python object

    public:
        explicit PyObjectOccupancyQuadtreeNode(
            const uint32_t depth = 0,
            const int child_index = -1,
            const float log_odds = 0)
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
                const auto &other_node =
                    reinterpret_cast<const PyObjectOccupancyQuadtreeNode &>(other);
                return m_py_object_.is(other_node.m_py_object_);  // Compare Python objects
            }
            return false;
        }

        [[nodiscard]] AbstractQuadtreeNode *
        Create(const uint32_t depth, const int child_index) const override {
            CheckRuntimeType<PyObjectOccupancyQuadtreeNode>(this, /*debug_only*/ true);
            const auto node = new PyObjectOccupancyQuadtreeNode(depth, child_index, /*log_odds*/ 0);
            return node;
        }

        [[nodiscard]] AbstractQuadtreeNode *
        Clone() const override {
            CheckRuntimeType<PyObjectOccupancyQuadtreeNode>(this, /*debug_only*/ true);
            const auto node = new PyObjectOccupancyQuadtreeNode(*this);
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
                // Read the length of the data
                s.read(reinterpret_cast<char *>(&length), sizeof(long));
                std::vector<char> buffer(length);
                s.read(buffer.data(), length);               // Read the data into the buffer
                m_py_object_ = py::module::import("pickle")  // Load the Python object
                                   .attr("loads")(py::bytes(buffer.data(), length));
            } catch (const std::exception &e) {
                ERL_WARN_ONCE("Failed to read Python object: {}", e.what());
            }
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            s.write(reinterpret_cast<const char *>(&m_log_odds_), sizeof(float));
            try {
                const py::bytes data = py::module::import("pickle").attr("dumps")(m_py_object_);
                const py::buffer_info info(py::buffer(data).request());  // Get buffer info
                const long length = info.size;  // Get the length of the data
                // Write the length of the data
                s.write(reinterpret_cast<const char *>(&length), sizeof(long));
                // Write the data to the stream
                s.write(static_cast<const char *>(info.ptr), length);
            } catch (const std::exception &e) {
                ERL_WARN_ONCE("Failed to write Python object: {}", e.what());
            }
            return s;
        }
    };
}  // namespace erl::geometry
