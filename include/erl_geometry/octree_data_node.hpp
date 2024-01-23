#pragma once

#include "abstract_octree_node.hpp"
#include <vector>

namespace erl::geometry {

    template<typename T>
    class OctreeDataNode : public AbstractOctreeNode {
    protected:
        T m_value_;

    public:
        using NodeType = OctreeDataNode<T>;

        OctreeDataNode() = default;

        explicit OctreeDataNode(T t)
            : m_value_(t) {}

        /**
         * Copy constructor, deep copy recursively.
         * @param other
         * @attention This is a deep copy, so it is expensive.
         */
        OctreeDataNode(const NodeType &other)
            : AbstractOctreeNode(),
              m_value_(other.m_value_) {
            if (other.m_children_.empty()) { return; }
            AllocateChildrenPtr();
            for (int i = 0; i < 8; ++i) {
                if (other.m_children_[i] != nullptr) {
                    m_children_[i] = std::static_pointer_cast<AbstractOctreeNode>(  // deep copy recursively
                        std::make_shared<NodeType>(*std::static_pointer_cast<NodeType>(other.m_children_[i]))
                    );
                }
            }
        }

        bool
        operator==(const NodeType &other) const {
            return m_value_ == other.m_value_;
        }

        bool
        operator!=(const NodeType &other) const {
            return m_value_ != other.m_value_;
        }

        //-- data manipulation
        inline void
        CopyData(const NodeType &other) {
            m_value_ = other.m_value_;
        }

        inline T &
        GetValue() {
            return m_value_;
        }

        inline const T &
        GetValue() const {
            return m_value_;
        }

        inline void
        SetValue(T t) {
            m_value_ = t;
        }

        //-- file IO
        std::istream &
        ReadData(std::istream &s) override {
            s.read(reinterpret_cast<char *>(&m_value_), sizeof(T));
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            s.write(reinterpret_cast<const char *>(&m_value_), sizeof(T));
            return s;
        }
    };
}  // namespace erl::geometry
