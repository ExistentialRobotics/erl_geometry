#include "erl_geometry/abstract_quadtree_node.hpp"

namespace erl::geometry {

    AbstractQuadtreeNode::AbstractQuadtreeNode(const AbstractQuadtreeNode &other)
        : m_depth_(other.m_depth_),
          m_child_index_(other.m_child_index_),
          m_num_children_(other.m_num_children_) {
        if (other.m_children_ == nullptr) { return; }
        this->AllocateChildrenPtr();
        ERL_ASSERTM(m_children_ != nullptr, "Failed to allocate memory.");
        for (int i = 0; i < 4; ++i) {
            const AbstractQuadtreeNode *child = other.m_children_[i];
            if (child == nullptr) { continue; }
            m_children_[i] = child->Clone();
        }
    }

    AbstractQuadtreeNode &
    AbstractQuadtreeNode::operator=(const AbstractQuadtreeNode &other) {
        if (this == &other) { return *this; }
        m_depth_ = other.m_depth_;
        m_child_index_ = other.m_child_index_;
        m_num_children_ = other.m_num_children_;
        if (other.m_children_ == nullptr) {
            this->DeleteChildrenPtr();
            return *this;
        }
        this->AllocateChildrenPtr();
        ERL_ASSERTM(m_children_ != nullptr, "Failed to allocate memory.");
        for (int i = 0; i < 4; ++i) {
            const AbstractQuadtreeNode *child = other.m_children_[i];
            if (child == nullptr) {
                m_children_[i] = nullptr;
            } else {
                m_children_[i] = child->Clone();
            }
        }
        return *this;
    }

    bool
    AbstractQuadtreeNode::operator==(const AbstractQuadtreeNode &other) const {  // NOLINT(*-no-recursion)
        // we don't do polymorphic check because it is expensive to do so here.
        // polymorphic check should be done by the tree: if two trees are the same type, their nodes should be the same type.
        // Unless we hack it by assigning nodes of wrong type to the tree, which is not supposed to happen.
        if (m_depth_ != other.m_depth_ || m_child_index_ != other.m_child_index_ || m_num_children_ != other.m_num_children_) { return false; }
        if (m_num_children_ == 0) { return true; }
        for (int i = 0; i < 4; ++i) {
            if (m_children_[i] == nullptr && other.m_children_[i] == nullptr) { continue; }
            if (m_children_[i] == nullptr || other.m_children_[i] == nullptr) { return false; }
            if (*m_children_[i] != *other.m_children_[i]) { return false; }
        }
        return true;
    }

    void
    AbstractQuadtreeNode::DeleteChildrenPtr() {
        if (m_children_ == nullptr) { return; }
        if (m_num_children_ > 0) {
            for (int i = 0; i < 4; ++i) {
                if (m_children_[i] != nullptr) { delete m_children_[i]; }
            }
            m_num_children_ = 0;
        }
        delete[] m_children_;
        m_children_ = nullptr;
    }

}  // namespace erl::geometry
