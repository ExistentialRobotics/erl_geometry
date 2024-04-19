#pragma once

#include "erl_common/assert.hpp"

#include <cstdint>
#include <typeinfo>

namespace erl::geometry {
    class AbstractOctreeNode {
    protected:
        uint32_t m_depth_ = 0;
        int m_child_index_ = -1;
        AbstractOctreeNode **m_children_ = nullptr;
        uint32_t m_num_children_ = 0;

    public:
        // rules of five: https://www.youtube.com/watch?v=juAZDfsaMvY
        // except for user-defined constructor,
        // always define: destructor, copy constructor, copy assignment, move constructor, move assignment

        AbstractOctreeNode() = delete;

        explicit AbstractOctreeNode(uint32_t depth, int child_index = -1)
            : m_depth_(depth),
              m_child_index_(child_index) {}

        // copy constructor
        AbstractOctreeNode(const AbstractOctreeNode &other)
            : m_depth_(other.m_depth_),
              m_child_index_(other.m_child_index_),
              m_num_children_(other.m_num_children_) {
            if (other.m_children_ == nullptr) { return; }
            AllocateChildrenPtr();
            ERL_ASSERTM(m_children_ != nullptr, "Failed to allocate memory.");
            for (int i = 0; i < 8; ++i) {
                AbstractOctreeNode *child = other.m_children_[i];
                if (child == nullptr) { continue; }
                m_children_[i] = child->Clone();
            }
        };

        // copy assignment
        AbstractOctreeNode &
        operator=(const AbstractOctreeNode &other) {
            if (this == &other) { return *this; }
            m_depth_ = other.m_depth_;
            m_child_index_ = other.m_child_index_;
            m_num_children_ = other.m_num_children_;
            if (other.m_children_ == nullptr) {
                DeleteChildrenPtr();
                return *this;
            }
            AllocateChildrenPtr();
            ERL_ASSERTM(m_children_ != nullptr, "Failed to allocate memory.");
            for (int i = 0; i < 8; ++i) {
                AbstractOctreeNode *child = other.m_children_[i];
                if (child == nullptr) {
                    m_children_[i] = nullptr;
                } else {
                    m_children_[i] = child->Clone();
                }
            }
            return *this;
        }

        // move constructor
        AbstractOctreeNode(AbstractOctreeNode &&other) noexcept
            : m_depth_(other.m_depth_),
              m_child_index_(other.m_child_index_),
              m_children_(other.m_children_),
              m_num_children_(other.m_num_children_) {
            other.m_children_ = nullptr;
            other.m_num_children_ = 0;
        }

        // move assignment
        AbstractOctreeNode &
        operator=(AbstractOctreeNode &&other) noexcept {
            if (this == &other) { return *this; }
            m_depth_ = other.m_depth_;
            m_child_index_ = other.m_child_index_;
            m_children_ = other.m_children_;
            m_num_children_ = other.m_num_children_;
            other.m_children_ = nullptr;
            other.m_num_children_ = 0;
            return *this;
        }

        // destructor
        virtual ~AbstractOctreeNode() { DeleteChildrenPtr(); }

        [[nodiscard]] virtual AbstractOctreeNode *
        Clone() const = 0;

        //-- attributes

        [[nodiscard]] inline const uint32_t &
        GetDepth() const {
            return m_depth_;
        }

        [[nodiscard]] inline const int &
        GetChildIndex() const {
            return m_child_index_;
        }

        //-- file IO
        virtual std::istream &
        ReadData(std::istream &s) = 0;

        virtual std::ostream &
        WriteData(std::ostream &s) const = 0;

        //-- comparison

        [[nodiscard]]
        virtual bool
        operator==(const AbstractOctreeNode &other) const {  // NOLINT(*-no-recursion)
            // we don't do polymorphic check because it is expensive to do so here.
            // polymorphic check should be done by the tree: if two trees are the same type, their nodes should be the same type.
            // Unless we hack it by assigning nodes of wrong type to the tree, which is not supposed to happen.
            if (m_depth_ != other.m_depth_ || m_child_index_ != other.m_child_index_ || m_num_children_ != other.m_num_children_) { return false; }
            if (m_num_children_ == 0 && other.m_num_children_ == 0) { return true; }
            for (int i = 0; i < 8; ++i) {
                if (m_children_[i] == nullptr && other.m_children_[i] == nullptr) { continue; }
                if (m_children_[i] == nullptr || other.m_children_[i] == nullptr) { return false; }
                if (!(*m_children_[i] == *other.m_children_[i])) { return false; }
            }
            return true;
        }

        bool
        operator!=(const AbstractOctreeNode &other) const {
            return !(*this == other);
        }

        //-- children

        inline void
        AllocateChildrenPtr() {
            if (m_children_ != nullptr) { return; }
            m_children_ = new AbstractOctreeNode *[8];
            for (int i = 0; i < 8; ++i) { m_children_[i] = nullptr; }
        }

        inline void
        DeleteChildrenPtr() {
            if (m_children_ == nullptr) { return; }
            for (int i = 0; i < 8; ++i) {
                if (m_children_[i] != nullptr) { delete m_children_[i]; }
            }
            delete[] m_children_;
            m_children_ = nullptr;
            m_num_children_ = 0;
        }

        [[nodiscard]] inline uint32_t
        GetNumChildren() const {
            return m_num_children_;
        }

        [[nodiscard]] inline bool
        HasAnyChild() const {
            return m_num_children_ > 0;
        }

        [[nodiscard]] inline bool
        HasChild(uint32_t index) const {
            if (m_num_children_ == 0) { return false; }
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            return m_children_[index] != nullptr;
        }

        template<typename Derived>
        [[nodiscard]] inline Derived *
        CreateChild(uint32_t index) {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            ERL_DEBUG_ASSERT(m_children_[index] == nullptr, "Child %u already exists.", index);
            AbstractOctreeNode *child = AllocateChildPtr(index);
            ERL_DEBUG_ASSERT(typeid(*child) == typeid(Derived), "AllocateChildPtr(index) must return a pointer of the correct type.");
            ERL_DEBUG_ASSERT(child->m_depth_ == m_depth_ + 1, "Child depth is not set correctly by AllocateChildPtr(index).");
            m_children_[index] = child;
            m_num_children_++;
            return static_cast<Derived *>(child);
        }

        inline void
        RemoveChild(uint32_t index) {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            ERL_DEBUG_ASSERT(m_children_[index] != nullptr, "Child %u does not exist.", index);
            delete m_children_[index];
            m_children_[index] = nullptr;
            m_num_children_--;
        }

        template<typename Derived>
        inline Derived *
        GetChild(uint32_t index) {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            return static_cast<Derived *>(m_children_[index]);
        }

        template<typename Derived>
        [[nodiscard]] inline const Derived *
        GetChild(uint32_t index) const {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            return static_cast<const Derived *>(m_children_[index]);
        }

        [[nodiscard]] virtual inline bool
        AllowMerge(const AbstractOctreeNode *other) const {
            (void) other;
            return m_num_children_ == 0;
        }

        virtual inline void
        Prune() {
            ERL_DEBUG_ASSERT(m_num_children_ == 8, "Prune() can only be called when all children are present.");
            for (int i = 0; i < 8; ++i) {
                delete m_children_[i];
                m_children_[i] = nullptr;
            }
            m_num_children_ = 0;
        }

        virtual inline void
        Expand() {
            ERL_DEBUG_ASSERT(m_num_children_ == 0, "Expand() can only be called when no children are present.");
            if (m_children_ == nullptr) { m_children_ = new AbstractOctreeNode *[8]; }
            for (int i = 0; i < 8; ++i) { m_children_[i] = AllocateChildPtr(i); }
            m_num_children_ = 8;
        }

    private:  // derived classes should implement this
        virtual AbstractOctreeNode *
        AllocateChildPtr(uint32_t index) = 0;
    };
}  // namespace erl::geometry
