#pragma once

#include "erl_common/assert.hpp"

#include <cstdint>
#include <typeinfo>

namespace erl::geometry {
    /**
     * AbstractQuadtreeNode is a base class for all quadtree node implementations. It provides a common interface for file I/O and children management.
     */
    class AbstractQuadtreeNode {
    protected:
        uint32_t m_depth_ = 0;
        int m_child_index_ = -1;
        AbstractQuadtreeNode **m_children_ = nullptr;
        uint32_t m_num_children_ = 0;

    public:
        // rules of five: https://www.youtube.com/watch?v=juAZDfsaMvY
        // except for user-defined constructor,
        // always define: destructor, copy constructor, copy assignment, move constructor, move assignment

        AbstractQuadtreeNode() = delete;

        explicit AbstractQuadtreeNode(uint32_t depth, int child_index = -1)
            : m_depth_(depth),
              m_child_index_(child_index) {}

        // copy constructor
        AbstractQuadtreeNode(const AbstractQuadtreeNode &other)
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
        };

        // copy assignment
        AbstractQuadtreeNode &
        operator=(const AbstractQuadtreeNode &other) {
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

        // move constructor
        AbstractQuadtreeNode(AbstractQuadtreeNode &&other) noexcept
            : m_depth_(other.m_depth_),
              m_child_index_(other.m_child_index_),
              m_children_(other.m_children_),
              m_num_children_(other.m_num_children_) {
            other.m_depth_ = 0;
            other.m_child_index_ = -1;
            other.m_children_ = nullptr;
            other.m_num_children_ = 0;
        }

        // move assignment
        AbstractQuadtreeNode &
        operator=(AbstractQuadtreeNode &&other) noexcept {
            if (this == &other) { return *this; }
            m_depth_ = other.m_depth_;
            m_child_index_ = other.m_child_index_;
            m_children_ = other.m_children_;
            m_num_children_ = other.m_num_children_;
            other.m_depth_ = 0;
            other.m_child_index_ = -1;
            other.m_children_ = nullptr;
            other.m_num_children_ = 0;
            return *this;
        }

        // destructor
        virtual ~AbstractQuadtreeNode() { this->DeleteChildrenPtr(); }

        [[nodiscard]] virtual AbstractQuadtreeNode *
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
        operator==(const AbstractQuadtreeNode &other) const {  // NOLINT(*-no-recursion)
            // we don't do polymorphic check because it is expensive to do so here.
            // polymorphic check should be done by the tree: if two trees are the same type, their nodes should be the same type.
            // Unless we hack it by assigning nodes of wrong type to the tree, which is not supposed to happen.
            if (m_depth_ != other.m_depth_ || m_child_index_ != other.m_child_index_ || m_num_children_ != other.m_num_children_) { return false; }
            if (m_num_children_ == 0) { return true; }
            for (int i = 0; i < 4; ++i) {
                if (m_children_[i] == nullptr && other.m_children_[i] == nullptr) { continue; }
                if (m_children_[i] == nullptr || other.m_children_[i] == nullptr) { return false; }
                if (!(*m_children_[i] == *other.m_children_[i])) { return false; }
            }
            return true;
        }

        bool
        operator!=(const AbstractQuadtreeNode &other) const {
            return !(*this == other);
        }

        //-- children

        inline void
        AllocateChildrenPtr() {
            if (m_children_ != nullptr) { return; }
            m_children_ = new AbstractQuadtreeNode *[4];
            for (int i = 0; i < 4; ++i) { m_children_[i] = nullptr; }
        }

        inline void
        DeleteChildrenPtr() {
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
            if (m_children_ == nullptr) { return false; }
            ERL_DEBUG_ASSERT(index < 4, "Index must be in [0, 3], but got %u.", index);
            return m_children_[index] != nullptr;
        }

        template<typename Derived>
        [[nodiscard]] inline Derived *
        CreateChild(uint32_t child_index) {
            ERL_DEBUG_ASSERT(child_index < 4, "Child index must be in [0, 3], but got %u.", child_index);
            ERL_DEBUG_ASSERT(m_children_[child_index] == nullptr, "Child %u already exists.", child_index);
            AbstractQuadtreeNode *child = AllocateChildPtr(child_index);
            ERL_DEBUG_ASSERT(typeid(*child) == typeid(Derived), "AllocateChildPtr(child_index) must return a pointer of the correct type.");
            ERL_DEBUG_ASSERT(child->m_depth_ == m_depth_ + 1, "Child depth is not set correctly by AllocateChildPtr(child_index).");
            m_children_[child_index] = child;
            m_num_children_++;
            return static_cast<Derived *>(child);
        }

        inline void
        RemoveChild(uint32_t child_index) {
            ERL_DEBUG_ASSERT(child_index < 4, "Child index must be in [0, 3], but got %u.", child_index);
            ERL_DEBUG_ASSERT(m_children_[child_index] != nullptr, "Child %u does not exist.", child_index);
            delete m_children_[child_index];
            m_children_[child_index] = nullptr;
            m_num_children_--;
        }

        template<typename Derived>
        inline Derived *
        GetChild(uint32_t child_index) {
            ERL_DEBUG_ASSERT(child_index < 4, "Child index must be in [0, 3], but got %u.", child_index);
            return static_cast<Derived *>(m_children_[child_index]);
        }

        template<typename Derived>
        [[nodiscard]] inline const Derived *
        GetChild(uint32_t child_index) const {
            ERL_DEBUG_ASSERT(child_index < 4, "Child index must be in [0, 3], but got %u.", child_index);
            return static_cast<const Derived *>(m_children_[child_index]);
        }

        [[nodiscard]] virtual inline bool
        AllowMerge(const AbstractQuadtreeNode *other) const {
            (void) other;
            return m_num_children_ == 0;
        }

        virtual inline void
        Prune() {
            ERL_DEBUG_ASSERT(m_num_children_ == 4, "Prune() can only be called when all children are present.");
            for (int i = 0; i < 4; ++i) {
                delete m_children_[i];
                m_children_[i] = nullptr;
            }
            m_num_children_ = 0;
        }

        virtual inline void
        Expand() {
            ERL_DEBUG_ASSERT(m_num_children_ == 0, "Expand() can only be called when no children are present.");
            if (m_children_ == nullptr) { m_children_ = new AbstractQuadtreeNode *[4]; }
            for (int i = 0; i < 4; ++i) { m_children_[i] = AllocateChildPtr(i); }
            m_num_children_ = 4;
        }

    private:
        virtual AbstractQuadtreeNode *
        AllocateChildPtr(uint32_t child_index) = 0;
    };
}  // namespace erl::geometry
