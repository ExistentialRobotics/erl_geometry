#pragma once

#include "erl_common/assert.hpp"

namespace erl::geometry {

    class AbstractOctreeNode {
    public:
        virtual ~AbstractOctreeNode() = default;
    };

    template<typename T>
    class OctreeDataNode : public AbstractOctreeNode {

        template<typename Node, typename Interface>
        friend class OctreeImpl;

    protected:
        AbstractOctreeNode **m_children_ = nullptr;
        int m_num_children_ = 0;
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
              m_num_children_(other.m_num_children_),
              m_value_(other.m_value_) {
            CopyChildren<NodeType>(other);
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

        inline const T &
        GetValue() const {
            return m_value_;
        }

        inline void
        SetValue(T t) {
            m_value_ = t;
        }

        //-- file IO
        virtual std::istream &
        ReadData(std::istream &s) {
            s.read(reinterpret_cast<char *>(&m_value_), sizeof(T));
            return s;
        }

        virtual std::ostream &
        WriteData(std::ostream &s) const {
            s.write(reinterpret_cast<const char *>(&m_value_), sizeof(T));
            return s;
        }

        //-- children

        inline void
        AllocateChildrenPtr() {
            if (HasChildrenPtr()) { return; }
            m_children_ = new AbstractOctreeNode *[8];
            for (int i = 0; i < 8; ++i) { m_children_[i] = nullptr; }
        }

        inline void
        DeleteChildrenPtr() {
            for (int i = 0; i < 8; ++i) {
                if (m_children_[i] != nullptr) { delete m_children_[i]; }
            }
            delete[] m_children_;
            m_children_ = nullptr;
            m_num_children_ = 0;
        }

        template<typename Derived>
        inline void
        DeleteChildrenPtr2() {
            for (int i = 0; i < 8; ++i) { delete static_cast<Derived *>(m_children_[i]); }
            delete[] m_children_;
            m_children_ = nullptr;
            m_num_children_ = 0;
        }

        [[nodiscard]] inline bool
        HasChildrenPtr() const {
            return m_children_ != nullptr;
        }

        [[nodiscard]] inline int
        GetNumChildren() const {
            return m_num_children_;
        }

        [[nodiscard]] inline bool
        HasAnyChild() const {
            return m_num_children_ > 0;
        }

        [[nodiscard]] inline bool
        HasChild(unsigned int index) const {
            if (!m_num_children_) { return false; }
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            return m_children_[index] != nullptr;
        }

        template<typename Derived>
        [[nodiscard]] inline Derived *
        CreateChild(unsigned int index) {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            ERL_DEBUG_ASSERT(m_children_[index] == nullptr, "Child %u already exists.", index);
            auto *child = new Derived();
            m_children_[index] = child;
            m_num_children_++;
            return child;
        }

        inline void
        RemoveChild(unsigned int index) {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            ERL_DEBUG_ASSERT(m_children_[index] != nullptr, "Child %u does not exist.", index);
            delete m_children_[index];
            m_children_[index] = nullptr;
            m_num_children_--;
        }

        template<typename Derived>
        inline Derived *
        GetChild(unsigned int index) {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            return static_cast<Derived *>(m_children_[index]);
        }

        template<typename Derived>
        [[nodiscard]] inline const Derived *
        GetChild(unsigned int index) const {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            return static_cast<const Derived *>(m_children_[index]);
        }

        template<typename Derived>
        void
        CopyChildren(const Derived &other) {
            if (!other.HasChildrenPtr()) { return; }
            AllocateChildrenPtr();
            for (int i = 0; i < 8; ++i) {
                if (other.m_children_[i] == nullptr) { continue; }
                m_children_[i] = new Derived(*static_cast<Derived *>(other.m_children_[i]));
            }
            m_num_children_ = other.m_num_children_;
        }
    };
}  // namespace erl::geometry
