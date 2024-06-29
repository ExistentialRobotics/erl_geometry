#pragma once

#include "erl_common/logging.hpp"
#include "erl_common/string_utils.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>

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
        inline static std::map<std::string, std::function<std::shared_ptr<AbstractQuadtreeNode>(uint32_t, int)>> s_class_id_mapping_ = {};

    public:
        // rules of five: https://www.youtube.com/watch?v=juAZDfsaMvY
        // except for user-defined constructor,
        // always define: destructor, copy constructor, copy assignment, move constructor, move assignment

        AbstractQuadtreeNode() = delete;

        explicit AbstractQuadtreeNode(uint32_t depth, int child_index = -1)
            : m_depth_(depth),
              m_child_index_(child_index) {
            ERL_DEBUG_WARN_ONCE_COND(
                typeid(*this) != typeid(AbstractQuadtreeNode) && s_class_id_mapping_.find(GetNodeType()) == s_class_id_mapping_.end(),
                "Tree type {} not registered, do you forget to use ERL_REGISTER_QUADTREE_NODE({})?",
                GetNodeType(),
                GetNodeType());
        }

        /**
         * copy constructor, deep copy. If you want to do shallow copy, please wrap it in a smart pointer. AbstractOctreeNode uses raw pointers internally and
         * is responsible for memory management. So, shallow copy is impossible, which will lead to double free.
         * @param other
         */
        AbstractQuadtreeNode(const AbstractQuadtreeNode &other);

        // copy assignment
        AbstractQuadtreeNode &
        operator=(const AbstractQuadtreeNode &other);

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

        //-- factory pattern
        [[nodiscard]] std::string
        GetNodeType() const {
            return demangle(typeid(*this).name());
        }

        /**
         * Implemented by derived classes to create a new node of the same type.
         * @return a new node of the same type of node.
         */
        [[nodiscard]] virtual AbstractQuadtreeNode *
        Create(uint32_t depth, int child_index) const = 0;

        static std::shared_ptr<AbstractQuadtreeNode>
        CreateNode(const std::string &node_type, const uint32_t depth, const int child_index) {
            const auto it = s_class_id_mapping_.find(node_type);
            if (it == s_class_id_mapping_.end()) {
                ERL_WARN("Unknown Octree node type: {}", node_type);
                return nullptr;
            }
            return it->second(depth, child_index);
        }

        template<typename Derived>
        static bool
        RegisterNodeType() {
            const std::string node_type = demangle(typeid(Derived).name());
            if (s_class_id_mapping_.find(node_type) != s_class_id_mapping_.end()) {
                ERL_WARN("{} is already registered.", node_type);
                return false;
            }

            s_class_id_mapping_[node_type] = [](uint32_t depth, int child_index) { return std::make_shared<Derived>(depth, child_index); };
            return true;
        }

        /**
         * Deep copy of the node. Used for copy constructor and copy assignment.
         * @return deep copy of the node.
         */
        [[nodiscard]] virtual AbstractQuadtreeNode *
        Clone() const = 0;

        //-- attributes

        [[nodiscard]] const uint32_t &
        GetDepth() const {
            return m_depth_;
        }

        [[nodiscard]] const int &
        GetChildIndex() const {
            return m_child_index_;
        }

        //-- file IO
        virtual std::istream &
        ReadData(std::istream &s) = 0;

        virtual std::ostream &
        WriteData(std::ostream &s) const = 0;

        //-- comparison

        [[nodiscard]] virtual bool
        operator==(const AbstractQuadtreeNode &other) const;

        bool
        operator!=(const AbstractQuadtreeNode &other) const {  // NOLINT(*-no-recursion)
            return !(*this == other);
        }

        //-- children

        void
        AllocateChildrenPtr() {
            if (m_children_ != nullptr) { return; }
            m_children_ = new AbstractQuadtreeNode *[4];
            for (int i = 0; i < 4; ++i) { m_children_[i] = nullptr; }
        }

        void
        DeleteChildrenPtr();

        [[nodiscard]] uint32_t
        GetNumChildren() const {
            return m_num_children_;
        }

        [[nodiscard]] bool
        HasAnyChild() const {
            return m_num_children_ > 0;
        }

        [[nodiscard]] bool
        HasChild(uint32_t index) const {
            if (m_children_ == nullptr) { return false; }
            ERL_DEBUG_ASSERT(index < 4, "Index must be in [0, 3], but got %u.", index);
            return m_children_[index] != nullptr;
        }

        [[nodiscard]] AbstractQuadtreeNode *
        CreateChild(uint32_t child_index) {
            ERL_DEBUG_ASSERT(child_index < 4, "Child index must be in [0, 3], but got %u.", child_index);
            ERL_DEBUG_ASSERT(m_children_[child_index] == nullptr, "Child %u already exists.", child_index);
            AbstractQuadtreeNode *child = this->Create(m_depth_ + 1, static_cast<int>(child_index));
            m_children_[child_index] = child;
            m_num_children_++;
            return child;
        }

        void
        RemoveChild(uint32_t child_index) {
            ERL_DEBUG_ASSERT(child_index < 4, "Child index must be in [0, 3], but got %u.", child_index);
            ERL_DEBUG_ASSERT(m_children_[child_index] != nullptr, "Child %u does not exist.", child_index);
            delete m_children_[child_index];
            m_children_[child_index] = nullptr;
            m_num_children_--;
        }

        template<typename Derived>
        Derived *
        GetChild(uint32_t child_index) {
            ERL_DEBUG_ASSERT(child_index < 4, "Child index must be in [0, 3], but got %u.", child_index);
            return static_cast<Derived *>(m_children_[child_index]);
        }

        template<typename Derived>
        [[nodiscard]] const Derived *
        GetChild(uint32_t child_index) const {
            ERL_DEBUG_ASSERT(child_index < 4, "Child index must be in [0, 3], but got %u.", child_index);
            return static_cast<const Derived *>(m_children_[child_index]);
        }

        [[nodiscard]] virtual bool
        AllowMerge(const AbstractQuadtreeNode *other) const {
            return m_num_children_ == 0 && other->m_num_children_ == 0;
        }

        virtual void
        Prune() {
            ERL_DEBUG_ASSERT(m_num_children_ == 4, "Prune() can only be called when all children are present.");
            for (int i = 0; i < 4; ++i) {
                delete m_children_[i];
                m_children_[i] = nullptr;
            }
            m_num_children_ = 0;
        }

        virtual void
        Expand() {
            ERL_DEBUG_ASSERT(m_num_children_ == 0, "Expand() can only be called when no children are present.");
            if (m_children_ == nullptr) { m_children_ = new AbstractQuadtreeNode *[4]; }
            for (int i = 0; i < 4; ++i) { m_children_[i] = this->Create(m_depth_ + 1, i); }
            m_num_children_ = 4;
        }
    };

#define ERL_REGISTER_QUADTREE_NODE(Derived) inline const volatile bool kRegistered##Derived = erl::geometry::AbstractQuadtreeNode::RegisterNodeType<Derived>()
}  // namespace erl::geometry
