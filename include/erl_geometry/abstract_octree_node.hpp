#pragma once

#include "erl_common/assert.hpp"
#include <memory>
#include <vector>

namespace erl::geometry {
    class AbstractOctreeNode {
    public:
        AbstractOctreeNode() = default;
        virtual ~AbstractOctreeNode() = default;

        //-- children

        inline void
        AllocateChildrenPtr() {
            if (HasChildrenPtr()) { return; }
            m_children_.resize(8, nullptr);
        }

        inline void
        DeleteChildrenPtr() {
            m_children_.clear();
            m_num_children_ = 0;
        }

        [[nodiscard]] inline bool
        HasChildrenPtr() const {
            return !m_children_.empty();
        }

        [[nodiscard]] inline unsigned int
        GetNumChildren() const {
            return m_num_children_;
        }

        [[nodiscard]] inline bool
        HasAnyChild() const {
            return m_num_children_ > 0;
        }

        [[nodiscard]] inline bool
        HasChild(unsigned int index) const {
            if (m_children_.empty()) { return false; }
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            return m_children_[index] != nullptr;
        }

        inline void
        SetChild(std::shared_ptr<AbstractOctreeNode> child, unsigned int index) {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.", index);
            auto& slot = m_children_[index];
            if (slot != nullptr) {
                slot.reset();
                m_num_children_--;
            }
            if (child != nullptr) {
                slot = std::move(child);
                m_num_children_++;
            }
        }

        template<typename T = AbstractOctreeNode>
        inline std::shared_ptr<T>
        GetChild(unsigned int index) {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.\n", index);
            ERL_DEBUG_ASSERT(m_children_.size() == 8, "Incorrect number of child ptrs. Get %zu instead of 8.\n", m_children_.size());
            return std::static_pointer_cast<T>(m_children_[index]);
        }

        template<typename T = AbstractOctreeNode>
        [[nodiscard]] inline std::shared_ptr<const T>
        GetChild(unsigned int index) const {
            ERL_DEBUG_ASSERT(index < 8, "Index must be in [0, 7], but got %u.\n", index);
            return std::static_pointer_cast<const T>(m_children_[index]);
        }

        //-- file IO
        virtual std::istream&
        ReadData(std::istream& s) = 0;

        virtual std::ostream&
        WriteData(std::ostream& s) const = 0;

    protected:
        std::vector<std::shared_ptr<AbstractOctreeNode>> m_children_ = {};
        unsigned int m_num_children_ = 0;
    };
}  // namespace erl::geometry
