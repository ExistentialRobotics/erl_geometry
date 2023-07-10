#pragma once

#include "erl_common/assert.hpp"
#include "erl_common/grid_map_info.hpp"
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace erl::geometry {
    class AbstractQuadtreeNode {
    public:
        AbstractQuadtreeNode() = default;

        virtual ~AbstractQuadtreeNode() = default;

        //-- children

        inline void
        AllocateChildrenPtr() {
            if (!m_children_.empty()) { return; }
            m_children_.resize(4, nullptr);
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
            ERL_DEBUG_ASSERT(index < 4, "Index must be in [0, 3], but got %u.\n", index);
            return m_children_[index] != nullptr;
        }

        void
        SetChild(std::shared_ptr<AbstractQuadtreeNode> child, unsigned int index) {
            ERL_DEBUG_ASSERT(index < 4, "Index must be in [0, 3], but got %u.\n", index);
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

        template<typename T = AbstractQuadtreeNode>
        std::shared_ptr<T>
        GetChild(unsigned int index) {
            ERL_DEBUG_ASSERT(index < 4, "Index must be in [0, 3], but got %u.\n", index);
            ERL_DEBUG_ASSERT(m_children_.size() == 4, "Incorrect number of child ptrs. Get %zu instead of 4.\n", m_children_.size());
            return std::static_pointer_cast<T>(m_children_[index]);
        }

        template<typename T = AbstractQuadtreeNode>
        [[nodiscard]] std::shared_ptr<const T>
        GetChild(unsigned int index) const {
            ERL_DEBUG_ASSERT(index < 4, "Index must be in [0, 3], but got %u.\n", index);
            return std::static_pointer_cast<const T>(m_children_[index]);
        }

        //-- file IO
        virtual std::istream&
        ReadData(std::istream& s) = 0;

        virtual std::ostream&
        WriteData(std::ostream& s) const = 0;

    protected:
        std::vector<std::shared_ptr<AbstractQuadtreeNode>> m_children_ = {};
        unsigned int m_num_children_ = 0;
    };
}  // namespace erl::geometry
