#pragma once

#include <cstdint>
#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>
#include <vector>
#include "erl_common/assert.hpp"

namespace erl::geometry {

    /**
     * QuadtreeKey is a simple class that represents a key for a quadtree node. It is a 2D vector of uint16_t.
     * Each element counts the number of cells from the origin as discrete address of a voxel.
     */
    class QuadtreeKey {
    public:
        typedef uint16_t KeyType;

    private:
        KeyType m_k_[2] = {0, 0};

    public:
        // Hash function for OctreeKey when used with absl containers.
        template<typename H>
        [[maybe_unused]] friend H
        AbslHashValue(H h, const QuadtreeKey& key) {
            return H::combine(std::move(h), key.m_k_[0], key.m_k_[1]);
        }

        // Hash function for OctreeKey when used with std hash containers.
        struct KeyHash {
            [[nodiscard]] inline std::size_t
            operator()(const QuadtreeKey& key) const {
                return (std::size_t(key.m_k_[0]) << 16) | std::size_t(key.m_k_[1]);
            }
        };

        QuadtreeKey() = default;

        QuadtreeKey(KeyType a, KeyType b)
            : m_k_{a, b} {}

        QuadtreeKey(const QuadtreeKey& other)
            : m_k_{other.m_k_[0], other.m_k_[1]} {}

        QuadtreeKey&
        operator=(const QuadtreeKey& other) {
            if (this == &other) { return *this; }
            m_k_[0] = other.m_k_[0];
            m_k_[1] = other.m_k_[1];
            return *this;
        }

        QuadtreeKey(QuadtreeKey&& other) noexcept
            : m_k_{std::exchange(other.m_k_[0], 0), std::exchange(other.m_k_[1], 0)} {}

        QuadtreeKey&
        operator=(QuadtreeKey&& other) noexcept {
            if (this == &other) { return *this; }
            m_k_[0] = std::exchange(other.m_k_[0], 0);
            m_k_[1] = std::exchange(other.m_k_[1], 0);
            return *this;
        }

        [[nodiscard]] inline bool
        operator==(const QuadtreeKey& other) const {
            return m_k_[0] == other.m_k_[0] && m_k_[1] == other.m_k_[1];
        }

        [[nodiscard]] inline bool
        operator!=(const QuadtreeKey& other) const {
            return m_k_[0] != other.m_k_[0] || m_k_[1] != other.m_k_[1];
        }

        inline KeyType&
        operator[](unsigned int i) {
            return m_k_[i];
        }

        [[nodiscard]] inline const KeyType&
        operator[](unsigned int i) const {
            return m_k_[i];
        }

        [[nodiscard]] inline bool
        operator<(const QuadtreeKey& other) const {
            return m_k_[0] < other.m_k_[0] || (m_k_[0] == other.m_k_[0] && m_k_[1] < other.m_k_[1]);
        }

        [[nodiscard]] inline bool
        operator<=(const QuadtreeKey& other) const {
            return m_k_[0] < other.m_k_[0] || (m_k_[0] == other.m_k_[0] && m_k_[1] <= other.m_k_[1]);
        }

        [[nodiscard]] inline bool
        operator>(const QuadtreeKey& other) const {
            return m_k_[0] > other.m_k_[0] || (m_k_[0] == other.m_k_[0] && m_k_[1] > other.m_k_[1]);
        }

        [[nodiscard]] inline bool
        operator>=(const QuadtreeKey& other) const {
            return m_k_[0] > other.m_k_[0] || (m_k_[0] == other.m_k_[0] && m_k_[1] >= other.m_k_[1]);
        }

        [[nodiscard]] inline explicit
        operator std::string() const {
            return std::to_string(m_k_[0]) + "," + std::to_string(m_k_[1]);
        }

        /**
         * Compute the key of a child node from the key of its parent node and the index of the child node.
         * @param pos index of child node (0..3)
         * @param center_offset_key
         * @param parent_key
         * @param child_key
         */
        inline static void
        ComputeChildKey(unsigned int pos, QuadtreeKey::KeyType center_offset_key, const QuadtreeKey& parent_key, QuadtreeKey& child_key) {
            child_key.m_k_[0] = parent_key.m_k_[0] + ((pos & 1) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
            child_key.m_k_[1] = parent_key.m_k_[1] + ((pos & 2) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
        }

        /**
         * Compute child index (0..3) from key at given level.
         * @param key
         * @param level level=0 means the leaf level
         * @return
         */
        inline static int
        ComputeChildIndex(const QuadtreeKey& key, unsigned int level) {
            int pos = 0;
            QuadtreeKey::KeyType mask = 1 << level;
            if (key.m_k_[0] & mask) { pos += 1; }
            if (key.m_k_[1] & mask) { pos += 2; }
            return pos;
        }

        inline static bool
        KeyInAabb(const QuadtreeKey& key, QuadtreeKey::KeyType center_offset_key, const QuadtreeKey& aabb_min_key, const QuadtreeKey& aabb_max_key) {
            return (aabb_min_key[0] <= (key[0] + center_offset_key)) &&  //
                   (aabb_min_key[1] <= (key[1] + center_offset_key)) &&  //
                   (aabb_max_key[0] >= (key[0] - center_offset_key)) &&  //
                   (aabb_max_key[1] >= (key[1] - center_offset_key));
        }
    };

    /**
     * Data structure to efficiently compute the nodes to update from a scan insertion using a hash set.
     */
    typedef absl::flat_hash_set<QuadtreeKey> QuadtreeKeySet;
    typedef absl::flat_hash_map<QuadtreeKey, std::vector<long>> QuadtreeKeyVectorMap;
    typedef std::vector<QuadtreeKey> QuadtreeKeyVector;

    /**
     * Data structure to efficiently track changed nodes.
     */
    typedef absl::flat_hash_map<QuadtreeKey, bool> QuadtreeKeyBoolMap;

    /**
     * Data structure for efficient ray casting.
     */
    class QuadtreeKeyRay {
    public:
        typedef QuadtreeKeyVector::iterator Iterator;
        typedef QuadtreeKeyVector::const_iterator ConstIterator;
        typedef QuadtreeKeyVector::reverse_iterator ReverseIterator;

    private:
        QuadtreeKeyVector m_ray_ = {};
        Iterator m_end_of_ray_ = m_ray_.begin();

    public:
        QuadtreeKeyRay() {
            m_ray_.resize(100000);
            Reset();
        }

        QuadtreeKeyRay(const QuadtreeKeyRay& other)
            : m_ray_(other.m_ray_),
              m_end_of_ray_(m_ray_.begin() + (other.end() - other.begin())) {}

        inline void
        Reset() {
            m_end_of_ray_ = begin();
        }

        inline void
        AddKey(const QuadtreeKey& k) {
            ERL_DEBUG_ASSERT(m_end_of_ray_ != m_ray_.end(), "Ray is full.");
            *m_end_of_ray_ = k;
            ++m_end_of_ray_;
        }

        [[nodiscard]] inline auto
        size() const {
            return (unsigned long) (m_end_of_ray_ - m_ray_.begin());
        }

        [[nodiscard]] inline auto
        capacity() const {
            return m_ray_.size();
        }

        inline Iterator
        begin() {
            return m_ray_.begin();
        }

        inline Iterator
        end() {
            return m_end_of_ray_;
        }

        [[nodiscard]] inline ConstIterator
        begin() const {
            return m_ray_.begin();
        }

        [[nodiscard]] inline ConstIterator
        end() const {
            return m_end_of_ray_;
        }

        inline ReverseIterator
        rbegin() {
            return ReverseIterator(m_end_of_ray_);
        }

        inline ReverseIterator
        rend() {
            return m_ray_.rend();
        }

        [[nodiscard]] inline const QuadtreeKey&
        operator[](int idx) const {
            ERL_DEBUG_ASSERT(idx >= 0 && idx < int(size()), "Index out of bounds.");
            return m_ray_[idx];
        }
    };

}  // namespace erl::geometry
