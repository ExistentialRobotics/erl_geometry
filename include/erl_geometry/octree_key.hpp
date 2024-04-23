#pragma once

#include <cstdint>
#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>
#include <vector>
#include "erl_common/assert.hpp"

namespace erl::geometry {

    /**
     * OctreeKey is a simple class that represents a key for an octree node. It is a 3D vector of uint16_t.
     * Each element counts the number of cells from the origin as discrete address of a voxel.
     */
    class OctreeKey {
    public:
        typedef uint16_t KeyType;

    private:
        KeyType m_k_[3] = {0, 0, 0};

    public:
        // Hash function for OctreeKey when used with absl containers.
        template<typename H>
        [[maybe_unused]] friend H
        AbslHashValue(H h, const OctreeKey& key) {
            return H::combine(std::move(h), key.m_k_[0], key.m_k_[1], key.m_k_[2]);
        }

        // Hash function for OctreeKey when used with std hash containers.
        struct [[maybe_unused]] KeyHash {
            [[nodiscard]] inline std::size_t
            operator()(const OctreeKey& key) const {
                return (std::size_t(key.m_k_[0]) << 32) | (std::size_t(key.m_k_[1]) << 16) | std::size_t(key.m_k_[2]);
            }
        };

        OctreeKey() = default;

        OctreeKey(KeyType a, KeyType b, KeyType c)
            : m_k_{a, b, c} {}

        OctreeKey(const OctreeKey& other)
            : m_k_{other.m_k_[0], other.m_k_[1], other.m_k_[2]} {}

        OctreeKey&
        operator=(const OctreeKey& other) {
            if (this == &other) { return *this; }
            m_k_[0] = other.m_k_[0];
            m_k_[1] = other.m_k_[1];
            m_k_[2] = other.m_k_[2];
            return *this;
        }

        OctreeKey(OctreeKey&& other) noexcept
            : m_k_{std::exchange(other.m_k_[0], 0), std::exchange(other.m_k_[1], 0), std::exchange(other.m_k_[2], 0)} {}

        OctreeKey&
        operator=(OctreeKey&& other) noexcept {
            if (this == &other) { return *this; }
            m_k_[0] = std::exchange(other.m_k_[0], 0);
            m_k_[1] = std::exchange(other.m_k_[1], 0);
            m_k_[2] = std::exchange(other.m_k_[2], 0);
            return *this;
        }

        [[nodiscard]] inline bool
        operator==(const OctreeKey& other) const {
            return m_k_[0] == other.m_k_[0] && m_k_[1] == other.m_k_[1] && m_k_[2] == other.m_k_[2];
        }

        [[nodiscard]] inline bool
        operator!=(const OctreeKey& other) const {
            return m_k_[0] != other.m_k_[0] || m_k_[1] != other.m_k_[1] || m_k_[2] != other.m_k_[2];
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
        operator<(const OctreeKey& other) const {
            return m_k_[0] < other.m_k_[0] ||                                //
                   (m_k_[0] == other.m_k_[0] && (m_k_[1] < other.m_k_[1] ||  //
                                                 (m_k_[1] == other.m_k_[1] && m_k_[2] < other.m_k_[2])));
        }

        [[nodiscard]] inline bool
        operator<=(const OctreeKey& other) const {
            return m_k_[0] < other.m_k_[0] ||                                //
                   (m_k_[0] == other.m_k_[0] && (m_k_[1] < other.m_k_[1] ||  //
                                                 (m_k_[1] == other.m_k_[1] && m_k_[2] <= other.m_k_[2])));
        }

        [[nodiscard]] inline bool
        operator>(const OctreeKey& other) const {
            return m_k_[0] > other.m_k_[0] ||                                //
                   (m_k_[0] == other.m_k_[0] && (m_k_[1] > other.m_k_[1] ||  //
                                                 (m_k_[1] == other.m_k_[1] && m_k_[2] > other.m_k_[2])));
        }

        [[nodiscard]] inline bool
        operator>=(const OctreeKey& other) const {
            return m_k_[0] > other.m_k_[0] ||                                //
                   (m_k_[0] == other.m_k_[0] && (m_k_[1] > other.m_k_[1] ||  //
                                                 (m_k_[1] == other.m_k_[1] && m_k_[2] >= other.m_k_[2])));
        }

        [[nodiscard]] inline explicit
        operator std::string() const {
            return std::to_string(m_k_[0]) + "," + std::to_string(m_k_[1]) + "," + std::to_string(m_k_[2]);
        }

        /**
         * Compute the key of a child node from the key of its parent node and the index of the child node.
         * @param pos index of child node (0..7)
         * @param center_offset_key
         * @param parent_key
         * @param child_key
         */
        inline static void
        ComputeChildKey(unsigned int pos, OctreeKey::KeyType center_offset_key, const OctreeKey& parent_key, OctreeKey& child_key) {
            child_key.m_k_[0] = parent_key.m_k_[0] + ((pos & 1) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
            child_key.m_k_[1] = parent_key.m_k_[1] + ((pos & 2) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
            child_key.m_k_[2] = parent_key.m_k_[2] + ((pos & 4) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
        }

        /**
         * Compute child index (0..7) from key at given level.
         * @param key
         * @param level level=0 means the leaf level
         * @return
         */
        inline static int
        ComputeChildIndex(const OctreeKey& key, uint32_t level) {
            int pos = 0;
            OctreeKey::KeyType mask = 1 << level;
            if (key.m_k_[0] & mask) { pos += 1; }
            if (key.m_k_[1] & mask) { pos += 2; }
            if (key.m_k_[2] & mask) { pos += 4; }
            return pos;
        }

        inline static bool
        KeyInAabb(const OctreeKey& key, OctreeKey::KeyType center_offset_key, const OctreeKey& aabb_min_key, const OctreeKey& aabb_max_key) {
            return (aabb_min_key.m_k_[0] <= (key.m_k_[0] + center_offset_key)) &&  //
                   (aabb_min_key.m_k_[1] <= (key.m_k_[1] + center_offset_key)) &&  //
                   (aabb_min_key.m_k_[2] <= (key.m_k_[2] + center_offset_key)) &&  //
                   (aabb_max_key.m_k_[0] >= (key.m_k_[0] - center_offset_key)) &&  //
                   (aabb_max_key.m_k_[1] >= (key.m_k_[1] - center_offset_key)) &&  //
                   (aabb_max_key.m_k_[2] >= (key.m_k_[2] - center_offset_key));
        }
    };

    /**
     * Data structure to efficiently compute the nodes to update from a scan insertion using a hash set.
     */
    typedef absl::flat_hash_set<OctreeKey> OctreeKeySet;
    typedef absl::flat_hash_map<OctreeKey, std::vector<long>> OctreeKeyVectorMap;
    typedef std::vector<OctreeKey> OctreeKeyVector;

    /**
     * Data structure to efficiently track changed nodes.
     */
    typedef absl::flat_hash_map<OctreeKey, bool> OctreeKeyBoolMap;

    /**
     * Data structure for efficient ray casting.
     */
    class OctreeKeyRay {
    public:
        typedef OctreeKeyVector::iterator Iterator;
        typedef OctreeKeyVector::const_iterator ConstIterator;
        typedef OctreeKeyVector::reverse_iterator ReverseIterator;

    private:
        OctreeKeyVector m_ray_ = {};
        Iterator m_end_of_ray_ = m_ray_.begin();

    public:
        OctreeKeyRay() {
            m_ray_.resize(100000);
            Reset();
        }

        OctreeKeyRay(const OctreeKeyRay& other)
            : m_ray_(other.m_ray_),
              m_end_of_ray_(m_ray_.begin() + (other.end() - other.begin())) {}

        inline void
        Reset() {
            m_end_of_ray_ = begin();
        }

        inline void
        AddKey(const OctreeKey& k) {
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

        [[nodiscard]] inline const OctreeKey&
        operator[](int idx) const {
            ERL_DEBUG_ASSERT(idx >= 0 && idx < int(size()), "Index out of bounds.");
            return m_ray_[idx];
        }
    };

}  // namespace erl::geometry
