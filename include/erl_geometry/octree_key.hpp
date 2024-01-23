#pragma once

#include <cstdint>
#include <unordered_set>
#include <unordered_map>
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

        struct KeyHash {
            [[nodiscard]] inline std::size_t
            operator()(const OctreeKey& key) const {
                return (std::size_t(key.m_k_[0]) << 32) | (std::size_t(key.m_k_[1]) << 16) | std::size_t(key.m_k_[2]);
            }
        };

        OctreeKey() = default;

        OctreeKey(KeyType a, KeyType b, KeyType c)
            : m_k_{a, b, c} {}

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

        /**
         * Compute the key of a child node from the key of its parent node and the index of the child node.
         * @param pos index of child node (0..7)
         * @param center_offset_key
         * @param parent_key
         * @param child_key
         */
        inline static void
        ComputeChildKey(unsigned int pos, OctreeKey::KeyType center_offset_key, const OctreeKey& parent_key, OctreeKey& child_key) {
            child_key[0] = parent_key[0] + ((pos & 1) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
            child_key[1] = parent_key[1] + ((pos & 2) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
            child_key[2] = parent_key[2] + ((pos & 4) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
        }

        /**
         * Compute child index (0..7) from key at given level.
         * @param key
         * @param level level=0 means the leaf level
         * @return
         */
        inline static uint8_t
        ComputeChildIndex(const OctreeKey& key, unsigned int level) {
            uint8_t pos = 0;
            if (key[0] & (1 << level)) { pos += 1; }
            if (key[1] & (1 << level)) { pos += 2; }
            if (key[2] & (1 << level)) { pos += 4; }
            return pos;
        }

        inline static bool
        KeyInAabb(const OctreeKey& key, OctreeKey::KeyType center_offset_key, const OctreeKey& aabb_min_key, const OctreeKey& aabb_max_key) {
            return (aabb_min_key[0] <= (key[0] + center_offset_key)) && (aabb_min_key[1] <= (key[1] + center_offset_key)) &&
                   (aabb_min_key[2] <= (key[2] + center_offset_key)) && (aabb_max_key[0] >= (key[0] - center_offset_key)) &&
                   (aabb_max_key[1] >= (key[1] - center_offset_key)) && (aabb_max_key[2] >= (key[2] - center_offset_key));
        }

    private:
        KeyType m_k_[3] = {0, 0, 0};
    };

    /**
     * Data structure to efficiently compute the nodes to update from a scan insertion using a hash set.
     */
    typedef std::unordered_set<OctreeKey, OctreeKey::KeyHash> OctreeKeySet;

    /**
     * Data structure to efficiently track changed nodes.
     */
    typedef std::unordered_map<OctreeKey, bool, OctreeKey::KeyHash> OctreeKeyBoolMap;

    /**
     * Data structure for efficient ray casting.
     */
    class OctreeKeyRay {
    public:
        typedef std::vector<OctreeKey>::iterator Iterator;
        typedef std::vector<OctreeKey>::const_iterator ConstIterator;
        typedef std::vector<OctreeKey>::reverse_iterator ReverseIterator;

    private:
        std::vector<OctreeKey> m_ray_ = {};
        Iterator m_end_of_ray_ = m_ray_.begin();

    public:
        OctreeKeyRay() {
            m_ray_.resize(100000);
            Reset();
        }

        OctreeKeyRay(const OctreeKeyRay& other) {
            m_ray_ = other.m_ray_;
            auto size = other.end() - other.begin();
            m_end_of_ray_ = m_ray_.begin() + size;
        }

        inline void
        Reset() {
            m_end_of_ray_ = begin();
        }

        inline void
        AddKey(const OctreeKey& k) {
            ERL_ASSERTM(m_end_of_ray_ != m_ray_.end(), "Ray is full.");
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

        [[nodiscard]] inline OctreeKey
        operator[](int idx) const {
            ERL_ASSERTM(idx >= 0 && idx < int(size()), "Index out of bounds.");
            return m_ray_[idx];
        }
    };

}  // namespace erl::geometry
