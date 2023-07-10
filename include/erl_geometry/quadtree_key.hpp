#pragma once

#include <cstdint>
#include <unordered_set>
#include <unordered_map>
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

        struct KeyHash {
            [[nodiscard]] inline std::size_t
            operator()(const QuadtreeKey& key) const {
                return (std::size_t(key[0]) << 16) | std::size_t(key[1]);
            }
        };

        QuadtreeKey() = default;

        QuadtreeKey(KeyType a, KeyType b)
            : m_k_{a, b} {}

        [[nodiscard]] bool
        operator==(const QuadtreeKey& other) const {
            return m_k_[0] == other.m_k_[0] && m_k_[1] == other.m_k_[1];
        }

        [[nodiscard]] bool
        operator!=(const QuadtreeKey& other) const {
            return m_k_[0] != other.m_k_[0] || m_k_[1] != other.m_k_[1];
        }

        KeyType&
        operator[](unsigned int i) {
            return m_k_[i];
        }

        [[nodiscard]] const KeyType&
        operator[](unsigned int i) const {
            return m_k_[i];
        }

    private:
        KeyType m_k_[2] = {0, 0};
    };

    /**
     * SurfaceData structure to efficiently compute the nodes to update from a scan insertion using a hash set.
     */
    typedef std::unordered_set<QuadtreeKey, QuadtreeKey::KeyHash> QuadtreeKeySet;

    /**
     * SurfaceData structure to efficiently track changed nodes.
     */
    typedef std::unordered_map<QuadtreeKey, bool, QuadtreeKey::KeyHash> QuadtreeKeyBoolMap;

    /**
     * SurfaceData structure for efficient ray casting.
     */
    class QuadtreeKeyRay {
    public:
        typedef std::vector<QuadtreeKey>::iterator Iterator;
        typedef std::vector<QuadtreeKey>::const_iterator ConstIterator;
        typedef std::vector<QuadtreeKey>::reverse_iterator ReverseIterator;

    private:
        std::vector<QuadtreeKey> m_ray_ = {};
        Iterator m_end_of_ray_ = m_ray_.begin();

    public:
        QuadtreeKeyRay() {
            m_ray_.resize(100000);
            Reset();
        }

        QuadtreeKeyRay(const QuadtreeKeyRay& other) {
            m_ray_ = other.m_ray_;
            auto size = other.end() - other.begin();
            m_end_of_ray_ = m_ray_.begin() + size;
        }

        inline void
        Reset() {
            m_end_of_ray_ = begin();
        }

        inline void
        AddKey(const QuadtreeKey& k) {
            ERL_ASSERTM(m_end_of_ray_ != m_ray_.end(), "Ray is full.\n");
            *m_end_of_ray_ = k;
            ++m_end_of_ray_;
        }

        [[nodiscard]] auto
        size() const {
            return (unsigned long) (m_end_of_ray_ - m_ray_.begin());
        }

        [[nodiscard]] auto
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
    };

    /**
     * Compute the key of a child node from the key of its parent node and the index of the child node.
     * @param pos index of child node (0..3)
     * @param center_offset_key
     * @param parent_key
     * @param child_key
     * @return
     */
    inline void
    ComputeChildKey(unsigned int pos, QuadtreeKey::KeyType center_offset_key, const QuadtreeKey& parent_key, QuadtreeKey& child_key) {
        child_key[0] = parent_key[0] + ((pos & 1) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
        child_key[1] = parent_key[1] + ((pos & 2) ? center_offset_key : -center_offset_key - (center_offset_key ? 0 : 1));
    }

    /**
     * Compute child index (0..3) from key at given level.
     * @param key
     * @param level level=0 means the leaf level
     * @return
     */
    inline uint8_t
    ComputeChildIndex(const QuadtreeKey& key, unsigned int level) {
        uint8_t pos = 0;
        if (key[0] & (1 << level)) { pos += 1; }
        if (key[1] & (1 << level)) { pos += 2; }
        return pos;
    }

    inline bool
    KeyInAabb(const QuadtreeKey& key, QuadtreeKey::KeyType center_offset_key, const QuadtreeKey& aabb_min_key, const QuadtreeKey& aabb_max_key) {
        return (aabb_min_key[0] <= (key[0] + center_offset_key)) && (aabb_min_key[1] <= (key[1] + center_offset_key)) &&
               (aabb_max_key[0] >= (key[0] - center_offset_key)) && (aabb_max_key[1] >= (key[1] - center_offset_key));
    }

}  // namespace erl::geometry
