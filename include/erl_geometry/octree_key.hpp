#pragma once

#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>

#include <cstdint>
#include <vector>

namespace erl::geometry {

    /**
     * OctreeKey is a simple class that represents a key for an octree node. It is a 3D vector of uint16_t.
     * Each element counts the number of cells from the origin as discrete address of a voxel.
     */
    class OctreeKey {
    public:
        using KeyType = uint16_t;

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
            [[nodiscard]] std::size_t
            operator()(const OctreeKey& key) const {
                return (static_cast<std::size_t>(key.m_k_[0]) << 32) | (static_cast<std::size_t>(key.m_k_[1]) << 16) | static_cast<std::size_t>(key.m_k_[2]);
            }
        };

        OctreeKey() = default;

        OctreeKey(const KeyType a, const KeyType b, const KeyType c)
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

        [[nodiscard]] bool
        operator==(const OctreeKey& other) const {
            return m_k_[0] == other.m_k_[0] && m_k_[1] == other.m_k_[1] && m_k_[2] == other.m_k_[2];
        }

        [[nodiscard]] bool
        operator!=(const OctreeKey& other) const {
            return m_k_[0] != other.m_k_[0] || m_k_[1] != other.m_k_[1] || m_k_[2] != other.m_k_[2];
        }

        KeyType&
        operator[](const uint32_t i) {
            return m_k_[i];
        }

        [[nodiscard]] const KeyType&
        operator[](const uint32_t i) const {
            return m_k_[i];
        }

        [[nodiscard]] bool
        operator<(const OctreeKey& other) const {
            return m_k_[0] < other.m_k_[0] ||                                //
                   (m_k_[0] == other.m_k_[0] && (m_k_[1] < other.m_k_[1] ||  //
                                                 (m_k_[1] == other.m_k_[1] && m_k_[2] < other.m_k_[2])));
        }

        [[nodiscard]] bool
        operator<=(const OctreeKey& other) const {
            return m_k_[0] < other.m_k_[0] ||                                //
                   (m_k_[0] == other.m_k_[0] && (m_k_[1] < other.m_k_[1] ||  //
                                                 (m_k_[1] == other.m_k_[1] && m_k_[2] <= other.m_k_[2])));
        }

        [[nodiscard]] bool
        operator>(const OctreeKey& other) const {
            return m_k_[0] > other.m_k_[0] ||                                //
                   (m_k_[0] == other.m_k_[0] && (m_k_[1] > other.m_k_[1] ||  //
                                                 (m_k_[1] == other.m_k_[1] && m_k_[2] > other.m_k_[2])));
        }

        [[nodiscard]] bool
        operator>=(const OctreeKey& other) const {
            return m_k_[0] > other.m_k_[0] ||                                //
                   (m_k_[0] == other.m_k_[0] && (m_k_[1] > other.m_k_[1] ||  //
                                                 (m_k_[1] == other.m_k_[1] && m_k_[2] >= other.m_k_[2])));
        }

        [[nodiscard]] explicit
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
        static void
        ComputeChildKey(const uint32_t pos, const KeyType center_offset_key, const OctreeKey& parent_key, OctreeKey& child_key) {
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
        static int
        ComputeChildIndex(const OctreeKey& key, const uint32_t level) {
            int pos = 0;
            const KeyType mask = 1 << level;
            if (key.m_k_[0] & mask) { pos += 1; }
            if (key.m_k_[1] & mask) { pos += 2; }
            if (key.m_k_[2] & mask) { pos += 4; }
            return pos;
        }

        static bool
        KeyInAabb(const OctreeKey& key, const KeyType center_offset_key, const OctreeKey& aabb_min_key, const OctreeKey& aabb_max_key) {
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
    using OctreeKeySet = absl::flat_hash_set<OctreeKey>;
    using OctreeKeyVectorMap = absl::flat_hash_map<OctreeKey, std::vector<long>>;
    using OctreeKeyVector = std::vector<OctreeKey>;

    /**
     * Data structure to efficiently track changed nodes.
     */
    using OctreeKeyBoolMap = absl::flat_hash_map<OctreeKey, bool>;
    using OctreeKeyRay = std::vector<OctreeKey>;
}  // namespace erl::geometry
