#pragma once
#include "occupancy_octree_node.hpp"

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    class SurfaceMappingOctreeNode : public OccupancyOctreeNode {

    public:
        struct SurfaceData {
            Eigen::Vector3d position = {0.0, 0.0, 0.0};
            Eigen::Vector3d normal = {0.0, 0.0, 0.0};
            double var_position = 0.;
            double var_normal = 0.;

            SurfaceData() = default;

            SurfaceData(Eigen::Vector3d position, Eigen::Vector3d normal, const double var_position, const double var_normal)
                : position(std::move(position)),
                  normal(std::move(normal)),
                  var_position(var_position),
                  var_normal(var_normal) {}

            SurfaceData(const SurfaceData &other) = default;
            SurfaceData &
            operator=(const SurfaceData &other) = default;
            SurfaceData(SurfaceData &&other) noexcept = default;
            SurfaceData &
            operator=(SurfaceData &&other) noexcept = default;

            [[nodiscard]] bool
            operator==(const SurfaceData &other) const {
                return position == other.position && normal == other.normal && var_position == other.var_position && var_normal == other.var_normal;
            }

            [[nodiscard]] bool
            operator!=(const SurfaceData &other) const {
                return !(*this == other);
            }
        };

    private:
        std::shared_ptr<SurfaceData> m_data_ = nullptr;

    public:
        explicit SurfaceMappingOctreeNode(const uint32_t depth = 0, const int child_index = -1, const float log_odds = 0)
            : OccupancyOctreeNode(depth, child_index, log_odds) {}

        SurfaceMappingOctreeNode(const SurfaceMappingOctreeNode &other)
            : OccupancyOctreeNode(other) {
            if (other.m_data_ == nullptr) {
                m_data_.reset();
                return;
            }
            m_data_ = std::make_shared<SurfaceData>(*other.m_data_);
        }

        SurfaceMappingOctreeNode &
        operator=(const SurfaceMappingOctreeNode &other) {
            if (this == &other) { return *this; }
            OccupancyOctreeNode::operator=(other);
            if (other.m_data_ == nullptr) {
                m_data_.reset();
                return *this;
            }
            m_data_ = std::make_shared<SurfaceData>(*other.m_data_);
            return *this;
        }

        SurfaceMappingOctreeNode(SurfaceMappingOctreeNode &&other) noexcept = default;

        SurfaceMappingOctreeNode &
        operator=(SurfaceMappingOctreeNode &&other) noexcept = default;

        [[nodiscard]] AbstractOctreeNode *
        Create(const uint32_t depth, const int child_index) const override {
            auto node = new SurfaceMappingOctreeNode(depth, child_index, /*log_odds*/ 0);
            ERL_TRACY_RECORD_ALLOC(node, sizeof(SurfaceMappingOctreeNode));
            return node;
        }

        [[nodiscard]] AbstractOctreeNode *
        Clone() const override {
            auto node = new SurfaceMappingOctreeNode(*this);
            ERL_TRACY_RECORD_ALLOC(node, sizeof(SurfaceMappingOctreeNode));
            return node;
        }

        bool
        operator==(const AbstractOctreeNode &other) const override {
            if (!OccupancyOctreeNode::operator==(other)) { return false; }
            const auto *other_ptr = dynamic_cast<const SurfaceMappingOctreeNode *>(&other);
            if (other_ptr == nullptr) { return false; }
            if (m_data_ == nullptr && other_ptr->m_data_ != nullptr) { return false; }
            if (m_data_ != nullptr && (other_ptr->m_data_ == nullptr || *m_data_ != *other_ptr->m_data_)) { return false; }
            return true;
        }

        void
        SetSurfaceData(Eigen::Vector3d position, Eigen::Vector3d normal, double var_position, double var_normal) {
            m_data_ = std::make_shared<SurfaceData>(std::move(position), std::move(normal), var_position, var_normal);
            ERL_DEBUG_ASSERT(std::abs(m_data_->normal.norm() - 1.0) < 1.e-6, "normal.norm() = {:.6f}", m_data_->normal.norm());
        }

        void
        SetSurfaceData(const std::shared_ptr<SurfaceData> &data) {
            m_data_ = data;
            ERL_DEBUG_ASSERT(std::abs(m_data_->normal.norm() - 1.0) < 1.e-6, "normal.norm() = {:.6f}", m_data_->normal.norm());
        }

        std::shared_ptr<SurfaceData>
        GetSurfaceData() {
            return m_data_;
        }

        void
        ResetSurfaceData() {
            m_data_.reset();
        }

        std::istream &
        ReadData(std::istream &s) override {
            OccupancyOctreeNode::ReadData(s);
            char has_data;
            s.read(&has_data, sizeof(char));
            if (has_data == 0) {
                m_data_.reset();
                return s;
            }
            m_data_ = std::make_shared<SurfaceData>();
            s.read(reinterpret_cast<char *>(m_data_->position.data()), sizeof(double) * 3);
            s.read(reinterpret_cast<char *>(m_data_->normal.data()), sizeof(double) * 3);
            s.read(reinterpret_cast<char *>(&m_data_->var_position), sizeof(double));
            s.read(reinterpret_cast<char *>(&m_data_->var_normal), sizeof(double));
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            OccupancyOctreeNode::WriteData(s);
            if (m_data_ == nullptr) {
                s << static_cast<char>(0);
                return s;
            }
            s << static_cast<char>(1);
            s.write(reinterpret_cast<const char *>(m_data_->position.data()), sizeof(double) * 3);
            s.write(reinterpret_cast<const char *>(m_data_->normal.data()), sizeof(double) * 3);
            s.write(reinterpret_cast<const char *>(&m_data_->var_position), sizeof(double));
            s.write(reinterpret_cast<const char *>(&m_data_->var_normal), sizeof(double));
            return s;
        }
    };

    ERL_REGISTER_OCTREE_NODE(SurfaceMappingOctreeNode);
}  // namespace erl::geometry
