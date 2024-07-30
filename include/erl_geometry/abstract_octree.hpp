#pragma once

#include "aabb.hpp"
#include "abstract_octree_node.hpp"
#include "nd_tree_setting.hpp"

#include "erl_common/string_utils.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace erl::geometry {

    /**
     * AbstractOctree is a base class for all octree implementations. It provides a common interface for factory pattern and file I/O.
     */
    class AbstractOctree {
        std::shared_ptr<NdTreeSetting> m_setting_ = std::make_shared<NdTreeSetting>();

    protected:
        inline static std::map<std::string, std::function<std::shared_ptr<AbstractOctree>(const std::shared_ptr<NdTreeSetting>&)>> s_class_id_mapping_ = {};

    public:
        AbstractOctree() = delete;  // no default constructor

        explicit AbstractOctree(std::shared_ptr<NdTreeSetting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
            ERL_DEBUG_WARN_ONCE_COND(
                typeid(*this) != typeid(AbstractOctree) && s_class_id_mapping_.find(GetTreeType()) == s_class_id_mapping_.end(),
                "Tree type {} not registered, do you forget to use ERL_REGISTER_OCTREE({})?",
                GetTreeType(),
                GetTreeType());
        }

        AbstractOctree(const AbstractOctree& other) = default;
        AbstractOctree&
        operator=(const AbstractOctree& other) = default;
        AbstractOctree(AbstractOctree&& other) = default;
        AbstractOctree&
        operator=(AbstractOctree&& other) = default;

        virtual ~AbstractOctree() = default;

        //-- factory pattern
        /**
         * returns actual class name as string for identification
         * @return The type of the tree.
         */
        [[nodiscard]] std::string
        GetTreeType() const {
            return demangle(typeid(*this).name());
        }

        /**
         * Implemented by derived classes to create a new tree of the same type.
         * @return A new tree of the same type.
         */
        [[nodiscard]] virtual std::shared_ptr<AbstractOctree>
        Create(const std::shared_ptr<NdTreeSetting>& setting) const = 0;

        /**
         * Create a new tree of the given type.
         * @param tree_id
         * @param setting
         * @return
         */
        static std::shared_ptr<AbstractOctree>
        CreateTree(const std::string& tree_id, const std::shared_ptr<NdTreeSetting>& setting);

        template<typename Derived>
        static std::enable_if_t<std::is_base_of_v<AbstractOctree, Derived>, bool>
        RegisterTreeType(const std::string& tree_type) {
            if (s_class_id_mapping_.find(tree_type) != s_class_id_mapping_.end()) {
                ERL_WARN("{} is already registered.", tree_type);
                return false;
            }

            s_class_id_mapping_[tree_type] = [](const std::shared_ptr<NdTreeSetting>& setting) {
                auto tree_setting = std::dynamic_pointer_cast<typename Derived::Setting>(setting);
                if (tree_setting == nullptr) { tree_setting = std::make_shared<typename Derived::Setting>(); }
                return std::make_shared<Derived>(tree_setting);
            };
            ERL_DEBUG("{} is registered.", tree_type);
            return true;
        }

        //-- setting
        /**
         * Get the setting of the tree.
         * @tparam T The type of the setting.
         * @return
         */
        template<typename T>
        std::shared_ptr<T>
        GetSetting() const {
            return std::reinterpret_pointer_cast<T>(m_setting_);
        }

        /**
         * This function should be called when the setting is changed.
         */
        virtual void
        ApplySetting() = 0;

        [[nodiscard]] bool
        ReadSetting(std::istream& s) const {
            std::streamsize len;
            s.read(reinterpret_cast<char*>(&len), sizeof(std::size_t));
            std::string yaml_str(len, '\0');
            s.read(yaml_str.data(), len);
            return m_setting_->FromYamlString(yaml_str);
        }

        void
        WriteSetting(std::ostream& s) const {
            const std::string yaml_str = m_setting_->AsYamlString() + "\n";  // add newline to separate from data
            const auto len = static_cast<std::streamsize>(yaml_str.size());
            s.write(reinterpret_cast<const char*>(&len), sizeof(std::size_t));
            s.write(yaml_str.data(), len);
        }

        //-- comparison
        [[nodiscard]] virtual bool
        operator==(const AbstractOctree& other) const = 0;

        [[nodiscard]] bool
        operator!=(const AbstractOctree& other) const {
            return !(*this == other);
        }

        //-- get tree information
        [[nodiscard]] uint32_t
        GetTreeDepth() const {
            return m_setting_->tree_depth;
        }

        [[nodiscard]] double
        GetResolution() const {
            return m_setting_->resolution;
        }

        [[nodiscard]] virtual std::size_t
        GetSize() const = 0;
        [[maybe_unused]] [[nodiscard]] virtual std::size_t
        GetMemoryUsage() const = 0;
        [[maybe_unused]] [[nodiscard]] virtual std::size_t
        GetMemoryUsagePerNode() const = 0;

        Eigen::Vector3d
        GetMetricMin() {
            Eigen::Vector3d min;
            GetMetricMin(min.x(), min.y(), min.z());
            return min;
        }

        [[nodiscard]] Eigen::Vector3d
        GetMetricMin() const {
            Eigen::Vector3d min;
            GetMetricMin(min.x(), min.y(), min.z());
            return min;
        }

        void
        GetMetricMin(Eigen::Vector3d& min) {
            GetMetricMin(min.x(), min.y(), min.z());
        }

        void
        GetMetricMin(Eigen::Vector3d& min) const {
            GetMetricMin(min.x(), min.y(), min.z());
        }

        virtual void
        GetMetricMin(double& x, double& y, double& z) = 0;
        virtual void
        GetMetricMin(double& x, double& y, double& z) const = 0;

        Eigen::Vector3d
        GetMetricMax() {
            Eigen::Vector3d max;
            GetMetricMax(max.x(), max.y(), max.z());
            return max;
        }

        [[nodiscard]] Eigen::Vector3d
        GetMetricMax() const {
            Eigen::Vector3d max;
            GetMetricMax(max.x(), max.y(), max.z());
            return max;
        }

        void
        GetMetricMax(Eigen::Vector3d& max) {
            GetMetricMax(max.x(), max.y(), max.z());
        }

        void
        GetMetricMax(Eigen::Vector3d& max) const {
            GetMetricMax(max.x(), max.y(), max.z());
        }

        virtual void
        GetMetricMax(double& x, double& y, double& z) = 0;
        virtual void
        GetMetricMax(double& x, double& y, double& z) const = 0;

        Aabb3D
        GetMetricAabb() {
            Eigen::Vector3d min, max;
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
            return {min, max};
        }

        [[nodiscard]] Aabb3D
        GetMetricAabb() const {
            Eigen::Vector3d min, max;
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
            return {min, max};
        }

        std::pair<Eigen::Vector3d, Eigen::Vector3d>
        GetMetricMinMax() {
            Eigen::Vector3d min, max;
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
            return {std::move(min), std::move(max)};
        }

        [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d>
        GetMetricMinMax() const {
            Eigen::Vector3d min, max;
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
            return {std::move(min), std::move(max)};
        }

        void
        GetMetricMinMax(Eigen::Vector3d& min, Eigen::Vector3d& max) {
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
        }

        void
        GetMetricMinMax(Eigen::Vector3d& min, Eigen::Vector3d& max) const {
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
        }

        virtual void
        GetMetricMinMax(double& min_x, double& min_y, double& min_z, double& max_x, double& max_y, double& max_z) = 0;
        virtual void
        GetMetricMinMax(double& min_x, double& min_y, double& min_z, double& max_x, double& max_y, double& max_z) const = 0;

        Eigen::Vector3d
        GetMetricSize() {
            Eigen::Vector3d size;
            GetMetricSize(size.x(), size.y(), size.z());
            return size;
        }

        [[nodiscard]] Eigen::Vector3d
        GetMetricSize() const {
            Eigen::Vector3d size;
            GetMetricSize(size.x(), size.y(), size.z());
            return size;
        }

        void
        GetMetricSize(Eigen::Vector3d& size) {
            GetMetricSize(size.x(), size.y(), size.z());
        }

        void
        GetMetricSize(Eigen::Vector3d& size) const {
            GetMetricSize(size.x(), size.y(), size.z());
        }

        virtual void
        GetMetricSize(double& x, double& y, double& z) = 0;
        virtual void
        GetMetricSize(double& x, double& y, double& z) const = 0;

        //-- IO
        virtual void
        Clear() = 0;
        virtual void
        Prune() = 0;
        /**
         * Write the tree as raw data to a file.
         * @param filename
         * @return
         */
        [[nodiscard]] bool
        Write(const std::string& filename) const;
        /**
         * Write the tree as raw data to a stream.
         * @param s
         * @return
         */
        [[nodiscard]] std::ostream&
        Write(std::ostream& s) const;

    protected:
        /**
         * Write all nodes to the output stream (without file header) for a created tree. Pruning the tree first produces smaller files and faster loading.
         * @param s
         * @return
         */
        virtual std::ostream&
        WriteData(std::ostream& s) const = 0;

    public:
        /**
         * Read an octree from a file and cast it to the given type.
         * @tparam T
         * @param filename
         * @return may return nullptr if the cast fails
         */
        template<typename T>
        static std::enable_if_t<std::is_base_of_v<AbstractOctree, T>, std::shared_ptr<T>>
        ReadAs(const std::string& filename) {
            return std::dynamic_pointer_cast<T>(Read(filename));
        }

        /**
         * Generic read function to read an octree from a file.
         * @param filename
         * @return An octree derived from AbstractOctree
         */
        static std::shared_ptr<AbstractOctree>
        Read(const std::string& filename);
        /**
         * Generic read function to read an octree from a stream.
         * @param s
         * @return An octree derived from AbstractOctree
         */
        static std::shared_ptr<AbstractOctree>
        Read(std::istream& s);

        //-- search node
        [[nodiscard]] virtual const AbstractOctreeNode*
        SearchNode(double x, double y, double z, uint32_t max_depth) const = 0;

        //-- iterators
        struct OctreeNodeIterator {
            virtual ~OctreeNodeIterator() = default;

            [[nodiscard]] virtual double
            GetX() const = 0;
            [[nodiscard]] virtual double
            GetY() const = 0;
            [[nodiscard]] virtual double
            GetZ() const = 0;
            [[nodiscard]] virtual double
            GetNodeSize() const = 0;
            [[nodiscard]] virtual uint32_t
            GetDepth() const = 0;
            virtual void
            Next() = 0;
            [[nodiscard]] virtual bool
            IsValid() const = 0;
            [[nodiscard]] virtual const AbstractOctreeNode*
            GetNode() = 0;
        };

        [[nodiscard]] virtual std::shared_ptr<OctreeNodeIterator>
        GetLeafIterator(uint32_t max_depth) const = 0;

        [[nodiscard]] virtual std::shared_ptr<OctreeNodeIterator>
        GetLeafInAabbIterator(const Aabb3D& aabb, uint32_t max_depth) const = 0;

        [[nodiscard]] virtual std::shared_ptr<OctreeNodeIterator>
        GetTreeIterator(uint32_t max_depth) const = 0;

        [[nodiscard]] virtual std::shared_ptr<OctreeNodeIterator>
        GetTreeInAabbIterator(const Aabb3D& aabb, uint32_t max_depth) const = 0;

        [[nodiscard]] virtual std::shared_ptr<OctreeNodeIterator>
        GetNodeOnRayIterator(
            double px,
            double py,
            double pz,
            double vx,
            double vy,
            double vz,
            double max_range,
            bool bidirectional,
            bool leaf_only,
            uint32_t min_node_depth,
            uint32_t max_node_depth) const = 0;

    protected:
        /**
         * Read all nodes from the input steam (without file header) for a created tree.
         */
        virtual std::istream&
        ReadData(std::istream& s) = 0;

    public:
        /**
         * Load the tree data from a file, the tree type in the file has to match the actual tree type.
         * @param filename
         * @return
         */
        bool
        LoadData(const std::string& filename);

        /**
         * Load the tree data from a stream, the tree type in the file has to match the actual tree type.
         * @param s
         * @return
         */
        bool
        LoadData(std::istream& s);

    protected:
        static bool
        ReadHeader(std::istream& s, std::string& tree_id, uint32_t& size);
    };

#define ERL_REGISTER_OCTREE(Derived) inline const volatile bool kRegistered##Derived = erl::geometry::AbstractOctree::RegisterTreeType<Derived>(#Derived)
}  // namespace erl::geometry
