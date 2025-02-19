#pragma once

#include "aabb.hpp"
#include "abstract_octree_node.hpp"
#include "nd_tree_setting.hpp"

#include "erl_common/factory_pattern.hpp"
#include "erl_common/string_utils.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace erl::geometry {

    /**
     * AbstractOctree is a base class for all octree implementations. It provides a common interface for factory pattern and file I/O.
     */

    template<typename Dtype>
    class AbstractOctree {
        std::shared_ptr<NdTreeSetting> m_setting_ = std::make_shared<NdTreeSetting>();

    protected:
        inline static std::map<std::string, std::function<std::shared_ptr<AbstractOctree>(const std::shared_ptr<NdTreeSetting>&)>> s_class_id_mapping_ = {};

    public:
        using Factory = common::FactoryPattern<AbstractOctree, false, false, std::shared_ptr<NdTreeSetting>>;
        using Vector3 = Eigen::Vector3<Dtype>;

        AbstractOctree() = delete;  // no default constructor

        explicit AbstractOctree(std::shared_ptr<NdTreeSetting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
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
        CreateTree(const std::string& tree_id, const std::shared_ptr<NdTreeSetting>& setting) {
            return Factory::GetInstance().Create(tree_id, setting);
        }

        template<typename Derived>
        static bool
        Register(std::string tree_type = "") {
            return Factory::GetInstance().template Register<Derived>(tree_type, [](const std::shared_ptr<NdTreeSetting>& setting) {
                auto tree_setting = std::dynamic_pointer_cast<typename Derived::Setting>(setting);
                if (setting == nullptr) { tree_setting = std::make_shared<typename Derived::Setting>(); }
                ERL_ASSERTM(tree_setting != nullptr, "setting is nullptr.");
                return std::make_shared<Derived>(tree_setting);
            });
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

        [[nodiscard]] Dtype
        GetResolution() const {
            return static_cast<Dtype>(m_setting_->resolution);
        }

        [[nodiscard]] virtual std::size_t
        GetSize() const = 0;
        [[maybe_unused]] [[nodiscard]] virtual std::size_t
        GetMemoryUsage() const = 0;
        [[maybe_unused]] [[nodiscard]] virtual std::size_t
        GetMemoryUsagePerNode() const = 0;

        Vector3
        GetMetricMin() {
            Vector3 min;
            GetMetricMin(min.x(), min.y(), min.z());
            return min;
        }

        [[nodiscard]] Vector3
        GetMetricMin() const {
            Vector3 min;
            GetMetricMin(min.x(), min.y(), min.z());
            return min;
        }

        void
        GetMetricMin(Vector3& min) {
            GetMetricMin(min.x(), min.y(), min.z());
        }

        void
        GetMetricMin(Vector3& min) const {
            GetMetricMin(min.x(), min.y(), min.z());
        }

        virtual void
        GetMetricMin(Dtype& x, Dtype& y, Dtype& z) = 0;
        virtual void
        GetMetricMin(Dtype& x, Dtype& y, Dtype& z) const = 0;

        Vector3
        GetMetricMax() {
            Vector3 max;
            GetMetricMax(max.x(), max.y(), max.z());
            return max;
        }

        [[nodiscard]] Vector3
        GetMetricMax() const {
            Vector3 max;
            GetMetricMax(max.x(), max.y(), max.z());
            return max;
        }

        void
        GetMetricMax(Vector3& max) {
            GetMetricMax(max.x(), max.y(), max.z());
        }

        void
        GetMetricMax(Vector3& max) const {
            GetMetricMax(max.x(), max.y(), max.z());
        }

        virtual void
        GetMetricMax(Dtype& x, Dtype& y, Dtype& z) = 0;
        virtual void
        GetMetricMax(Dtype& x, Dtype& y, Dtype& z) const = 0;

        Aabb<Dtype, 3>
        GetMetricAabb() {
            Vector3 min, max;
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
            return {min, max};
        }

        [[nodiscard]] Aabb<Dtype, 3>
        GetMetricAabb() const {
            Vector3 min, max;
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
            return {min, max};
        }

        std::pair<Vector3, Vector3>
        GetMetricMinMax() {
            Vector3 min, max;
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
            return {std::move(min), std::move(max)};
        }

        [[nodiscard]] std::pair<Vector3, Vector3>
        GetMetricMinMax() const {
            Vector3 min, max;
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
            return {std::move(min), std::move(max)};
        }

        void
        GetMetricMinMax(Vector3& min, Vector3& max) {
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
        }

        void
        GetMetricMinMax(Vector3& min, Vector3& max) const {
            GetMetricMinMax(min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
        }

        virtual void
        GetMetricMinMax(Dtype& min_x, Dtype& min_y, Dtype& min_z, Dtype& max_x, Dtype& max_y, Dtype& max_z) = 0;
        virtual void
        GetMetricMinMax(Dtype& min_x, Dtype& min_y, Dtype& min_z, Dtype& max_x, Dtype& max_y, Dtype& max_z) const = 0;

        Vector3
        GetMetricSize() {
            Vector3 size;
            GetMetricSize(size.x(), size.y(), size.z());
            return size;
        }

        [[nodiscard]] Vector3
        GetMetricSize() const {
            Vector3 size;
            GetMetricSize(size.x(), size.y(), size.z());
            return size;
        }

        void
        GetMetricSize(Vector3& size) {
            GetMetricSize(size.x(), size.y(), size.z());
        }

        void
        GetMetricSize(Vector3& size) const {
            GetMetricSize(size.x(), size.y(), size.z());
        }

        virtual void
        GetMetricSize(Dtype& x, Dtype& y, Dtype& z) = 0;
        virtual void
        GetMetricSize(Dtype& x, Dtype& y, Dtype& z) const = 0;

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
        SearchNode(Dtype x, Dtype y, Dtype z, uint32_t max_depth) const = 0;

        //-- iterators
        struct OctreeNodeIterator {
            virtual ~OctreeNodeIterator() = default;

            [[nodiscard]] virtual Dtype
            GetX() const = 0;
            [[nodiscard]] virtual Dtype
            GetY() const = 0;
            [[nodiscard]] virtual Dtype
            GetZ() const = 0;
            [[nodiscard]] virtual Dtype
            GetNodeSize() const = 0;
            [[nodiscard]] virtual uint32_t
            GetDepth() const = 0;
            virtual void
            Next() = 0;
            [[nodiscard]] virtual bool
            IsValid() const = 0;
            [[nodiscard]] virtual const AbstractOctreeNode*
            GetNode() const = 0;
        };

        [[nodiscard]] virtual std::shared_ptr<OctreeNodeIterator>
        GetTreeIterator(uint32_t max_depth) const = 0;

        [[nodiscard]] virtual std::shared_ptr<OctreeNodeIterator>
        GetLeafInAabbIterator(const Aabb<Dtype, 3>& aabb, uint32_t max_depth) const = 0;

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

#define ERL_REGISTER_OCTREE(Derived) inline const volatile bool kRegistered##Derived = Derived::Register<Derived>()
}  // namespace erl::geometry

#include "abstract_octree.tpp"
