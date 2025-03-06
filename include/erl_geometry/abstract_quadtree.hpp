#pragma once

#include "aabb.hpp"
#include "abstract_quadtree_node.hpp"
#include "nd_tree_setting.hpp"

#include "erl_common/factory_pattern.hpp"
#include "erl_common/string_utils.hpp"

#include <functional>
#include <memory>
#include <string>

namespace erl::geometry {

    /**
     * AbstractQuadtree is a base class for all quadtree implementations. It provides a common interface for factory pattern and file I/O.
     */
    template<typename Dtype>
    class AbstractQuadtree {
        inline static const std::string kFileHeader = fmt::format("# {}", type_name<AbstractQuadtree>());
        std::shared_ptr<NdTreeSetting> m_setting_ = std::make_shared<NdTreeSetting>();

    public:
        using Factory = common::FactoryPattern<AbstractQuadtree, false, false, std::shared_ptr<NdTreeSetting>>;
        using Vector2 = Eigen::Vector2<Dtype>;

        AbstractQuadtree() = delete;  // no default constructor

        explicit AbstractQuadtree(std::shared_ptr<NdTreeSetting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        AbstractQuadtree(const AbstractQuadtree& other) = default;
        AbstractQuadtree&
        operator=(const AbstractQuadtree& other) = default;
        AbstractQuadtree(AbstractQuadtree&& other) = default;
        AbstractQuadtree&
        operator=(AbstractQuadtree&& other) = default;

        virtual ~AbstractQuadtree() = default;

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
        [[nodiscard]] virtual std::shared_ptr<AbstractQuadtree>
        Create(const std::shared_ptr<NdTreeSetting>& setting) const = 0;

        /**
         * Create a new tree of the given type.
         * @param tree_id
         * @param setting
         * @return
         */
        static std::shared_ptr<AbstractQuadtree>
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
        operator==(const AbstractQuadtree& other) const = 0;

        [[nodiscard]] bool
        operator!=(const AbstractQuadtree& other) const {
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

        Vector2
        GetMetricMin() {
            Vector2 min;
            GetMetricMin(min.x(), min.y());
            return min;
        }

        [[nodiscard]] Vector2
        GetMetricMin() const {
            Vector2 min;
            GetMetricMin(min.x(), min.y());
            return min;
        }

        void
        GetMetricMin(Vector2& min) {
            GetMetricMin(min.x(), min.y());
        }

        void
        GetMetricMin(Vector2& min) const {
            GetMetricMin(min.x(), min.y());
        }

        virtual void
        GetMetricMin(Dtype& x, Dtype& y) = 0;
        virtual void
        GetMetricMin(Dtype& x, Dtype& y) const = 0;

        Vector2
        GetMetricMax() {
            Vector2 max;
            GetMetricMax(max.x(), max.y());
            return max;
        }

        [[nodiscard]] Vector2
        GetMetricMax() const {
            Vector2 max;
            GetMetricMax(max.x(), max.y());
            return max;
        }

        void
        GetMetricMax(Vector2& max) {
            GetMetricMax(max.x(), max.y());
        }

        void
        GetMetricMax(Vector2& max) const {
            GetMetricMax(max.x(), max.y());
        }

        virtual void
        GetMetricMax(Dtype& x, Dtype& y) = 0;
        virtual void
        GetMetricMax(Dtype& x, Dtype& y) const = 0;

        Aabb<Dtype, 2>
        GetMetricAabb() {
            Vector2 min, max;
            GetMetricMinMax(min.x(), min.y(), max.x(), max.y());
            return {std::move(min), std::move(max)};
        }

        [[nodiscard]] Aabb<Dtype, 2>
        GetMetricAabb() const {
            Vector2 min, max;
            GetMetricMinMax(min.x(), min.y(), max.x(), max.y());
            return {std::move(min), std::move(max)};
        }

        std::pair<Vector2, Vector2>
        GetMetricMinMax() {
            Vector2 min, max;
            GetMetricMinMax(min.x(), min.y(), max.x(), max.y());
            return {std::move(min), std::move(max)};
        }

        [[nodiscard]] std::pair<Vector2, Vector2>
        GetMetricMinMax() const {
            Vector2 min, max;
            GetMetricMinMax(min.x(), min.y(), max.x(), max.y());
            return {std::move(min), std::move(max)};
        }

        void
        GetMetricMinMax(Vector2& min, Vector2& max) {
            GetMetricMinMax(min.x(), min.y(), max.x(), max.y());
        }

        void
        GetMetricMinMax(Vector2& min, Vector2& max) const {
            GetMetricMinMax(min.x(), min.y(), max.x(), max.y());
        }

        virtual void
        GetMetricMinMax(Dtype& min_x, Dtype& min_y, Dtype& max_x, Dtype& max_y) = 0;
        virtual void
        GetMetricMinMax(Dtype& min_x, Dtype& min_y, Dtype& max_x, Dtype& max_y) const = 0;

        Vector2
        GetMetricSize() {
            Vector2 size;
            GetMetricSize(size.x(), size.y());
            return size;
        }

        [[nodiscard]] Vector2
        GetMetricSize() const {
            Vector2 size;
            GetMetricSize(size.x(), size.y());
            return size;
        }

        void
        GetMetricSize(Vector2& size) {
            GetMetricSize(size.x(), size.y());
        }

        void
        GetMetricSize(Vector2& size) const {
            GetMetricSize(size.x(), size.y());
        }

        virtual void
        GetMetricSize(Dtype& x, Dtype& y) = 0;
        virtual void
        GetMetricSize(Dtype& x, Dtype& y) const = 0;

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
        static std::enable_if_t<std::is_base_of_v<AbstractQuadtree, T>, std::shared_ptr<T>>
        ReadAs(const std::string& filename) {
            return std::dynamic_pointer_cast<T>(Read(filename));
        }

        /**
         * Generic read function to read an octree from a file.
         * @param filename
         * @return An octree derived from AbstractOctree
         */
        static std::shared_ptr<AbstractQuadtree>
        Read(const std::string& filename);
        /**
         * Generic read function to read an octree from a stream.
         * @param s
         * @return An octree derived from AbstractOctree
         */
        static std::shared_ptr<AbstractQuadtree>
        Read(std::istream& s);

        //-- search node
        [[nodiscard]] virtual const AbstractQuadtreeNode*
        SearchNode(Dtype x, Dtype y, uint32_t max_depth) const = 0;

        //-- iterators
        struct QuadtreeNodeIterator {
            virtual ~QuadtreeNodeIterator() = default;

            [[nodiscard]] virtual Dtype
            GetX() const = 0;
            [[nodiscard]] virtual Dtype
            GetY() const = 0;
            [[nodiscard]] virtual Dtype
            GetNodeSize() const = 0;
            [[nodiscard]] virtual uint32_t
            GetDepth() const = 0;
            virtual void
            Next() = 0;
            [[nodiscard]] virtual bool
            IsValid() const = 0;
            [[nodiscard]] virtual const AbstractQuadtreeNode*
            GetNode() const = 0;
        };

        [[nodiscard]] virtual std::shared_ptr<QuadtreeNodeIterator>
        GetTreeIterator(uint32_t max_depth) const = 0;

        [[nodiscard]] virtual std::shared_ptr<QuadtreeNodeIterator>
        GetLeafInAabbIterator(const Aabb<Dtype, 2>& aabb, uint32_t max_depth) const = 0;

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

    // #define ERL_REGISTER_QUADTREE(Derived) inline const volatile bool kRegistered##Derived = Derived::Register<Derived>()
}  // namespace erl::geometry

#include "abstract_quadtree.tpp"
