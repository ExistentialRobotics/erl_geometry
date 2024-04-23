#pragma once

#include <memory>
#include <map>
#include <string>
#include "nd_tree_setting.hpp"

namespace erl::geometry {

    /**
     * AbstractOctree is a base class for all octree implementations. It provides a common interface for factory pattern and file I/O.
     */
    class AbstractOctree {
    protected:
        std::shared_ptr<NdTreeSetting> m_setting_ = std::make_shared<NdTreeSetting>();
        inline static std::map<std::string, std::shared_ptr<AbstractOctree>> s_class_id_mapping_ = {};  // cppcheck-suppress unusedStructMember

    public:
        AbstractOctree() = delete;  // no default constructor

        explicit AbstractOctree(const std::shared_ptr<NdTreeSetting>& setting)
            : m_setting_(setting) {}

        AbstractOctree(const AbstractOctree&) = delete;

        virtual ~AbstractOctree() = default;

        template<typename T>
        inline std::shared_ptr<T>
        GetSetting() const {
            return std::reinterpret_pointer_cast<T>(m_setting_);
        }

        /**
         * This function should be called after the tree is created or when the setting is changed.
         */
        virtual void
        ApplySetting() = 0;

        inline void
        ReadSetting(std::istream& s) {
            std::streamsize len;
            s.read(reinterpret_cast<char*>(&len), sizeof(std::size_t));
            std::string yaml_str(len, '\0');
            s.read(yaml_str.data(), len);
            m_setting_->FromYamlString(yaml_str);
        }

        inline void
        WriteSetting(std::ostream& s) const {
            std::string yaml_str = m_setting_->AsYamlString();
            auto len = std::streamsize(yaml_str.size());
            s.write(reinterpret_cast<const char*>(&len), sizeof(std::size_t));
            s.write(yaml_str.data(), len);
        }

        //-- comparison
        [[nodiscard]] virtual bool
        operator==(const AbstractOctree& other) const = 0;

        [[nodiscard]] inline bool
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

        /// returns actual class name as string for identification
        [[nodiscard]] virtual std::string
        GetTreeType() const = 0;
        [[nodiscard]] virtual std::size_t
        GetSize() const = 0;
        [[maybe_unused]] [[nodiscard]] virtual std::size_t
        GetMemoryUsage() const = 0;
        [[maybe_unused]] [[nodiscard]] virtual std::size_t
        GetMemoryUsagePerNode() const = 0;
        virtual void
        GetMetricMin(double& x, double& y, double& z) = 0;
        virtual void
        GetMetricMin(double& x, double& y, double& z) const = 0;
        virtual void
        GetMetricMax(double& x, double& y, double& z) = 0;
        virtual void
        GetMetricMax(double& x, double& y, double& z) const = 0;
        virtual void
        GetMetricMinMax(double& min_x, double& min_y, double& min_z, double& max_x, double& max_y, double& max_z) = 0;
        virtual void
        GetMetricMinMax(double& min_x, double& min_y, double& min_z, double& max_x, double& max_y, double& max_z) const = 0;
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
        inline static std::enable_if_t<std::is_base_of_v<AbstractOctree, T>, std::shared_ptr<T>>
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
        static void
        RegisterTreeType(const std::shared_ptr<AbstractOctree>& tree);
        inline static const std::string sk_FileHeader_ = "# erl::geometry::AbstractOctree";  // cppcheck-suppress unusedStructMember

        //-- factory pattern
        [[nodiscard]] virtual std::shared_ptr<AbstractOctree>
        Create() const = 0;
        static std::shared_ptr<AbstractOctree>
        CreateTree(const std::string& tree_id);
    };
}  // namespace erl::geometry
