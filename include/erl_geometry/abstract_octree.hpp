#pragma once

#include <memory>
#include <map>
#include <string>

namespace erl::geometry {

    /**
     * AbstractOctree is a base class for all octree implementations. It provides a common interface for factory pattern and file I/O.
     */
    class AbstractOctree {

    public:
        AbstractOctree() = default;
        virtual ~AbstractOctree() = default;

        [[nodiscard]] virtual std::shared_ptr<AbstractOctree>
        Create() const = 0;

        //-- get tree information

        /// returns actual class name as string for identification
        [[nodiscard]] virtual std::string
        GetTreeType() const = 0;
        [[nodiscard]] virtual double
        GetResolution() const = 0;
        virtual void
        SetResolution(double res) = 0;
        [[nodiscard]] virtual std::size_t
        GetSize() const = 0;
        [[nodiscard]] virtual std::size_t
        GetMemoryUsage() const = 0;
        [[nodiscard]] virtual std::size_t
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

        /**
         * Create a new tree of the given type and resolution.
         * @param id string of the tree type
         * @param res resolution of the tree
         * @return
         */
        static std::shared_ptr<AbstractOctree>
        CreateTree(const std::string& id, double res);

    private:
        inline static std::map<std::string, std::shared_ptr<AbstractOctree>> s_class_id_mapping_ = {};

    protected:
        static bool
        ReadHeader(std::istream& s, std::string& id, unsigned int& size, double& res);
        static void
        RegisterTreeType(const std::shared_ptr<AbstractOctree>& tree);
        inline static const std::string sk_FileHeader_ = "# erl::geometry::AbstractOctree";
    };
}  // namespace erl::geometry
