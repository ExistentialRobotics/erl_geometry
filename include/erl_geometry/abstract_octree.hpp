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

        // Write to file
        [[nodiscard]] bool
        Write(const std::string& filename) const;
        // Write to stream
        [[nodiscard]] std::ostream&
        Write(std::ostream& s) const;
        /**
         * Write all nodes to the output stream (without file header) for a created tree. Pruning the tree first produces smaller files and faster loading.
         * @param s
         * @return
         */
        virtual std::ostream&
        WriteData(std::ostream& s) const = 0;

        // Read from file
        static std::shared_ptr<AbstractOctree>
        Read(const std::string& filename);
        // Read from stream
        static std::shared_ptr<AbstractOctree>
        Read(std::istream& s);
        /**
         * Read all nodes from the input steam (without file header) for a created tree.
         */
        virtual std::istream&
        ReadData(std::istream& s) = 0;

        /**
         * Create a new tree of the given type.
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
