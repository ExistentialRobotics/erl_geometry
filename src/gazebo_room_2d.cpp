#include "erl_geometry/gazebo_room_2d.hpp"

#include "erl_common/eigen.hpp"
#include "erl_geometry/polygon_to_mesh.hpp"

#include <open3d/geometry/LineSet.h>
#include <open3d/t/geometry/LineSet.h>
#include <open3d/t/geometry/TriangleMesh.h>

namespace erl::geometry {

    template<typename T>
    static void
    ReadVar(char *&data_ptr, T &var) {
        var = reinterpret_cast<T *>(data_ptr)[0];
        data_ptr += sizeof(T);
    }

    template<typename T>
    static void
    ReadPtr(char *&data_ptr, const size_t n, T *&ptr) {
        ptr = reinterpret_cast<T *>(data_ptr);
        data_ptr += sizeof(T) * n;
    }

    GazeboRoom2D::TrainDataLoader::TrainDataLoader(const std::string &path) {
        using namespace common;
        const std::filesystem::path folder(path);
        // (3, N)
        Eigen::Matrix3Xd poses = LoadEigenMatrixFromBinaryFile(folder / "poses.dat");
        // (270, N)
        Eigen::MatrixXd ranges = LoadEigenMatrixFromBinaryFile(folder / "ranges.dat");
        // (N, )
        Eigen::VectorXd thetas = LoadEigenMatrixFromBinaryFile(folder / "thetas.dat").leftCols<1>();

        for (long i = 0; i < poses.cols(); ++i) {
            TrainDataFrame frame;
            frame.rotation = Eigen::Rotation2Dd(poses(2, i)).toRotationMatrix();
            frame.translation = poses.col(i).head<2>() +
                                frame.rotation * Eigen::Vector2d(kSensorOffsetX, kSensorOffsetY);
            frame.angles = thetas;
            frame.ranges = ranges.col(i);
            m_data_frames_.emplace_back(std::move(frame));
        }
    }

    GazeboRoom2D::TestDataFrame::TestDataFrame(const std::string &path) {
        auto data = erl::common::LoadBinaryFile<char>(path);
        auto data_ptr = data.data();
        int numel_px;
        int numel_p_res;
        double *px;

        ReadVar(data_ptr, numel_px);
        ReadPtr(data_ptr, numel_px, px);
        ReadVar(data_ptr, dim);
        ReadVar(data_ptr, num_queries);
        ReadVar(data_ptr, numel_p_res);

        positions.resize(2, numel_px / 2);
        std::copy_n(px, numel_px, positions.data());
        out_buf.resize(numel_p_res, 0);
    }

    void
    GazeboRoom2D::TestDataFrame::Extract(
        Eigen::VectorXd &distance,
        Eigen::Matrix2Xd &gradient,
        Eigen::VectorXd &distance_variance,
        Eigen::Matrix2Xd &gradient_variance) {

        auto out = Eigen::Map<Eigen::MatrixXd>(out_buf.data(), 2 * (dim + 1), num_queries);
        distance.resize(num_queries);
        gradient.resize(2, num_queries);
        distance_variance.resize(num_queries);
        gradient_variance.resize(2, num_queries);

        for (int i = 0; i < num_queries; ++i) {
            distance[i] = out(0, i);
            distance_variance[i] = out(1 + dim, i);
            for (int j = 0; j < dim; ++j) {
                gradient(j, i) = out(1 + j, i);
                gradient_variance(j, i) = out(2 + dim + j, i);
            }
        }
    }

    std::shared_ptr<open3d::geometry::TriangleMesh>
    GazeboRoom2D::ExtrudeTo3D(const double room_height, const bool add_ceiling) {
        open3d::geometry::LineSet wall_line_set;
        wall_line_set.points_ = {
            {-3.10, 1.71, 0.0},
            {6.58, 2.58, 0.0},
            {6.9, -1.23, 0.0},
            {3.95, -1.49, 0.0},
            {4.02, -2.65, 0.0},
            {8.17, -2.27, 0.0},
            {7.73, 2.67, 0.0},
            {16.53, 3.43, 0.0},
            {17.99, -12.27, 0.0},
            {-1.77, -14.04, 0.0},
            {-2.07, -10.20, 0.0},
            {3.84, -9.69, 0.0},
            {3.65, -7.57, 0.0},
            {-2.36, -8.05, 0.0},  // outer wall
            {10.42, -6.03, 0.0},
            {10.68, -9.06, 0.0},
            {13.84, -8.81, 0.0},
            {13.48, -5.71, 0.0},  // inner wall
        };
        wall_line_set.lines_ = {
            {0, 1},
            {1, 2},
            {2, 3},
            {3, 4},
            {4, 5},
            {5, 6},
            {6, 7},
            {7, 8},
            {8, 9},
            {9, 10},
            {10, 11},
            {11, 12},
            {12, 13},
            {13, 0},  // outer wall
            {14, 15},
            {15, 16},
            {16, 17},
            {17, 14},  // inner wall
        };
        const auto wall_line_set_t = open3d::t::geometry::LineSet::FromLegacy(wall_line_set);
        const open3d::core::Tensor z_dir(std::vector<double>{0.0, 0.0, room_height});
        auto wall_mesh = wall_line_set_t.ExtrudeLinear(z_dir).ToLegacy();
        wall_mesh.PaintUniformColor({0.5, 0.5, 1.0});  // light blue
        wall_mesh.ComputeVertexNormals();

        std::vector<std::vector<Eigen::Vector2d>> polygons(2);
        polygons[0].reserve(14);
        for (int i = 0; i < 14; ++i) {
            polygons[0].emplace_back(wall_line_set.points_[i][0], wall_line_set.points_[i][1]);
        }
        polygons[1].reserve(4);
        for (int i = 14; i < 18; ++i) {
            polygons[1].emplace_back(wall_line_set.points_[i][0], wall_line_set.points_[i][1]);
        }
        auto ground_mesh = PolygonToMesh(polygons, 0.0, true);
        ground_mesh->PaintUniformColor({0.67, 0.33, 0.0});  // brown

        auto room_mesh = std::make_shared<open3d::geometry::TriangleMesh>(wall_mesh);
        *room_mesh += *ground_mesh;

        if (add_ceiling) {
            auto ceiling_mesh = PolygonToMesh(polygons, room_height, false);
            ceiling_mesh->PaintUniformColor({1.0, 1.0, 0.67});  // light yellow
            *room_mesh += *ceiling_mesh;
        }
        return room_mesh;
    }

}  // namespace erl::geometry
