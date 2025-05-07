#include "erl_geometry/gazebo_room_2d.hpp"

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

}  // namespace erl::geometry
