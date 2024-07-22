#include "erl_geometry/gazebo_room_2d.hpp"

namespace erl::geometry {

    GazeboRoom2D::TrainDataFrame::TrainDataFrame(double *pa, double *pr, const double *pose_ptr, const int numel) {
        angles.resize(numel);
        ranges.resize(numel);
        std::copy_n(pa, numel, angles.begin());
        std::copy_n(pr, numel, ranges.begin());

        // clang-format off
        rotation << pose_ptr[2], pose_ptr[4],
                    pose_ptr[3], pose_ptr[5];
        translation << pose_ptr[0], pose_ptr[1];
        // clang-format on
        translation += rotation * Eigen::Vector2d(kSensorOffsetX, kSensorOffsetY);
    }

    GazeboRoom2D::TrainDataLoader::TrainDataLoader(const std::string &path) {
        auto data = erl::common::LoadBinaryFile<char>(path);

        char *data_ptr = data.data();
        const auto data_ptr_begin = data_ptr;
        const size_t data_size = data.size();
        int numel;
        double *pa, *pr, *pose_ptr;
        std::size_t pose_size;

        while (data_ptr < data_ptr_begin + data_size) {
            ReadVar(data_ptr, numel);
            ReadPtr(data_ptr, numel, pa);
            ReadPtr(data_ptr, numel, pr);
            ReadVar(data_ptr, pose_size);
            ReadPtr(data_ptr, pose_size, pose_ptr);
            m_data_frames_.emplace_back(pa, pr, pose_ptr, numel);
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
