#include "erl_common/binary_file.hpp"
#include "erl_common/eigen.hpp"

namespace erl::geometry {

    class GazeboRoom {

        template<typename T>
        inline static void
        ReadVar(char *&data_ptr, T &var) {
            var = reinterpret_cast<T *>(data_ptr)[0];
            data_ptr += sizeof(T);
        }

        template<typename T>
        inline static void
        ReadPtr(char *&data_ptr, size_t n, T *&ptr) {
            ptr = reinterpret_cast<T *>(data_ptr);
            data_ptr += sizeof(T) * n;
        }

    public:
        inline static const double sk_MaxRange_ = 30.;
        inline static const double sk_MinRange_ = 0.2;
        inline static const double sk_SensorOffsetX_ = 0.08;  // sensor x-offset in the robot frame (IMU frame).
        inline static const double sk_SensorOffsetY_ = 0.;    // sensor y-offset in the robot frame (IMU frame).

        typedef struct TrainDataFrame {
            Eigen::VectorXd angles;
            Eigen::VectorXd distances;
            std::vector<double> pose_matlab;
            Eigen::RMatrix23d pose_numpy;
            Eigen::Vector2d position;  // 2D position
            Eigen::Matrix2d rotation;  // 2D rotation

            TrainDataFrame(double *pa, double *pr, const double *pose_ptr, int numel) {
                angles.resize(numel);
                distances.resize(numel);
                std::copy(pa, pa + numel, angles.begin());
                std::copy(pr, pr + numel, distances.begin());

                Eigen::Matrix23d pose;
                // clang-format off
                pose << pose_ptr[0], pose_ptr[2], pose_ptr[4],
                        pose_ptr[1], pose_ptr[3], pose_ptr[5];
                // clang-format on
                position = pose.col(0);
                rotation = pose.block<2, 2>(0, 1);

                Eigen::Vector2d sensor_offset = rotation * Eigen::Vector2d{sk_SensorOffsetX_, sk_SensorOffsetY_};

                pose_matlab.clear();
                pose_matlab.reserve(6);
                pose_matlab.insert(pose_matlab.begin(), pose_ptr, pose_ptr + 6);
                pose_matlab[0] += sensor_offset[0];
                pose_matlab[1] += sensor_offset[1];

                pose_numpy.topLeftCorner<2, 2>() = rotation;
                pose_numpy.col(2) = position + sensor_offset;
            }

            TrainDataFrame
            Resize(int numel) {
                return {angles.data(), distances.data(), pose_matlab.data(), numel};
            }
        } TrainDataFrame;

        class TrainDataLoader {
            std::vector<TrainDataFrame> m_data_frames_;

        public:
            explicit TrainDataLoader(const char *path) {
                auto data = erl::common::LoadBinaryFile<char>(path);

                char *data_ptr = data.data();
                auto data_ptr_begin = data_ptr;
                size_t data_size = data.size();
                int numel;
                double *pa, *pr, *pose_ptr;
                std::size_t pose_size;

                while (data_ptr < data_ptr_begin + data_size) {
                    GazeboRoom::ReadVar(data_ptr, numel);
                    GazeboRoom::ReadPtr(data_ptr, numel, pa);
                    GazeboRoom::ReadPtr(data_ptr, numel, pr);
                    GazeboRoom::ReadVar(data_ptr, pose_size);
                    GazeboRoom::ReadPtr(data_ptr, pose_size, pose_ptr);
                    m_data_frames_.emplace_back(pa, pr, pose_ptr, numel);
                }
            }

            inline TrainDataFrame &
            operator[](size_t i) {
                return m_data_frames_[i];
            }

            inline const TrainDataFrame &
            operator[](size_t i) const {
                return m_data_frames_[i];
            }

            [[nodiscard]] inline size_t
            size() const {
                return m_data_frames_.size();
            }

            inline auto
            begin() {
                return m_data_frames_.begin();
            }

            inline auto
            end() {
                return m_data_frames_.end();
            }
        };

        struct TestDataFrame {
            Eigen::Matrix2Xd positions;
            std::vector<double> out_buf;  // for GPisMap
            int dim = 0;
            int num_queries = 0;

            explicit TestDataFrame(const char *path) {
                auto data = erl::common::LoadBinaryFile<char>(path);
                auto data_ptr = data.data();
                int numel_px;
                int numel_p_res;
                double *px;

                GazeboRoom::ReadVar(data_ptr, numel_px);
                GazeboRoom::ReadPtr(data_ptr, numel_px, px);
                GazeboRoom::ReadVar(data_ptr, dim);
                GazeboRoom::ReadVar(data_ptr, num_queries);
                GazeboRoom::ReadVar(data_ptr, numel_p_res);

                positions.resize(2, numel_px / 2);
                std::copy(px, px + numel_px, positions.data());
                out_buf.resize(numel_p_res, 0);
            }

            void
            Extract(Eigen::VectorXd &distance, Eigen::Matrix2Xd &gradient, Eigen::VectorXd &distance_variance, Eigen::Matrix2Xd &gradient_variance) {

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
        };
    };
}  // namespace erl::geometry
