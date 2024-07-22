#include "erl_geometry/ucsd_fah_2d.hpp"

#include <Eigen/Geometry>

namespace erl::geometry {
    UcsdFah2D::UcsdFah2D(const std::string &filename)
        : m_data_(common::LoadEigenMatrixFromBinaryFile(filename)),
          m_num_rays_((m_data_.rows() - 6) / 2) {}

    UcsdFah2D::Frame
    UcsdFah2D::operator[](const long index) const {
        const double *data = m_data_.col(index).data();
        return {
            static_cast<long>(data[0]),
            static_cast<long>(data[1]),
            static_cast<long>(data[2]),
            Eigen::Rotation2Dd(data[5]).toRotationMatrix(),
            {data[3], data[4]},
            Eigen::Map<const Eigen::VectorXd>(data + 6, m_num_rays_),
            Eigen::Map<const Eigen::VectorXd>(data + 6 + m_num_rays_, m_num_rays_)};
    }

}  // namespace erl::geometry
