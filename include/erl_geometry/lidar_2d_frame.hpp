#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    struct Lidar2DFrame {
        double x, y, theta;
        Eigen::VectorXd angles, ranges;

        Lidar2DFrame(double x, double y, double theta, const Eigen::Ref<const Eigen::VectorXd> &angles, const Eigen::Ref<const Eigen::VectorXd> &ranges)
            : x(x), y(y), theta(theta) {

            this->angles.resize(angles.size());
            this->ranges.resize(ranges.size());

            int cnt = 0;
            for (int i = 0; i < angles.size(); ++i) {
                if (!std::isnan(ranges[i])) {
                    this->angles[cnt] = angles[i];
                    this->ranges[cnt] = ranges[i];
                    cnt++;
                }
            }

            this->angles.conservativeResize(cnt);
            this->ranges.conservativeResize(cnt);
        }

        [[nodiscard]] long GetNumAngles() const {
            return angles.size();
        }

        [[nodiscard]] Eigen::Vector2d GetTranslation() const {
            return {x, y};
        }

        [[nodiscard]] Eigen::Matrix2d GetRotation() const {
            return Eigen::Rotation2Dd(theta).toRotationMatrix();
        }

        [[nodiscard]] Eigen::Matrix3d GetPose() const {
            return (Eigen::Translation2d(x, y) * Eigen::Rotation2Dd(theta)).matrix();
        }

        [[nodiscard]] Eigen::Matrix2Xd
        GetRayDirections() const {
            Eigen::Matrix2Xd directions(2, angles.size());
            directions << angles.array().cos().transpose(), angles.array().sin().transpose();
            return directions;
        }

        [[nodiscard]] Eigen::Matrix2Xd
        GetOrientedRayDirections() const {
            return GetPose().topLeftCorner<2, 2>() * GetRayDirections();
        }

        [[nodiscard]] Eigen::Matrix2Xd
        GetRayEndPointsInBodyFrame() const {
            Eigen::Matrix2Xd points(2, angles.size());
            points << (ranges.array() * angles.array().cos()).transpose(), (ranges.array() * angles.array().sin()).transpose();
            return points;
        }

        [[nodiscard]] Eigen::Matrix2Xd
        GetRayEndPointsInWorldFrame() const {
            return Eigen::Isometry2d(GetPose()) * GetRayEndPointsInBodyFrame();
        }

    };
}
