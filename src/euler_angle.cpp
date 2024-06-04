#include "erl_geometry/euler_angle.hpp"

#include "erl_common/logging.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace erl::geometry {

    EulerAngleOrder
    GetEulerAngleOrder(const std::string& euler_order) {
        const auto s = static_cast<char>(std::tolower(euler_order.c_str()[0]));
        const auto x = static_cast<char>(std::tolower(euler_order.c_str()[1]));
        const auto y = static_cast<char>(std::tolower(euler_order.c_str()[2]));
        const auto z = static_cast<char>(std::tolower(euler_order.c_str()[3]));

        ERL_ASSERTM(x != y && y != z, "Invalid Euler angle order: {}", euler_order.c_str());

        int order = 0;

        if (s == 'r') { order = 0b1000000; }

        if (x == 'y') {
            order |= 0b010000;
        } else if (x == 'z') {
            order |= 0b100000;
        }

        if (y == 'y') {
            order |= 0b000100;
        } else if (y == 'z') {
            order |= 0b001000;
        }

        if (z == 'y') {
            order |= 0b000001;
        } else if (z == 'z') {
            order |= 0b000010;
        }

        return static_cast<EulerAngleOrder>(order);
    }

    Eigen::Matrix3d
    EulerToRotation3D(double a, double b, double c, EulerAngleOrder euler_angle_order) {
        const auto order = static_cast<int>(euler_angle_order);

        /*
         * rzyx (sxyz): yaw = a, pitch = b, roll = c, i = 0, j = 1, k = 2, cross(x, y) = z --> right-handed
         *    / cos(a) cos(b), cos(a) sin(b) sin(c) - cos(c) sin(a), sin(a) sin(c) + cos(a) cos(c) sin(b) \
         *    |                                                                                           |
         *    | cos(b) sin(a), cos(a) cos(c) + sin(a) sin(b) sin(c), cos(c) sin(a) sin(b) - cos(a) sin(c) |
         *    |                                                                                           |
         *    \    -sin(b),                cos(b) sin(c),                        cos(b) cos(c)            /
         *
         * rxyz (szyx): roll = a, pitch = b, yaw = c, i = 2, j = 1, k = 0, cross(z, y) = -x --> left-handed
         *    /             cos(b) cos(c),                       -cos(b) sin(c),                sin(b)     \
         *    |                                                                                            |
         *    | cos(a) sin(c) + cos(c) sin(a) sin(b), cos(a) cos(c) - sin(a) sin(b) sin(c), -cos(b) sin(a) |
         *    |                                                                                            |
         *    \ sin(a) sin(c) - cos(a) cos(c) sin(b), cos(c) sin(a) + cos(a) sin(b) sin(c),  cos(a) cos(b) /
         *
         * rxzy (syzx): roll = a, yaw = b, pitch = c, i = 1, j = 2, k = 0, cross(y, z) = x --> right-handed
         *    /             cos(b) cos(c),               -sin(b),                cos(b) sin(c)            \  <-- if a=-a, b=-b, c=-c, this is the same as rxyz
         *    |                                                                                           |
         *    | sin(a) sin(c) + cos(a) cos(c) sin(b), cos(a) cos(b), cos(a) sin(b) sin(c) - cos(c) sin(a) |
         *    |                                                                                           |
         *    \ cos(c) sin(a) sin(b) - cos(a) sin(c), cos(b) sin(a), cos(a) cos(c) + sin(a) sin(b) sin(c) /
         *
         * rzxy (syxz): yaw = a, roll = b, pitch = c, i = 1, j = 0, k = 2, cross(y, x) = -z --> left-handed
         *    / cos(a) cos(c) - sin(a) sin(b) sin(c), -cos(b) sin(a), cos(a) sin(c) + cos(c) sin(a) sin(b) \
         *    |                                                                                            |
         *    | cos(c) sin(a) + cos(a) sin(b) sin(c),  cos(a) cos(b), sin(a) sin(c) - cos(a) cos(c) sin(b) |
         *    |                                                                                            |
         *    \            -cos(b) sin(c),                sin(b),                 cos(b) cos(c)            /
         *
         * Connection between static and relative order:
         *      R_s(x, y, z) = R_r(z, y, x)
         *      so, EulerToRotation3D(a, b, c, "sxyz") == EulerToRotation3D(c, b, a, "rzyx")
         * Note that Rot(angle).T = Rot(-angle)
         *      R_rxyz = Rx Ry Rz
         *      R_rxyz(m_alpha_, beta, gamma).T = Rz.T Ry.T Rx.T = R_rzyx(-gamma, -beta, -m_alpha_) = R_sxyz(-m_alpha_, -beta, -gamma)
         *      so, EulerToRotation3D(a, b, c, "rxyz") == EulerToRotation3D(-a, -b, -c, "sxyz").T
         *          EulerToRotation3D(a, b, c, "sxzy") == EulerToRotation3D(-a, -b, -c, "rxzy").T == EulerToRotation3D(-c, -b, -a, "syzx").T
         *                xzy is left-handed                                                                    yzx is right-handed
         *
         * rxyx' (sx'yx): roll = a, pitch = b, roll' = c, i = 0, j = 1, k = 2, cross(x, y) = z --> right-handed
         *    /     cos(b),                 sin(b) sin(c),                         cos(c) sin(b)             \
         *    |                                                                                              |
         *    |  sin(a) sin(b), cos(a) cos(c) - cos(b) sin(a) sin(c), - cos(a) sin(c) - cos(b) cos(c) sin(a) |
         *    |                                                                                              |
         *    \ -cos(a) sin(b), cos(c) sin(a) + cos(a) cos(b) sin(c),  cos(a) cos(b) cos(c) - sin(a) sin(c)  /
         *
         * rzyz' (sz'yz): yaw = a, pitch = b, yaw' = c, i = 2, j = 1, k = 0, cross(z, y) = -x --> left-handed
         *    / cos(a) cos(b) cos(c) - sin(a) sin(c), - cos(c) sin(a) - cos(a) cos(b) sin(c), cos(a) sin(b) \
         *    |                                                                                             |
         *    | cos(a) sin(c) + cos(b) cos(c) sin(a),  cos(a) cos(c) - cos(b) sin(a) sin(c),  sin(a) sin(b) |
         *    |                                                                                             |
         *    \            -cos(c) sin(b),                         sin(b) sin(c),                 cos(b)    /
         *
         * rxzx' (sx'zx): roll = a, yaw = b, roll' = c, i = 0, j = 2, k = 1, cross(x, z) = -y --> left-handed
         *    /     cos(b),               -cos(c) sin(b),                         sin(b) sin(c)             \
         *    |                                                                                             |
         *    | cos(a) sin(b), cos(a) cos(b) cos(c) - sin(a) sin(c), - cos(c) sin(a) - cos(a) cos(b) sin(c) |
         *    |                                                                                             |
         *    \ sin(a) sin(b), cos(a) sin(c) + cos(b) cos(c) sin(a),  cos(a) cos(c) - cos(b) sin(a) sin(c)  /
         */

        const bool is_relative = (order & 1 << 6) >> 6;  // 0b1xxxxxx
        int i = (order & 0b11 << 4) >> 4;
        const int j = (order & 0b11 << 2) >> 2;
        int k = order & 0b11;
        if (is_relative) {
            std::swap(a, c);
            std::swap(i, k);
        }
        const bool repetition = i == k;
        k = 3 - i - j;

        Eigen::Vector3i axis_vec_a = Eigen::Vector3i::Zero();
        Eigen::Vector3i axis_vec_b = Eigen::Vector3i::Zero();
        Eigen::Vector3i axis_vec_c = Eigen::Vector3i::Zero();
        axis_vec_a[i] = 1;
        axis_vec_b[j] = 1;
        axis_vec_c[k] = 1;
        if (axis_vec_a.cross(axis_vec_b).dot(axis_vec_c) < 0) {
            a = -a;
            b = -b;
            c = -c;
        }

        const double sa = std::sin(a), ca = std::cos(a);
        const double sb = std::sin(b), cb = std::cos(b);
        const double sc = std::sin(c), cc = std::cos(c);
        const double cc_ca = cc * ca;
        const double cc_sa = cc * sa;
        const double sc_ca = sc * ca;
        const double sc_sa = sc * sa;
        Eigen::Matrix3d r;
        // clang-format off
        if (repetition) {
            r(i, i) = cb,       r(i, j) = sb * sc,             r(i, k) = sb * cc;
            r(j, i) = sa * sb,  r(j, j) = -cb * sc_sa + cc_ca, r(j, k) = -cb * cc_sa - sc_ca;
            r(k, i) = -ca * sb, r(k, j) = cb * sc_ca + cc_sa,  r(k, k) = cb * cc_ca - sc_sa;
        } else {
            r(i, i) = ca * cb, r(i, j) = sb * sc_ca - cc_sa, r(i, k) = sb * cc_ca + sc_sa;
            r(j, i) = cb * sa, r(j ,j) = sb * sc_sa + cc_ca, r(j, k) = sb * cc_sa - sc_ca;
            r(k, i) = -sb,     r(k, j) = cb * sc,            r(k, k) = cb * cc;
        }
        // clang-format on

        return r;
    }
}  // namespace erl::geometry
