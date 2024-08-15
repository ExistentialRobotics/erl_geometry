#include "erl_geometry/intersection.hpp"

#include <erl_common/block_timer.hpp>

#include <cmath>

namespace erl::geometry {

    double
    ComputeNearestDistanceFromPointToLineSegment2D(const double x0, const double y0, const double x1, const double y1, const double x2, const double y2) {

        const double dx_20 = x2 - x0;
        const double dx_21 = x2 - x1;
        const double dy_20 = y2 - y0;
        const double dy_21 = y2 - y1;
        const double d = dx_21 * dx_21 + dy_21 * dy_21;
        const double lam = (dx_20 * dx_21 + dy_20 * dy_21) / d;

        double dist = (dy_21 * dx_20 - dy_20 * dx_21) / std::sqrt(d);
        dist = std::abs(dist);

        if (lam > 1.) {
            const double dx_10 = x1 - x0;
            const double dy_10 = y1 - y0;
            dist = std::sqrt(dx_10 * dx_10 + dy_10 * dy_10);
        } else if (lam < 0.) {
            dist = std::sqrt(dx_20 * dx_20 + dy_20 * dy_20);
        }

        return dist;
    }

    void
    ComputeIntersectionBetweenTwoLines2D(
        const Eigen::Vector2d &p00,
        const Eigen::Vector2d &p01,
        const Eigen::Vector2d &p10,
        const Eigen::Vector2d &p11,
        double &lam1,
        double &lam2,
        bool &intersected) {

        Eigen::Vector2d v_0 = p01 - p00;
        Eigen::Vector2d v_1 = p11 - p10;

        const double tmp = v_1.x() * v_0.y() - v_1.y() * v_0.x();  // are the two lines parallel?
        intersected = std::abs(tmp) > std::numeric_limits<double>::min();
        if (!intersected) {
            lam1 = std::numeric_limits<double>::infinity();
            lam2 = std::numeric_limits<double>::infinity();
            return;
        }
        lam1 = (v_1.x() * (p00.y() - p10.y()) - v_1.y() * (p00.x() - p10.x())) / tmp;
        lam2 = (v_0.x() * (p01.y() - p10.y()) - v_0.y() * (p01.x() - p10.x())) / tmp;
    }

    void
    ComputeIntersectionBetweenRayAndLine2D(
        const Eigen::Vector2d &p0,
        const Eigen::Vector2d &v,
        const Eigen::Vector2d &p1,
        const Eigen::Vector2d &p2,
        double &lam,
        double &dist,
        bool &intersected) {

        Eigen::Vector2d v_21 = p2 - p1;
        Eigen::Vector2d v_20 = p2 - p0;

        const double tmp = v_21.x() * v.y() - v_21.y() * v.x();  // tmp = (p2 - p1).cross(v)
        intersected = std::abs(tmp) > std::numeric_limits<double>::min();
        if (!intersected) {
            lam = std::numeric_limits<double>::infinity();
            dist = std::numeric_limits<double>::infinity();
            return;
        }
        lam = (v_20.x() * v.y() - v_20.y() * v.x()) / tmp;         // (p2 - p0).cross(v) / tmp
        dist = (v_21.x() * v_20.y() - v_21.y() * v_20.x()) / tmp;  // dist = (p2 - p1).cross(p2 - p0) / tmp
    }

    void
    ComputeIntersectionBetweenRayAndAabb2D(
        const Eigen::Vector2d &p,
        const Eigen::Vector2d &v_inv,
        const Eigen::Vector2d &box_min,
        const Eigen::Vector2d &box_max,
        double &d1,
        double &d2,
        bool &intersected) {

        double tx_1, tx_2, ty_1, ty_2;
        if (p[0] == box_min[0]) {
            tx_1 = 0;
        } else {
            tx_1 = (box_min[0] - p[0]) * v_inv[0];
        }
        if (p[0] == box_max[0]) {
            tx_2 = 0;
        } else {
            tx_2 = (box_max[0] - p[0]) * v_inv[0];
        }
        double t_min = std::min(tx_1, tx_2);
        double t_max = std::max(tx_1, tx_2);

        if (p[1] == box_min[1]) {
            ty_1 = 0;
        } else {
            ty_1 = (box_min[1] - p[1]) * v_inv[1];
        }
        if (p[1] == box_max[1]) {
            ty_2 = 0;
        } else {
            ty_2 = (box_max[1] - p[1]) * v_inv[1];
        }
        t_min = std::max(t_min, std::min(ty_1, ty_2));
        t_max = std::min(t_max, std::max(ty_1, ty_2));

        intersected = t_max >= t_min;
        d1 = std::numeric_limits<double>::infinity();
        d2 = std::numeric_limits<double>::infinity();
        if (intersected) {
            if (p[0] < box_min[0] || p[0] > box_max[0] || p[1] < box_min[1] || p[1] > box_max[1]) {  // ray start point is outside the box
                if (t_min >= 0) {                                                                    // forward intersection
                    d1 = t_min;                                                                      // first intersection point
                    d2 = t_max;                                                                      // second intersection point
                } else {                                                                             // backward intersection
                    d1 = t_max;                                                                      // first intersection point
                    d2 = t_min;                                                                      // second intersection point
                }
            } else {         // ray start point is inside the box
                d1 = t_max;  // forward intersection
                d2 = t_min;  // backward intersection
            }
        }
    }

    void
    ComputeIntersectionBetweenRayAndAabb3D(
        const Eigen::Vector3d &p,
        const Eigen::Vector3d &r_inv,
        const Eigen::Vector3d &box_min,
        const Eigen::Vector3d &box_max,
        double &d1,
        double &d2,
        bool &intersected) {

        double tx_1, tx_2, ty_1, ty_2, tz_1, tz_2;
        if (p[0] == box_min[0]) {
            tx_1 = 0;
        } else {
            tx_1 = (box_min[0] - p[0]) * r_inv[0];
        }
        if (p[0] == box_max[0]) {
            tx_2 = 0;
        } else {
            tx_2 = (box_max[0] - p[0]) * r_inv[0];
        }
        double t_min = std::min(tx_1, tx_2);
        double t_max = std::max(tx_1, tx_2);

        if (p[1] == box_min[1]) {
            ty_1 = 0;
        } else {
            ty_1 = (box_min[1] - p[1]) * r_inv[1];
        }
        if (p[1] == box_max[1]) {
            ty_2 = 0;
        } else {
            ty_2 = (box_max[1] - p[1]) * r_inv[1];
        }
        t_min = std::max(t_min, std::min(ty_1, ty_2));
        t_max = std::min(t_max, std::max(ty_1, ty_2));

        if (p[2] == box_min[2]) {
            tz_1 = 0;
        } else {
            tz_1 = (box_min[2] - p[2]) * r_inv[2];
        }
        if (p[2] == box_max[2]) {
            tz_2 = 0;
        } else {
            tz_2 = (box_max[2] - p[2]) * r_inv[2];
        }
        t_min = std::max(t_min, std::min(tz_1, tz_2));
        t_max = std::min(t_max, std::max(tz_1, tz_2));

        intersected = t_max >= t_min;
        d1 = std::numeric_limits<double>::infinity();
        d2 = std::numeric_limits<double>::infinity();
        if (intersected) {
            if (p[0] < box_min[0] || p[0] > box_max[0] ||  // check x
                p[1] < box_min[1] || p[1] > box_max[1] ||  // check y
                p[2] < box_min[2] || p[2] > box_max[2]) {  // ray start point is outside the box
                if (t_min >= 0) {                          // forward intersection
                    d1 = t_min;                            // first intersection point
                    d2 = t_max;                            // second intersection point
                } else {                                   // backward intersection
                    d1 = t_max;                            // first intersection point
                    d2 = t_min;                            // second intersection point
                }
            } else {         // ray start point is inside the box
                d1 = t_max;  // forward intersection
                d2 = t_min;  // backward intersection
            }
        }
    }

    void
    ComputeIntersectionBetweenLineAndEllipse2D(
        const double x0,
        const double y0,
        const double x1,
        const double y1,
        const double a,
        const double b,
        double &lam1,
        double &lam2,
        bool &intersected) {

        // ellipse equation: (x - cx)^2 / a^2 + (y - cy)^2 / b^2 = 1
        // line equation: x = x0 + lam * (x1 - x0), y = y0 + lam * (y1 - y0)
        // substitute line equation into ellipse equation and solve for lam

        const double a_sq = a * a;
        const double b_sq = b * b;
        const double x_diff = x0 - x1;
        const double y_diff = y0 - y1;
        const double x_diff_sq = x_diff * x_diff;
        const double y_diff_sq = y_diff * y_diff;
        const double cross_term = x0 * y1 - x1 * y0;
        const double cross_term_sq = cross_term * cross_term;
        double tmp0 = a_sq * y_diff_sq + b_sq * x_diff_sq;

        if (const double tmp1 = tmp0 - cross_term_sq; tmp1 < 0) {  // no intersection
            intersected = false;
        } else {
            const double tmp2 = a_sq * y0 * y_diff + b_sq * x0 * x_diff;
            const double tmp3 = std::sqrt(tmp1) * a * b;
            tmp0 = 1.0 / tmp0;
            lam1 = (-tmp3 + tmp2) * tmp0;
            lam2 = (tmp3 + tmp2) * tmp0;
            intersected = true;
        }

        /*
         * The following code is equivalent to the above code. Although its mathematical form is more compact, but uses 13 multiplications and 3 divisions.
         * The above code uses 17 multiplications and 1 division. 1 division cost is about 6 multiplications.

        a = 1.0 / a;
        b = 1.0 / b;

        x0 *= a;
        y0 *= b;
        x1 *= a;
        y1 *= b;
        const double x_diff = x0 - x1;
        const double y_diff = y0 - y1;
        const double x_diff_sq = x_diff * x_diff;
        const double y_diff_sq = y_diff * y_diff;
        const double cross_term = x0 * y1 - x1 * y0;
        const double cross_term_sq = cross_term * cross_term;
        double tmp0 = x_diff_sq + y_diff_sq;
        if (const double tmp1 = tmp0 - cross_term_sq; tmp1 < 0) {
            intersected = false;
        } else {
            const double tmp2 = x0 * x_diff + y0 * y_diff;
            const double tmp3 = std::sqrt(tmp1);
            tmp0 = 1.0 / tmp0;
            lam1 = (-tmp3 + tmp2) * tmp0;
            lam2 = (tmp3 + tmp2) * tmp0;
            intersected = true;
        }
        */
    }

    void
    ComputeIntersectionBetweenLineAndEllipsoid3D(
        const double x0,
        const double y0,
        const double z0,
        const double x1,
        const double y1,
        const double z1,
        const double a,
        const double b,
        const double c,
        double &lam1,
        double &lam2,
        bool &intersected) {

        // ellipse equation: (x - cx)^2 / a^2 + (y - cy)^2 / b^2 + (z - cz)^2 / c^2 = 1
        // line equation: x = x0 + lam * (x1 - x0), y = y0 + lam * (y1 - y0), z = z0 + lam * (z1 - z0)
        // substitute line equation into ellipse equation and solve for lam

        const double a_sq = a * a;
        const double b_sq = b * b;
        const double c_sq = c * c;

        const double a_sq_b_sq = a_sq * b_sq;
        const double a_sq_c_sq = a_sq * c_sq;
        const double b_sq_c_sq = b_sq * c_sq;

        const double x_diff = x0 - x1;
        const double y_diff = y0 - y1;
        const double z_diff = z0 - z1;
        const double x_diff_sq = x_diff * x_diff;
        const double y_diff_sq = y_diff * y_diff;
        const double z_diff_sq = z_diff * z_diff;

        const double cross_x = y0 * z1 - y1 * z0;
        const double cross_y = z0 * x1 - z1 * x0;
        const double cross_z = x0 * y1 - x1 * y0;
        const double cross_term_sq = a_sq * cross_x * cross_x + b_sq * cross_y * cross_y + c_sq * cross_z * cross_z;

        double tmp0 = b_sq_c_sq * x_diff_sq + a_sq_c_sq * y_diff_sq + a_sq_b_sq * z_diff_sq;

        if (const double tmp1 = tmp0 - cross_term_sq; tmp1 < 0) {  // no intersection
            intersected = false;
        } else {
            const double tmp2 = b_sq_c_sq * x0 * x_diff + a_sq_c_sq * y0 * y_diff + a_sq_b_sq * z0 * z_diff;
            const double tmp3 = std::sqrt(tmp1) * a * b * c;
            tmp0 = 1.0 / tmp0;
            lam1 = (-tmp3 + tmp2) * tmp0;
            lam2 = (tmp3 + tmp2) * tmp0;
            intersected = true;
        }
    }

}  // namespace erl::geometry
