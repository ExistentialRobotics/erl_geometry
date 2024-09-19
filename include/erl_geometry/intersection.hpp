#pragma once
#include "erl_common/eigen.hpp"

namespace erl::geometry {

    /**
     * find the nearest point on the segment (x_1, y_1) -- (x_2, y_2) to point (x_0, y_0)
     * line1: (x_1, y_1), (x_2, y_2)
     * line2: (x_0, y_0) of direction d=(y_2-y_1, x_1-x_2)
     * lam * (x_1, y_1) + (1-lam) * (x_2, y_2) == (x_0, y_0) + m_tree_ * d
     * if intersected, lam in (0, 1); |m_tree_| is the distance
     * if lam < 0, (x_2, y_2) is the closest point; if lam > 1, (x_1, y_1) is the closest point.
     *
     * @param x0: point x coordinate
     * @param y0: point y coordinate
     * @param x1: x coordinate of line vertex 1
     * @param y1: y coordinate of line vertex 1
     * @param x2: x coordinate of line vertex 2
     * @param y2: y coordinate of line vertex 2
     * @return
     */
    double
    ComputeNearestDistanceFromPointToLineSegment2D(double x0, double y0, double x1, double y1, double x2, double y2);

    void
    ComputeIntersectionBetweenTwoLines2D(
        const Eigen::Vector2d &p00,
        const Eigen::Vector2d &p01,
        const Eigen::Vector2d &p10,
        const Eigen::Vector2d &p11,
        double &lam1,
        double &lam2,
        bool &intersected);

    void
    ComputeIntersectionBetweenTwoLines3D(
        const Eigen::Vector3d &p00,
        const Eigen::Vector3d &p01,
        const Eigen::Vector3d &p10,
        const Eigen::Vector3d &p11,
        double &lam1,
        double &lam2,
        bool &intersected);

    /**
     * find the intersection between ray [p_0, d] and segment [p_1, p_2]
     * @param p0: ray start point
     * @param v: ray direction, assumed normalized
     * @param p1: point 1 on the line
     * @param p2: point 2 on the line
     * @param lam: the intersection point is lam * p_1 + (1 - lam) * p_2
     * @param dist: distance from p_0 to the line along direction d
     * @param intersected: whether the ray intersects the segment
     */
    void
    ComputeIntersectionBetweenRayAndLine2D(
        const Eigen::Vector2d &p0,
        const Eigen::Vector2d &v,
        const Eigen::Vector2d &p1,
        const Eigen::Vector2d &p2,
        double &lam,
        double &dist,
        bool &intersected);

    /**
     * find the intersection between ray [p, r] and axis-aligned bounding box [box_min, box_max]
     * @param p            ray start point
     * @param v_inv        1 / r considering performance of multiple computations with the same ray direction
     * @param box_min      box min point
     * @param box_max      box max point
     * @param d1           output distance from p to the first intersection point (closest one if p is outside the box, forward one if p is inside the box)
     * @param d2           output distance from p to the second intersection point (farthest one if p is outside the box, backward one if p is inside the box)
     * @param intersected  output whether the ray intersects the box
     * @param is_inside    output whether the ray starts inside the box
     *
     * @refitem https://tavianator.com/fast-branchless-raybounding-box-intersections/
     * @refitem https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
     * @refitem https://en.wikipedia.org/wiki/Cohen%E2%80%93Sutherland_algorithm
     * @refitem https://en.wikipedia.org/wiki/Liang%E2%80%93Barsky_algorithm
     * @refitem https://en.wikipedia.org/wiki/Cyrus%E2%80%93Beck_algorithm
     * @refitem https://en.wikipedia.org/wiki/Nicholl%E2%80%93Lee%E2%80%93Nicholl_algorithm
     * @refitem https://en.wikipedia.org/wiki/Line_clipping#Fast_clipping
     */
    void
    ComputeIntersectionBetweenRayAndAabb2D(
        const Eigen::Vector2d &p,
        const Eigen::Vector2d &v_inv,
        const Eigen::Vector2d &box_min,
        const Eigen::Vector2d &box_max,
        double &d1,
        double &d2,
        bool &intersected,
        bool &is_inside);

    /**
     * find the intersection between ray [p, r] and axis-aligned bounding box [box_min, box_max]
     * @param p            ray start point
     * @param r_inv        1 / r considering performance of multiple computations with the same ray direction
     * @param box_min      box min point
     * @param box_max      box max point
     * @param d1           output distance from p to the first intersection point (closest one if p is outside the box, forward one if p is inside the box)
     * @param d2           output distance from p to the second intersection point (farthest one if p is outside the box, backward one if p is inside the box)
     * @param intersected  output whether the ray intersects the box
     */
    void
    ComputeIntersectionBetweenRayAndAabb3D(
        const Eigen::Vector3d &p,
        const Eigen::Vector3d &r_inv,
        const Eigen::Vector3d &box_min,
        const Eigen::Vector3d &box_max,
        double &d1,
        double &d2,
        bool &intersected,
        bool &is_inside);

    /**
     * Compute the intersection between a line and an ellipse assumed to be centered at the origin and axis-aligned.
     * @param x0 x coordinate of the p0 of the line
     * @param y0 y coordinate of the p0 of the line
     * @param x1 x coordinate of the p1 of the line
     * @param y1 y coordinate of the p1 of the line
     * @param a x-axis radius of the ellipse
     * @param b y-axis radius of the ellipse
     * @param lam1 output the first intersection point: lam1 * (x0, y0) + (1 - lam1) * (x1, y1)
     * @param lam2 output the second intersection point: lam2 * (x0, y0) + (1 - lam2) * (x1, y1), lam1 < lam2
     * @param intersected output whether the line intersects the ellipse
     */
    void
    ComputeIntersectionBetweenLineAndEllipse2D(double x0, double y0, double x1, double y1, double a, double b, double &lam1, double &lam2, bool &intersected);

    /**
     * Compute the intersection between a line and an ellipse assumed to be centered at the origin and axis-aligned.
     * @param x0 x coordinate of the p0 of the line
     * @param y0 y coordinate of the p0 of the line
     * @param z0 z coordinate of the p0 of the line
     * @param x1 x coordinate of the p1 of the line
     * @param y1 y coordinate of the p1 of the line
     * @param z1 z coordinate of the p1 of the line
     * @param a x-axis radius of the ellipsoid
     * @param b y-axis radius of the ellipsoid
     * @param c z-axis radius of the ellipsoid
     * @param lam1 output the first intersection point: lam1 * (x0, y0, z0) + (1 - lam1) * (x1, y1, z1)
     * @param lam2 output the second intersection point: lam2 * (x0, y0, z0) + (1 - lam2) * (x1, y1, z1), lam1 < lam2
     * @param intersected output whether the line intersects the ellipsoid
     */
    void
    ComputeIntersectionBetweenLineAndEllipsoid3D(
        double x0,
        double y0,
        double z0,
        double x1,
        double y1,
        double z1,
        double a,
        double b,
        double c,
        double &lam1,
        double &lam2,
        bool &intersected);
}  // namespace erl::geometry
