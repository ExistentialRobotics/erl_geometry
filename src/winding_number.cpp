#include "erl_geometry/winding_number.hpp"

namespace erl::geometry {

    static bool
    IsLeft(const double &x, const double &y, const double &x_1, const double &y_1, const double &x_2, const double &y_2) {
        // suppose vertex1.y() < vertex2.y()
        return ((x_2 - x_1) * (y - y_1) - (x - x_1) * (y_2 - y_1)) > 0.;
    }

    int
    WindingNumber(const Eigen::Ref<const Eigen::Vector2d> &p, const Eigen::Ref<const Eigen::Matrix2Xd> &vertices) {

        const long num_vertices = vertices.cols();
        const double kx = p.x();
        const double ky = p.y();
        int wn = 0;
        for (auto i = 0; i < num_vertices - 1; ++i) {
            const double kx1 = vertices(0, i);
            const double ky1 = vertices(1, i);
            const double kx2 = vertices(0, i + 1);
            const double ky2 = vertices(1, i + 1);

            if (ky1 <= ky) {
                if ((ky2 > ky) && IsLeft(kx, ky, kx1, ky1, kx2, ky2)) { wn++; }
            } else if ((ky2 <= ky) && IsLeft(kx, ky, kx2, ky2, kx1, ky1)) {
                wn--;
            }
        }
        // the last vertex and the first vertex composite the last polygon segment
        const double kx1 = vertices(0, num_vertices - 1);
        const double ky1 = vertices(1, num_vertices - 1);
        const double kx2 = vertices(0, 0);
        const double ky2 = vertices(1, 0);

        if (ky1 <= ky) {
            if ((ky2 > ky) && IsLeft(kx, ky, kx1, ky1, kx2, ky2)) { wn++; }
        } else if ((ky2 <= ky) && IsLeft(kx, ky, kx2, ky2, kx1, ky1)) {
            wn--;
        }

        return wn;  // positive winding number <==> inside the polygon defined by `vertices`
    }
}  // namespace erl::geometry
