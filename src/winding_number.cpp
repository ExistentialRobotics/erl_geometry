#include "erl_geometry/winding_number.hpp"

namespace erl::geometry {

    static inline bool
    IsLeft(const double &x, const double &y, const double &x_1, const double &y_1, const double &x_2, const double &y_2) {
        // suppose vertex1.y() < vertex2.y()
        return ((x_2 - x_1) * (y - y_1) - (x - x_1) * (y_2 - y_1)) > 0.;
    }

    int
    WindingNumber(const Eigen::Ref<const Eigen::Vector2d> &p, const Eigen::Ref<const Eigen::Matrix2Xd> &vertices) {

        auto num_vertices = vertices.cols();
        const auto &kX = p.x();
        const auto &kY = p.y();
        int wn = 0;
        for (auto i = 0; i < num_vertices - 1; ++i) {
            const auto &kX1 = vertices(0, i);
            const auto &kY1 = vertices(1, i);
            const auto &kX2 = vertices(0, i + 1);
            const auto &kY2 = vertices(1, i + 1);

            if (kY1 <= kY) {
                if ((kY2 > kY) && IsLeft(kX, kY, kX1, kY1, kX2, kY2)) { wn++; }
            } else if ((kY2 <= kY) && IsLeft(kX, kY, kX2, kY2, kX1, kY1)) {
                wn--;
            }
        }
        // the last vertex and the first vertex composite the last polygon segment
        const auto &kX1 = vertices(0, num_vertices - 1);
        const auto &kY1 = vertices(1, num_vertices - 1);
        const auto &kX2 = vertices(0, 0);
        const auto &kY2 = vertices(1, 0);

        if (kY1 <= kY) {
            if ((kY2 > kY) && IsLeft(kX, kY, kX1, kY1, kX2, kY2)) { wn++; }
        } else if ((kY2 <= kY) && IsLeft(kX, kY, kX2, kY2, kX1, kY1)) {
            wn--;
        }

        return wn;  // positive winding number <==> inside the polygon defined by `vertices`
    }
}  // namespace erl::geometry
