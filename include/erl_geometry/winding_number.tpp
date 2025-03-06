#pragma once

namespace erl::geometry {

    template<typename Dtype>
    int
    WindingNumber(const Eigen::Ref<const Eigen::Vector2<Dtype>> &p, const Eigen::Ref<const Eigen::Matrix2X<Dtype>> &vertices) {

        auto is_left = [](const Dtype &x, const Dtype &y, const Dtype &x_1, const Dtype &y_1, const Dtype &x_2, const Dtype &y_2) {
            // suppose vertex1.y() < vertex2.y()
            return ((x_2 - x_1) * (y - y_1) - (x - x_1) * (y_2 - y_1)) > 0.;
        };

        const long num_vertices = vertices.cols();
        const Dtype kx = p.x();
        const Dtype ky = p.y();
        int wn = 0;
        for (auto i = 0; i < num_vertices - 1; ++i) {
            const Dtype kx1 = vertices(0, i);
            const Dtype ky1 = vertices(1, i);
            const Dtype kx2 = vertices(0, i + 1);
            const Dtype ky2 = vertices(1, i + 1);

            if (ky1 <= ky) {
                if ((ky2 > ky) && is_left(kx, ky, kx1, ky1, kx2, ky2)) { wn++; }
            } else if ((ky2 <= ky) && is_left(kx, ky, kx2, ky2, kx1, ky1)) {
                wn--;
            }
        }
        // the last vertex and the first vertex composite the last polygon segment
        const Dtype kx1 = vertices(0, num_vertices - 1);
        const Dtype ky1 = vertices(1, num_vertices - 1);
        const Dtype kx2 = vertices(0, 0);
        const Dtype ky2 = vertices(1, 0);

        if (ky1 <= ky) {
            if ((ky2 > ky) && is_left(kx, ky, kx1, ky1, kx2, ky2)) { wn++; }
        } else if ((ky2 <= ky) && is_left(kx, ky, kx2, ky2, kx1, ky1)) {
            wn--;
        }

        return wn;  // positive winding number <==> inside the polygon defined by `vertices`
    }
}  // namespace erl::geometry
