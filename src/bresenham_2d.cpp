#include "erl_geometry/bresenham_2d.hpp"

namespace erl::geometry {

    Eigen::Matrix2Xi
    Bresenham2D(const Eigen::Ref<const Eigen::Vector2i> &start, const Eigen::Ref<const Eigen::Vector2i> &end) {

        Eigen::Vector2i delta = (end - start).array().abs();
        const int num_vertices = delta.maxCoeff() + 1;
        Eigen::Matrix2Xi vertices(2, num_vertices);

        if (std::abs(delta.y()) == 0) {
            const int sign = delta.x() > 0 ? 1 : -1;
            int x = start.x();
            for (int i = 0; i < num_vertices; ++i, x += sign) {
                vertices(0, i) = x;
                vertices(1, i) = start.y();
            }
            return vertices;
        }

        if (std::abs(delta.x()) == 0) {
            const int sign = delta.y() > 0 ? 1 : -1;
            int y = start.y();
            for (int i = 0; i < num_vertices; ++i, y += sign) {
                vertices(0, i) = start.x();
                vertices(1, i) = y;
            }
            return vertices;
        }

        const int xi = delta.x() >= 0 ? 1 : -1;
        const int yi = delta.y() >= 0 ? 1 : -1;

        if (std::abs(delta.y()) < std::abs(delta.x())) {
            // to avoid swap:
            // xi    yi    kA    kB
            //  1     1    kA    kB
            //  1    -1   -kA    kB
            // -1     1    kA   -kB
            // -1    -1   -kA   -kB
            // plot line of slop between (-1, 1)
            const int ka = delta.y() * yi;
            const int kb = -delta.x() * xi;
            const int a = ka * 2;
            int d = a + kb;
            const int b = d + kb;
            int y = start.y();

            int i = 0;
            for (int x = start.x(); i < num_vertices; x += xi, ++i) {
                vertices.col(i) << x, y;

                if (d >= 0) {
                    y += yi;
                    d += b;
                } else {
                    d += a;
                }
            }
        } else {
            // plot line of slop out of (-1, 1)
            const int ka = delta.x() * xi;
            const int kb = -delta.y() * yi;
            const int a = ka * 2;
            int d = a + kb;
            const int b = d + kb;
            int x = start.x();

            int i = 0;
            for (int y = start.y(); i < num_vertices; y += yi, ++i) {
                vertices.col(i) << x, y;

                if (d >= 0) {
                    x += xi;
                    d += b;
                } else {
                    d += a;
                }
            }
        }

        return vertices;
    }

    Eigen::Matrix2Xi
    Bresenham2D(const Eigen::Ref<const Eigen::Vector2i> &start, const Eigen::Ref<const Eigen::Vector2i> &end, const std::function<bool(int, int)> &stop) {

        const Eigen::Vector2i delta = end - start;
        const int num_vertices = delta.cwiseAbs().maxCoeff() + 1;
        Eigen::Matrix2Xi vertices(2, num_vertices);

        if (std::abs(delta.y()) == 0) {
            const int sign = delta.x() > 0 ? 1 : -1;
            int x = start.x();
            const int y = start.y();
            int i = 0;
            for (; !stop(x, y) && i < num_vertices; ++i, x += sign) {
                vertices(0, i) = x;
                vertices(1, i) = start.y();
            }
            if (i < num_vertices) {
                vertices(0, i) = x;
                vertices(1, i) = start.y();
                vertices.conservativeResize(2, i + 1);
            }
            return vertices;
        }

        if (std::abs(delta.x()) == 0) {
            const int sign = delta.y() > 0 ? 1 : -1;
            const int x = start.x();
            int y = start.y();
            int i = 0;
            for (; !stop(x, y) && i < num_vertices; ++i, y += sign) {
                vertices(0, i) = start.x();
                vertices(1, i) = y;
            }
            if (i < num_vertices) {
                vertices(0, i) = start.x();
                vertices(1, i) = y;
                vertices.conservativeResize(2, i + 1);
            }
            return vertices;
        }

        const int xi = delta.x() >= 0 ? 1 : -1;
        const int yi = delta.y() >= 0 ? 1 : -1;

        if (std::abs(delta.y()) < std::abs(delta.x())) {
            // to avoid swap:
            // xi    yi    A    B
            //  1     1    A    B
            //  1    -1   -A    B
            // -1     1    A   -B
            // -1    -1   -A   -B
            // plot line of slop between (-1, 1)
            const int ka = delta.y() * yi;
            const int kb = -delta.x() * xi;
            const int a = ka * 2;
            int d = a + kb;
            const int b = d + kb;
            int x = start.x();
            int y = start.y();

            int i = 0;
            for (; !stop(x, y) && i < num_vertices; x += xi, ++i) {
                vertices.col(i) << x, y;

                if (d >= 0) {
                    y += yi;
                    d += b;
                } else {
                    d += a;
                }
            }
            if (i < num_vertices) {
                vertices.col(i) << x, y;
                vertices.conservativeResize(2, i + 1);
            }
        } else {
            // plot line of slop out of (-1, 1)
            const int ka = delta.x() * xi;
            const int kb = -delta.y() * yi;
            const int a = ka * 2;
            int d = a + kb;
            const int b = d + kb;
            int x = start.x();
            int y = start.y();

            int i = 0;
            for (; !stop(x, y) && i < num_vertices; y += yi, ++i) {
                vertices.col(i) << x, y;

                if (d >= 0) {
                    x += xi;
                    d += b;
                } else {
                    d += a;
                }
            }
            if (i < num_vertices) {
                vertices.col(i) << x, y;
                vertices.conservativeResize(2, i + 1);
            }
        }

        return vertices;
    }

    Eigen::Matrix2Xi
    ComputePixelsOfPolygonContour(const Eigen::Ref<const Eigen::Matrix2Xi> &polygon_vertices) {
        Eigen::Matrix2Xi pixels;
        const long num_vertices = polygon_vertices.cols();
        for (int i = 0; i < num_vertices; ++i) {
            Eigen::Matrix2Xi line = Bresenham2D(polygon_vertices.col(i), polygon_vertices.col((i + 1) % num_vertices));
            pixels.conservativeResize(2, pixels.cols() + line.cols());
            pixels.rightCols(line.cols()) = line;
        }
        return pixels;
    }
}  // namespace erl::geometry
