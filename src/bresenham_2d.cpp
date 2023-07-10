#include "erl_geometry/bresenham_2d.hpp"

namespace erl::geometry {

    Eigen::Matrix2Xi
    Bresenham2D(const Eigen::Ref<const Eigen::Vector2i> &start, const Eigen::Ref<const Eigen::Vector2i> &end) {

        Eigen::Vector2i delta = (end - start).array().abs();
        auto num_vertices = delta.maxCoeff() + 1;
        Eigen::Matrix2Xi vertices(2, num_vertices);

        if (std::abs(delta.y()) == 0) {
            auto sign = delta.x() > 0 ? 1 : -1;
            auto x = start.x();
            for (int i = 0; i < num_vertices; ++i, x += sign) {
                vertices(0, i) = x;
                vertices(1, i) = start.y();
            }
            return vertices;
        }

        if (std::abs(delta.x()) == 0) {
            auto sign = delta.y() > 0 ? 1 : -1;
            auto y = start.y();
            for (int i = 0; i < num_vertices; ++i, y += sign) {
                vertices(0, i) = start.x();
                vertices(1, i) = y;
            }
            return vertices;
        }

        auto xi = delta.x() >= 0 ? 1 : -1;
        auto yi = delta.y() >= 0 ? 1 : -1;

        if (std::abs(delta.y()) < std::abs(delta.x())) {
            // to avoid swap:
            // xi    yi    kA    kB
            //  1     1    kA    kB
            //  1    -1   -kA    kB
            // -1     1    kA   -kB
            // -1    -1   -kA   -kB
            // plot line of slop between (-1, 1)
            const auto kA = delta.y() * yi;
            const auto kB = -delta.x() * xi;
            auto a = kA * 2;
            auto d = a + kB;
            auto b = d + kB;
            auto y = start.y();

            auto i = 0;
            for (auto x = start.x(); i < num_vertices; x += xi, ++i) {
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
            const auto kA = delta.x() * xi;
            const auto kB = -delta.y() * yi;
            auto a = kA * 2;
            auto d = a + kB;
            auto b = d + kB;
            auto x = start.x();

            auto i = 0;
            for (auto y = start.y(); i < num_vertices; y += yi, ++i) {
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

        auto delta = end - start;
        auto num_vertices = delta.cwiseAbs().maxCoeff() + 1;
        Eigen::Matrix2Xi vertices(2, num_vertices);

        if (std::abs(delta.y()) == 0) {
            auto sign = delta.x() > 0 ? 1 : -1;
            int x = start.x();
            int y = start.y();
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
            auto sign = delta.y() > 0 ? 1 : -1;
            int x = start.x();
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

        int xi = delta.x() >= 0 ? 1 : -1;
        int yi = delta.y() >= 0 ? 1 : -1;

        if (std::abs(delta.y()) < std::abs(delta.x())) {
            // to avoid swap:
            // xi    yi    A    B
            //  1     1    A    B
            //  1    -1   -A    B
            // -1     1    A   -B
            // -1    -1   -A   -B
            // plot line of slop between (-1, 1)
            const auto kA = delta.y() * yi;
            const auto kB = -delta.x() * xi;
            auto a = kA * 2;
            auto d = a + kB;
            auto b = d + kB;
            auto x = start.x();
            auto y = start.y();

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
            const auto kA = delta.x() * xi;
            const auto kB = -delta.y() * yi;
            auto a = kA * 2;
            auto d = a + kB;
            auto b = d + kB;
            auto x = start.x();
            auto y = start.y();

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
        long num_vertices = polygon_vertices.cols();
        for (int i = 0; i < num_vertices; ++i) {
            auto start = polygon_vertices.col(i);
            auto end = polygon_vertices.col((i + 1) % num_vertices);
            auto line = Bresenham2D(start, end);
            pixels.conservativeResize(2, pixels.cols() + line.cols());
            pixels.rightCols(line.cols()) = line;
        }
        return pixels;
    }
}  // namespace erl::geometry
