#include "erl_common/test_helper.hpp"
#include "erl_geometry/intersection.hpp"

TEST(IntersectionLineEllipse, OneIntersection) {
    double lam1, lam2;
    bool intersected;

    erl::geometry::ComputeIntersectionBetweenLineAndEllipse2D<
        double>(-3, 1, 3, 1, 2, 1, lam1, lam2, intersected);
    EXPECT_TRUE(intersected);
    EXPECT_NEAR(lam1, 0.5, 1e-6);
    EXPECT_NEAR(lam2, 0.5, 1e-6);
}

TEST(IntersectionLineEllipse, TwoIntersections) {
    double lam1, lam2;
    bool intersected;

    erl::geometry::ComputeIntersectionBetweenLineAndEllipse2D<
        double>(-3, 0, 3, 0, 2, 1, lam1, lam2, intersected);
    EXPECT_TRUE(intersected);
    EXPECT_NEAR(lam1, 1.0 / 6.0, 1e-6);
    EXPECT_NEAR(lam2, 5.0 / 6.0, 1e-6);
}

TEST(IntersectionLineEllipse, NoIntersection) {
    double lam1, lam2;
    bool intersected;

    erl::geometry::ComputeIntersectionBetweenLineAndEllipse2D<
        double>(-3, 1.1, 3, 1, 2, 1, lam1, lam2, intersected);
    EXPECT_FALSE(intersected);
}
