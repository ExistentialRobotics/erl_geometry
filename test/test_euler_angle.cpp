#include "erl_common/test_helper.hpp"
#include "erl_geometry/euler_angle.hpp"

template<typename T>
inline Eigen::Matrix<T, 3, 3>
Rot3D(T roll, T pitch, T yaw, const std::string& axes = "rzyx") {
    typename Eigen::Matrix<T, 3, 3>::Index i, j, k;
    bool parity, repetition, frame;

    if (axes == "sxyz") {
        i = 0;
        parity = false;
        repetition = false;
        frame = false;
    }
    if (axes == "sxyx") {
        i = 0;
        parity = false;
        repetition = true;
        frame = false;
    }
    if (axes == "sxzy") {
        i = 0;
        parity = true;
        repetition = false;
        frame = false;
    }

    if (axes == "sxzx") {
        i = 0;
        parity = true;
        repetition = true;
        frame = false;
    }
    if (axes == "syzx") {
        i = 1;
        parity = false;
        repetition = false;
        frame = false;
    }
    if (axes == "syzy") {
        i = 1;
        parity = false;
        repetition = true;
        frame = false;
    }

    if (axes == "syxz") {
        i = 1;
        parity = true;
        repetition = false;
        frame = false;
    }
    if (axes == "syxy") {
        i = 1;
        parity = true;
        repetition = true;
        frame = false;
    }
    if (axes == "szxy") {
        i = 2;
        parity = false;
        repetition = false;
        frame = false;
    }

    if (axes == "szxz") {
        i = 2;
        parity = false;
        repetition = true;
        frame = false;
    }
    if (axes == "szyx") {
        i = 2;
        parity = true;
        repetition = false;
        frame = false;
    }
    if (axes == "szyz") {
        i = 2;
        parity = true;
        repetition = true;
        frame = false;
    }

    if (axes == "rzyx") {
        i = 0;
        parity = false;
        repetition = false;
        frame = true;
    }
    if (axes == "rxyx") {
        i = 0;
        parity = false;
        repetition = true;
        frame = true;
    }
    if (axes == "ryzx") {
        i = 0;
        parity = true;
        repetition = false;
        frame = true;
    }

    if (axes == "rxzx") {
        i = 0;
        parity = true;
        repetition = true;
        frame = true;
    }
    if (axes == "rxzy") {
        i = 1;
        parity = false;
        repetition = false;
        frame = true;
    }
    if (axes == "ryzy") {
        i = 1;
        parity = false;
        repetition = true;
        frame = true;
    }

    if (axes == "rzxy") {
        i = 1;
        parity = true;
        repetition = false;
        frame = true;
    }
    if (axes == "ryxy") {
        i = 1;
        parity = true;
        repetition = true;
        frame = true;
    }
    if (axes == "ryxz") {
        i = 2;
        parity = false;
        repetition = false;
        frame = true;
    }

    if (axes == "rzxz") {
        i = 2;
        parity = false;
        repetition = true;
        frame = true;
    }
    if (axes == "rxyz") {
        i = 2;
        parity = true;
        repetition = false;
        frame = true;
    }
    if (axes == "rzyz") {
        i = 2;
        parity = true;
        repetition = true;
        frame = true;
    }

    switch (i + parity) {
        case 0:
            j = 1;
            break;
        case 1:
            j = 2;
            break;
        case 2:
            j = 0;
            break;
        case 3:
            j = 1;
            break;
    }
    switch (i - parity + 1) {
        case 0:
            k = 1;
            break;
        case 1:
            k = 2;
            break;
        case 2:
            k = 0;
            break;
        case 3:
            k = 1;
            break;
    }

    if (frame) {
        T tmp = yaw;
        yaw = roll;
        roll = tmp;
    }
    if (parity) {
        roll = -roll;
        pitch = -pitch;
        yaw = -yaw;
    }

    T sr = std::sin(roll), sp = std::sin(pitch), sy = std::sin(yaw);
    T cr = std::cos(roll), cp = std::cos(pitch), cy = std::cos(yaw);
    T cycr = cy * cr, cysr = cy * sr;
    T sycr = sy * cr, sysr = sy * sr;
    Eigen::Matrix<T, 3, 3> r;
    if (repetition) {
        r(i, i) = cp, r(i, j) = sp * sy, r(i, k) = sp * cy;
        r(j, i) = sp * sr, r(j, j) = -cp * sysr + cycr, r(j, k) = -cp * cysr - sycr;
        r(k, i) = -sp * cr, r(k, j) = cp * sycr + cysr, r(k, k) = cp * cycr - sysr;
    } else {
        r(i, i) = cp * cr, r(i, j) = sp * sycr - cysr, r(i, k) = sp * cycr + sysr;
        r(j, i) = cp * sr, r(j, j) = sp * sysr + cycr, r(j, k) = sp * cysr - sycr;
        r(k, i) = -sp, r(k, j) = cp * sy, r(k, k) = cp * cy;
    }

    return r;
}

int
main() {

    const char* orders[] = {"sxyz", "sxyx", "sxzy", "sxzx", "syzx", "syzy", "syxz", "syxy", "szxy", "szxz", "szyx", "szyz",
                            "rzyx", "rxyx", "ryzx", "rxzx", "rxzy", "ryzy", "rzxy", "ryxy", "ryxz", "rzxz", "rxyz", "rzyz"};

    for (int i = 0; i < 3; ++i) {

        Eigen::Vector3d abc = Eigen::Vector3d::Random().array() * M_PI * 2 - M_PI;
        std::cout << "test round " << i << std::endl;
        Eigen::Matrix3d r_gt, r_ans;
        for (auto order: orders) {

            erl::common::ReportTime<std::chrono::nanoseconds>("Rot3D", 5, false, [&]() { r_gt = Rot3D(abc[0], abc[1], abc[2], order); });
            erl::common::ReportTime<std::chrono::nanoseconds>("EulerToRotation3D", 5, false, [&]() {
                r_ans = erl::geometry::EulerToRotation3D(abc[0], abc[1], abc[2], erl::geometry::GetEulerAngleOrder(order));  // 50% faster!
            });

            erl::common::CheckAnswers(order, r_ans, r_gt);
        }
        std::cout << std::endl;
    }

    return 0;
}
