#include "erl_geometry/euler_angle.hpp"

#include "erl_common/logging.hpp"

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

}  // namespace erl::geometry
