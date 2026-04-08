#pragma once

#include <cstdint>
#include <type_traits>

namespace erl::geometry::detail {

    template<typename T, typename = void>
    struct has_update_color : std::false_type {};

    template<typename T>
    struct has_update_color<
        T,
        std::void_t<decltype(std::declval<T>().UpdateColor(
            std::declval<uint8_t>(),
            std::declval<uint8_t>(),
            std::declval<uint8_t>(),
            std::declval<uint8_t>()))>> : std::true_type {};

    template<typename T>
    inline constexpr bool has_update_color_v = has_update_color<T>::value;

    template<typename T, typename = void>
    struct has_set_color : std::false_type {};

    template<typename T>
    struct has_set_color<
        T,
        std::void_t<decltype(std::declval<T>().SetColor(
            std::declval<uint8_t>(),
            std::declval<uint8_t>(),
            std::declval<uint8_t>(),
            std::declval<uint8_t>()))>> : std::true_type {};

    template<typename T>
    inline constexpr bool has_set_color_v = has_set_color<T>::value;

}  // namespace erl::geometry::detail
