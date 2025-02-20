#pragma once

namespace erl::geometry {

    extern bool initialized;

    /**
     * @brief Initialize the library.
     */
    bool
    Init();

    inline const static bool kAutoInitialized = Init();

}  // namespace erl::geometry
