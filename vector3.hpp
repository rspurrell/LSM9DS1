#pragma once

// Provides fixed-width integer types.
#include <cstdint>

namespace Math
{
    /**
     * @brief Represents a three-dimensional vector.
     *
     * This structure is used throughout the project for sensors and
     * mathematical operations involving X, Y, and Z components.
     *
     * @tparam T Component type.
     */
    template<typename T>
    struct Vector3
    {
        /// X-axis component.
        T x;

        /// Y-axis component.
        T y;

        /// Z-axis component.
        T z;
    };
}