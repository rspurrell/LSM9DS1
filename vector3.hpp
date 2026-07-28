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
        T x{0};
        /// Y-axis component.
        T y{0};
        /// Z-axis component.
        T z{0};

        T& roll  = x;
        T& pitch = y;
        T& yaw   = z;

        constexpr Vector3() = default;
        constexpr Vector3(T x, T y, T z) : x(x), y(y), z(z) {}
        constexpr Vector3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}

        constexpr Vector3& operator=(const Vector3& v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
            return *this;
        }
    };
}