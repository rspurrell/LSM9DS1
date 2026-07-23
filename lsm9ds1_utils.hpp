/******************************************************************************
 * @file    lsm9ds1_utils.hpp
 * @brief   Contains utility functions for converting raw sensor measurements.
 ******************************************************************************/

#pragma once

// Provides fixed-width integer types used by the raw sensor values.
#include <cstdint>

// Provides the Math::Vector3 type used for three-axis sensor measurements.
#include "vector3.hpp"

// Provides the accelerometer and gyroscope scale configuration types.
#include "lsm9ds1_registers.hpp"

/**
 * @namespace Convert
 * @brief Contains utility functions for converting raw sensor measurements.
 */
namespace Convert
{
    /**
     * @brief Converts raw accelerometer values into meters per second squared (m/s²).
     *
     * @param rawValues Raw signed 16-bit accelerometer values returned by the LSM9DS1.
     * @param scale Accelerometer full-scale range used when the sensor was configured.
     *
     * @return Accelerometer values expressed in meters per second squared.
     */
    Math::Vector3<float> ToMetersPerSecondSquared(const Math::Vector3<int16_t>& rawValues, Accelerometer::CtrlReg6XL::Scale scale);

    /**
     * @brief Converts raw accelerometer values into units of gravitational acceleration.
     *
     * @param rawValues Raw signed 16-bit accelerometer values returned by the LSM9DS1.
     * @param scale Accelerometer full-scale range used when the sensor was configured.
     *
     * @return Accelerometer values expressed in g.
     */
    Math::Vector3<float> ToGs(const Math::Vector3<int16_t>& rawValues, Accelerometer::CtrlReg6XL::Scale scale);

    /**
     * @brief Converts raw gyroscope values into degrees per second.
     *
     * @param rawValues Raw signed 16-bit gyroscope values returned by the LSM9DS1.
     * @param scale Gyroscope full-scale range used when the sensor was configured.
     *
     * @return Angular velocity values expressed in degrees per second.
     */
    Math::Vector3<float> ToDegreesPerSecond(const Math::Vector3<int16_t>& rawValues, Gyroscope::CtrlReg1G::Scale scale);

    /**
     * @brief Converts raw Magnetometer values into degrees per second.
     *
     * @param rawValues Raw signed 16-bit magnetometer values returned by the LSM9DS1.
     * @param scale Magnetometer full-scale range used when the sensor was configured.
     *
     * @return Magnetometer values expressed in milligauss.
     */
    Math::Vector3<float> ToMilliGauss(const Math::Vector3<int16_t>& rawValues, Magnetometer::CtrlReg2M::Scale scale);
}