/******************************************************************************
 * @file    mahony_ahrs.hpp
 * @brief   Registers and bit definitions for the STMicroelectronics LSM9DS1 9-DOF IMU.
 ******************************************************************************/

 #pragma once

#include <cmath>

#include "vector3.hpp"

namespace Math
{
    /**
     * @class MahonyAHRS
     * @brief An implementation of the Mahony Sensor Fusion algorithm for Attitude and Heading Reference Systems (AHRS).
     *
     * This class uses proportional-integral (PI) feedback control loops to fuse data from
     * a 9-DOF Inertial Measurement Unit (IMU) containing a triaxial gyroscope, accelerometer,
     * and magnetometer. It maintains an internal orientation estimate using a 32-bit float
     * quaternion representation (q_0, q_1, q_2, q_3) to completely avoid gimbal lock.
     *
     */
    class MahonyAHRS
    {
    public:
        /**
         * @brief Constructs a new Mahony Filter object with configurable feedback gains.
         *
         * Initializes the filter state with an identity quaternion (q = [1, 0, 0, 0]^T),
         * representing a perfectly level, north-facing initial target orientation.
         *
         * @param[in] proportionalGain Tuning parameter for error correction speed. Defaults to 1.0f.
         * @param[in] integralGain Tuning parameter for gyro bias correction accumulation. Defaults to 0.0f.
         */
        MahonyAHRS(float proportionalGain = 1.0f, float integralGain = 0.0f)
            : kp(proportionalGain), ki(integralGain), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
            eIntX(0.0f), eIntY(0.0f), eIntZ(0.0f)
        {}

        /**
         * @brief Updates the Mahony filter with new IMU and Magnetometer measurements.
         *
         * This method runs the sensor fusion algorithm, combining gyroscope, accelerometer,
         * and magnetometer readings to compute the updated 3D orientation quaternion.
         *
         * @param[in] gx Gyroscope X-axis angular velocity. Must be converted to \b radians \b per \b second \b (rad/s).
         * @param[in] gy Gyroscope Y-axis angular velocity. Must be converted to \b radians \b per \b second \b (rad/s).
         * @param[in] gz Gyroscope Z-axis angular velocity. Must be converted to \b radians \b per \b second \b (rad/s).
         *
         * @param[in] ax Accelerometer X-axis acceleration. Pass in \b g-force \b (g) or \b m/s² (normalized internally).
         * @param[in] ay Accelerometer Y-axis acceleration. Pass in \b g-force \b (g) or \b m/s² (normalized internally).
         * @param[in] az Accelerometer Z-axis acceleration. Pass in \b g-force \b (g) or \b m/s² (normalized internally).
         *
         * @param[in] mx Magnetometer X-axis flux density. Pass in \b Microteslas \b (µT) or \b Gauss (normalized internally).
         * @param[in] my Magnetometer Y-axis flux density. Pass in \b Microteslas \b (µT) or \b Gauss (normalized internally).
         *           Some sensors may need to invert this value to align axes.
         * @param[in] mz Magnetometer Z-axis flux density. Pass in \b Microteslas \b (µT) or \b Gauss (normalized internally).
         *           Some sensors may need to invert this value to align axes.
         *
         * @param[in] dt Delta time. The precise elapsed time since the last filter execution, measured in \b seconds \b (s).
         *
         * @note All parameters must be native 32-bit single-precision floating-point (`float`) primitives.
         * @return void This method updates internal class quaternion states directly.
         */
        void Update(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz, float dt);

        /**
         * @brief Computes standard Euler orientation angles from the internal estimated quaternion.
         *
         * Converts the current state matrix into human-readable rotations. Yaw outputs angle tracks
         * relative to Magnetic North.
         *
         */
        Math::Vector3<float> GetEulerAngles();

    private:
        /**
         * @brief Proportional gain coefficient K_p.
         *
         * Governs the closing rate and convergence speed of sensor errors. Higher values force
         * accelerometer and magnetometer inputs to correct gyroscope drift faster, but introduce
         * high-frequency vibration/noise to the output angles. Default value is 1.0f.
         */
        float kp = 1.0f;

        /**
         * @brief Integral gain coefficient K_i.
         *
         * Establishes a steady-state error accumulation loop used to continuously calculate and
         * subtract gyroscope bias/drift. Set to 0.0f if your gyro is pre-calibrated. Default value is 0.0f.
         */
        float ki = 0.0f;

        /** @brief Real scalar component q_0 of the orientation estimation quaternion. */
        float q0 = 1.0f;
        /** @brief First imaginary component q_1 of the orientation estimation quaternion (X-axis). */
        float q1 = 0.0f;
        /** @brief Second imaginary component q_2 of the orientation estimation quaternion (Y-axis). */
        float q2 = 0.0f;
        /** @brief Third imaginary component q_3 of the orientation estimation quaternion (Z-axis). */
        float q3 = 0.0f;

        /** @brief Integrated error tracker accumulators on the X-axis for gyro bias correction. */
        float eIntX = 0.0f;
        /** @brief Integrated error tracker accumulators on the Y-axis for gyro bias correction. */
        float eIntY = 0.0f;
        /** @brief Integrated error tracker accumulators on the Z-axis for gyro bias correction. */
        float eIntZ = 0.0f;
    };
}