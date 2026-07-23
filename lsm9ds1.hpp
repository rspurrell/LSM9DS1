/******************************************************************************
 * @file    LSM9DS1.hpp
 * @brief   Driver for the STMicroelectronics LSM9DS1 9-DOF IMU.
 *
 * This class provides a C++ interface for communicating with the
 * accelerometer, gyroscope and magnetometer contained within the LSM9DS1
 * over the Linux I²C subsystem.
 ******************************************************************************/

#pragma once

#include <cstdint> // Provides fixed-width integer types (uint8_t, uint16_t, etc.).
#include <memory> // Provides dynamic memory management
#include <string> // Provides the std::string class.

#include "lsm9ds1_registers.hpp" // Provides Registers and bit definitions for the LSM9DS1.
#include "lsm9ds1_utils.hpp" // Provides the Math::Vector3 structure for 3D vectors.
#include "vector3.hpp" // Provides the Math::Vector3 structure for 3D vectors.

/**
 * @class LSM9DS1
 *
 * @brief Driver for the LSM9DS1 9-DOF IMU.
 *
 * The LSM9DS1 actually appears as two independent I²C slave devices:
 *
 *      Accelerometer + Gyroscope
 *      Magnetometer
 *
 * This class manages communication with both devices while presenting
 * a single interface to the application.
 */
class LSM9DS1
{
public:

    /**
     * @brief Constructs an LSM9DS1 object.
     *
     * @param device Linux I²C device.
     */
    explicit LSM9DS1(const std::string& device = "/dev/i2c-1");

    /**
     * @brief Destructor.
     */
    ~LSM9DS1();

    /**
     * @brief Opens the Linux I²C device.
     *
     * @return True if successful.
     */
    bool Open();

    /**
     * @brief Closes the Linux I²C device.
     */
    void Close();

    /**
     * @brief Returns whether the Linux device is open.
     *
     * @return True if open.
     */
    bool IsOpen() const;

    /**
     * @brief Initializes the accelerometer and gyroscope.
     *
     * @param accelConfig Configuration for the accelerometer.
     * @param gyroConfig Configuration for the gyroscope.
     *
     * @return True if initialization succeeded.
     */
    bool InitializeAccelGyro(Accelerometer::CtrlReg6XL::Configuration accelConfig, Gyroscope::CtrlReg1G::Configuration gyroConfig);

    /**
     * @brief Initializes the accelerometer.
     *
     * @param accelConfig Configuration for the accelerometer.
     *
     * @return True if initialization succeeded.
     */
    bool InitializeAccel(Accelerometer::CtrlReg6XL::Configuration accelConfig);

    /**
     * @brief Initializes the magnetometer.
     *
     * @param configR1M Configuration for the magnetometer register 1.
     * @param configR2M Configuration for the magnetometer register 2.
     * @param configR3M Configuration for the magnetometer register 3.
     * @param configR4M Configuration for the magnetometer register 4.
     * @param configR5M Configuration for the magnetometer register 5.
     *
     * @return True if initialization succeeded.
     */
    bool InitializeMagnetometer(
        Magnetometer::CtrlReg1M::Configuration configR1M,
        Magnetometer::CtrlReg2M::Configuration configR2M,
        Magnetometer::CtrlReg3M::Configuration configR3M,
        Magnetometer::CtrlReg4M::Configuration configR4M,
        Magnetometer::CtrlReg5M::Configuration configR5M
    );

    /**
     * @brief Reads the current acceleration.
     *
     * @param[out] rawValues Raw acceleration values.
     *
     * @return True if successful.
     */
    bool ReadAcceleration(Math::Vector3<int16_t>& rawValues) const;

    /**
     * @brief Reads the current acceleration in meters per second squared (m/s²).
     *
     * @param[out] accMpsps Acceleration values in m/s².
     *
     * @return True if successful.
     */
    bool ReadAccelerationMetersPerSecondSquared(Math::Vector3<float>& accMpsps) const;

    /**
     * @brief Reads the current acceleration in units of gravitational acceleration (G).
     *
     * @param[out] accGs Acceleration values in Gs.
     *
     * @return True if successful.
     */
    bool ReadAccelerationGs(Math::Vector3<float>& accGs) const;

    /**
     * @brief Reads the current angular velocity.
     *
     * @param[out] rawValues Raw gyroscope values.
     *
     * @return True if successful.
     */
    bool ReadGyroscope(Math::Vector3<int16_t>& rawValues) const;

    /**
     * @brief Reads the current angular velocity in degrees per seond.
     *
     * @param[out] dps Gyroscope values in degrees per second.
     *
     * @return True if successful.
     */
    bool ReadGyroscopeDps(Math::Vector3<float>& dps) const;

    /**
     * @brief Reads the current magnetometer sensor data.
     *
     * @param[out] rawValues Raw magnetometer values.
     *
     * @return True if successful.
     */
    bool ReadMagnetometer(Math::Vector3<int16_t>& rawValues) const;

    /**
     * @brief Reads the current magnetometer sensor data in milligauss.
     *
     * @param[out] magMilliGauss Magnetometer values in milligauss.
     *
     * @return True if successful.
     */
    bool ReadMagnetometerMilliGauss(Math::Vector3<float>& magMilliGauss) const;

    /**
     * @brief Reads the WHO_AM_I register from the accelerometer/gyroscope.
     *
     * @param[out] id Device identification value.
     *
     * @return True if successful.
     */
    bool WhoAmIAccelGyro(uint8_t& id) const;

    /**
     * @brief Reads the WHO_AM_I register from the magnetometer.
     *
     * @param[out] id Device identification value.
     *
     * @return True if successful.
     */
    bool WhoAmIMagnetometer(uint8_t& id) const;

    /**
     * @brief Verifies the accelerometer/gyroscope.
     *
     * @return True if the correct device is detected.
     */
    bool VerifyAccelGyro() const;

    /**
     * @brief Verifies the magnetometer.
     *
     * @return True if the correct device is detected.
     */
    bool VerifyMagnetometer() const;

private:

    /**
     * @brief Reads an 8-bit register.
     *
     * @param deviceAddress I²C slave address.
     * @param registerAddress Register to read.
     * @param[out] value Returned register value.
     *
     * @return True if successful.
     */
    bool ReadRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t& value) const;

    /**
     * @brief Reads an 16-bit register.
     *
     * @param deviceAddress I²C slave address.
     * @param registerAddress Register to read.
     * @param[out] value Returned register value.
     *
     * @return True if successful.
     */
    bool ReadRegister16(uint8_t deviceAddress, uint8_t registerAddress, int16_t& value) const;

    /**
     * @brief Writes an 8-bit register.
     *
     * @param deviceAddress I²C slave address.
     * @param registerAddress Register to write.
     * @param value Value to write.
     *
     * @return True if successful.
     */
    bool WriteRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value);


    /// Default I²C address of the accelerometer/gyroscope.
    static constexpr uint8_t kAccelGyroAddress = 0x6B;
    /// Default I²C address of the magnetometer.
    static constexpr uint8_t kMagAddress = 0x1E;

    /// kWhoAmIRegister common register.
    static constexpr uint8_t kWhoAmIRegister = 0x0F;

    /// Expected kWhoAmIRegister value for the accelerometer/gyroscope.
    static constexpr uint8_t kAccelGyroId = 0x68;
    /// Expected kWhoAmIRegister value for the magnetometer.
    static constexpr uint8_t kMagId = 0x3D;

    /// Linux I²C device path.
    std::string devicePath_;

    /// Linux file descriptor.
    int fd_;

    /// Stores the current active accelerometer configuration for CtrlReg6XL
    std::unique_ptr<Accelerometer::CtrlReg6XL::Configuration> pCtrlReg6XLConfig_;
    /// Stores the current active gyroscope configuration for CtrlReg1G
    std::unique_ptr<Gyroscope::CtrlReg1G::Configuration> pCtrlReg1GConfig_;

    /// Stores the current active Magnetometer configuration for CtrlReg1M
    std::unique_ptr<Magnetometer::CtrlReg1M::Configuration> pCtrlReg1MConfig_;
    /// Stores the current active Magnetometer configuration for CtrlReg2M
    std::unique_ptr<Magnetometer::CtrlReg2M::Configuration> pCtrlReg2MConfig_;
    /// Stores the current active Magnetometer configuration for CtrlReg3M
    std::unique_ptr<Magnetometer::CtrlReg3M::Configuration> pCtrlReg3MConfig_;
    /// Stores the current active Magnetometer configuration for CtrlReg4M
    std::unique_ptr<Magnetometer::CtrlReg4M::Configuration> pCtrlReg4MConfig_;
    /// Stores the current active Magnetometer configuration for CtrlReg5M
    std::unique_ptr<Magnetometer::CtrlReg5M::Configuration> pCtrlReg5MConfig_;
};