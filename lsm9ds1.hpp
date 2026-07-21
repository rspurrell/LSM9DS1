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
#include <string> // Provides the std::string class.

#include "lsm9ds1_registers.hpp" // Provides Registers and bit definitions for the LSM9DS1.

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
};