/******************************************************************************
 * @file        LSM9DS1.hpp
 * @brief       Driver for the STMicroelectronics LSM9DS1 9-DOF IMU.
 *
 * This class provides a C++ interface for communicating with the
 * accelerometer, gyroscope and magnetometer contained within the LSM9DS1
 * over the Linux I²C subsystem.
 ******************************************************************************/

#pragma once

#include <cstdint> // Provides fixed-width integer types (uint8_t, uint16_t, etc.).
#include <string> // Provides the std::string class.

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

private:

    /// Linux I²C device path.
    std::string devicePath_;

    /// Linux file descriptor.
    int fd_;

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
    bool ReadRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t& value);

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
};