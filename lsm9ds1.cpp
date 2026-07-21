#include "lsm9ds1.hpp"

#include <fcntl.h> // Declares file control options and the open() function.
#include <linux/i2c-dev.h> // Declares the Linux I²C userspace API.
#include <sys/ioctl.h> // Declares the ioctl() function for device control.
#include <unistd.h> // Declares POSIX functions such as read(), write(), and close().

LSM9DS1::LSM9DS1(const std::string& device) : devicePath_(device), fd_(-1)
{}

LSM9DS1::~LSM9DS1()
{
    Close();
}

bool LSM9DS1::Open()
{
    fd_ = open(devicePath_.c_str(), O_RDWR);
    return fd_ >= 0;
}

void LSM9DS1::Close()
{
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

bool LSM9DS1::IsOpen() const
{
    return fd_ >= 0;
}

bool LSM9DS1::ReadRegister(uint8_t addr, uint8_t reg, uint8_t& value) const
{
    // Select the target sensor device on the shared I2C bus.
    if (ioctl(fd_, I2C_SLAVE, addr) < 0)
    {
        // Failure to address the requested slave.
        return false;
    }

    // Send the register number to the chip.
    if (write(fd_, &reg, 1) != 1)
    {
        // Failure to write to the register.
        return false;
    }

    // Read back one byte from the register.
    if (read(fd_, &value, 1) != 1)
    {
        // Failure to read from the register.
        return false;
    }

    return true;
}

bool LSM9DS1::ReadRegister16(uint8_t deviceAddress, uint8_t registerAddress, int16_t& value) const
{
    uint8_t lowByte;
    uint8_t highByte;

    if (!ReadRegister(deviceAddress, registerAddress, lowByte))
    {
        return false;
    }

    if (!ReadRegister(deviceAddress, registerAddress + 1, highByte))
    {
        return false;
    }

    value = static_cast<int16_t>((static_cast<uint16_t>(highByte) << 8) | lowByte);

    return true;
}

bool LSM9DS1::WriteRegister(uint8_t addr, uint8_t reg, uint8_t value)
{
    if (ioctl(fd_, I2C_SLAVE, addr) < 0)
    {
        // Failure to address the requested slave.
        return false;
    }

    // Two-byte transfer: register address followed by value.
    uint8_t buffer[2] = { reg, value };
    if (write(fd_, buffer, sizeof(buffer)) != sizeof(buffer))
    {
        // Failure to write to the register.
        return false;
    }

    return true;
}

bool LSM9DS1::InitializeAccelGyro(Accelerometer::CtrlReg6XL::Configuration accelConfig, Gyroscope::CtrlReg1G::Configuration gyroConfig)
{
    // Accelerometer must be configured first. Writing to CtrlReg6XL disables Gyro.
    if (!InitializeAccel(accelConfig))
    {
        return false;
    }

    // Activates both the accelerometer and gyroscope at the same data rate.
    if (!WriteRegister(kAccelGyroAddress, static_cast<uint8_t>(Gyroscope::ControlRegister::CtrlReg1G), gyroConfig.value))
    {
        return false;
    }

    return true;
}

bool LSM9DS1::InitializeAccel(Accelerometer::CtrlReg6XL::Configuration accelConfig)
{
    // Enable the accelerometer with the specified configuration.
    if (!WriteRegister(kAccelGyroAddress, static_cast<uint8_t>(Accelerometer::ControlRegister::CtrlReg6XL), accelConfig.value))
    {
        return false;
    }

    return true;
}

bool LSM9DS1::WhoAmIAccelGyro(uint8_t& id) const
{
    return ReadRegister(kAccelGyroAddress, kWhoAmIRegister, id);
}

bool LSM9DS1::WhoAmIMagnetometer(uint8_t& id) const
{
    return ReadRegister(kMagAddress, kWhoAmIRegister, id);
}

bool LSM9DS1::VerifyAccelGyro() const
{
    uint8_t id;

    if (!WhoAmIAccelGyro(id))
    {
        return false;
    }

    return id == kAccelGyroId;
}

bool LSM9DS1::VerifyMagnetometer() const
{
    uint8_t id;

    if (!WhoAmIMagnetometer(id))
    {
        return false;
    }

    return id == kMagId;
}
