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

    // Store gyroscope configuration after successfully writing it to the hardware.
    pCtrlReg1GConfig_ = std::make_unique<Gyroscope::CtrlReg1G::Configuration>(gyroConfig);

    return true;
}

bool LSM9DS1::InitializeAccel(Accelerometer::CtrlReg6XL::Configuration accelConfig)
{
    // Enable the accelerometer with the specified configuration.
    if (!WriteRegister(kAccelGyroAddress, static_cast<uint8_t>(Accelerometer::ControlRegister::CtrlReg6XL), accelConfig.value))
    {
        return false;
    }

    // Store accelerometer configuration after successfully writing it to the hardware.
    pCtrlReg6XLConfig_ = std::make_unique<Accelerometer::CtrlReg6XL::Configuration>(accelConfig);
    // Writing to CTRL_REG6_XL places the gyroscope into power-down mode. Clear any gyroscope configuration.
    pCtrlReg1GConfig_.reset();

    return true;
}

bool LSM9DS1::InitializeMagnetometer(
    Magnetometer::CtrlReg1M::Configuration configR1M,
    Magnetometer::CtrlReg2M::Configuration configR2M,
    Magnetometer::CtrlReg3M::Configuration configR3M,
    Magnetometer::CtrlReg4M::Configuration configR4M,
    Magnetometer::CtrlReg5M::Configuration configR5M
)
{
    if (!WriteRegister(kMagAddress, static_cast<uint8_t>(Magnetometer::ControlRegister::CtrlReg1M), configR1M.value))
    {
        return false;
    }
    pCtrlReg1MConfig_ = std::make_unique<Magnetometer::CtrlReg1M::Configuration>(configR1M);

    if (!WriteRegister(kMagAddress, static_cast<uint8_t>(Magnetometer::ControlRegister::CtrlReg2M), configR2M.value))
    {
        return false;
    }
    pCtrlReg2MConfig_ = std::make_unique<Magnetometer::CtrlReg2M::Configuration>(configR2M);

    if (!WriteRegister(kMagAddress, static_cast<uint8_t>(Magnetometer::ControlRegister::CtrlReg3M), configR3M.value))
    {
        return false;
    }
    pCtrlReg3MConfig_ = std::make_unique<Magnetometer::CtrlReg3M::Configuration>(configR3M);

    if (!WriteRegister(kMagAddress, static_cast<uint8_t>(Magnetometer::ControlRegister::CtrlReg4M), configR4M.value))
    {
        return false;
    }
    pCtrlReg4MConfig_ = std::make_unique<Magnetometer::CtrlReg4M::Configuration>(configR4M);

    if (!WriteRegister(kMagAddress, static_cast<uint8_t>(Magnetometer::ControlRegister::CtrlReg5M), configR5M.value))
    {
        return false;
    }
    pCtrlReg5MConfig_ = std::make_unique<Magnetometer::CtrlReg5M::Configuration>(configR5M);
    return true;
}

bool LSM9DS1::ReadAcceleration(Math::Vector3<int16_t>& rawValues) const
{
    if (!ReadRegister16(kAccelGyroAddress, static_cast<uint8_t>(Accelerometer::OutputRegister::Out_X_L_XL), rawValues.x))
    {
        return false;
    }

    if (!ReadRegister16(kAccelGyroAddress, static_cast<uint8_t>(Accelerometer::OutputRegister::Out_Y_L_XL), rawValues.y))
    {
        return false;
    }

    if (!ReadRegister16(kAccelGyroAddress, static_cast<uint8_t>(Accelerometer::OutputRegister::Out_Z_L_XL), rawValues.z))
    {
        return false;
    }

    return true;
}

bool LSM9DS1::ReadAccelerationMetersPerSecondSquared(Math::Vector3<float>& accMpsps) const
{
    if (!pCtrlReg6XLConfig_)
    {
        return false;
    }

    Math::Vector3<int16_t> rawValues;
    if (!ReadAcceleration(rawValues))
    {
        return false;
    }

    accMpsps = Convert::ToMetersPerSecondSquared(rawValues, Accelerometer::CtrlReg6XL::GetScale(*pCtrlReg6XLConfig_));

    return true;
}

bool LSM9DS1::ReadAccelerationGs(Math::Vector3<float>& accGs) const
{
    if (!pCtrlReg6XLConfig_)
    {
        return false;
    }

    Math::Vector3<int16_t> rawValues;
    if (!ReadAcceleration(rawValues))
    {
        return false;
    }

    accGs = Convert::ToGs(rawValues, Accelerometer::CtrlReg6XL::GetScale(*pCtrlReg6XLConfig_));

    return true;
}

bool LSM9DS1::ReadGyroscope(Math::Vector3<int16_t>& rawValues) const
{
    if (!ReadRegister16(kAccelGyroAddress, static_cast<uint8_t>(Gyroscope::OutputRegister::Out_X_L_G), rawValues.x))
    {
        return false;
    }

    if (!ReadRegister16(kAccelGyroAddress, static_cast<uint8_t>(Gyroscope::OutputRegister::Out_Y_L_G), rawValues.y))
    {
        return false;
    }

    if (!ReadRegister16(kAccelGyroAddress, static_cast<uint8_t>(Gyroscope::OutputRegister::Out_Z_L_G), rawValues.z))
    {
        return false;
    }

    return true;
}

bool LSM9DS1::ReadGyroscopeDps(Math::Vector3<float>& dps) const
{
    if (!pCtrlReg1GConfig_)
    {
        return false;
    }

    Math::Vector3<int16_t> rawValues;
    if (!ReadGyroscope(rawValues))
    {
        return false;
    }

    dps = Convert::ToDegreesPerSecond(rawValues, Gyroscope::CtrlReg1G::GetScale(*pCtrlReg1GConfig_));

    return true;
}

bool LSM9DS1::ReadMagnetometer(Math::Vector3<int16_t>& rawValues) const
{
    if (!ReadRegister16(kMagAddress, static_cast<uint8_t>(Magnetometer::OutputRegister::Out_X_L_M), rawValues.x))
    {
        return false;
    }

    if (!ReadRegister16(kMagAddress, static_cast<uint8_t>(Magnetometer::OutputRegister::Out_Y_L_M), rawValues.y))
    {
        return false;
    }

    if (!ReadRegister16(kMagAddress, static_cast<uint8_t>(Magnetometer::OutputRegister::Out_Z_L_M), rawValues.z))
    {
        return false;
    }
    return true;
}

bool LSM9DS1::ReadMagnetometerMilliGauss(Math::Vector3<float>& magMilliGauss) const
{
    if (!pCtrlReg3MConfig_ || !pCtrlReg2MConfig_)
    {
        return false;
    }

    Math::Vector3<int16_t> rawValues;
    if (!ReadMagnetometer(rawValues))
    {
        return false;
    }

    magMilliGauss = Convert::ToMilliGauss(rawValues, Magnetometer::CtrlReg2M::GetScale(*pCtrlReg2MConfig_));

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
