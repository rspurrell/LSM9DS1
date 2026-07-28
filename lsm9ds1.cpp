#include "lsm9ds1.hpp"

#include <algorithm> // Provides access to std::clamp.
#include <limits> // Provides access to numeric_limits lowest() and max().
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

    if (!WriteRegister(kMagAddress, static_cast<uint8_t>(Magnetometer::ControlRegister::CtrlReg2M), configR2M.value))
    {
        return false;
    }
    pCtrlReg2MConfig_ = std::make_unique<Magnetometer::CtrlReg2M::Configuration>(configR2M);

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

    if (!WriteRegister(kMagAddress, static_cast<uint8_t>(Magnetometer::ControlRegister::CtrlReg1M), configR1M.value))
    {
        return false;
    }
    pCtrlReg1MConfig_ = std::make_unique<Magnetometer::CtrlReg1M::Configuration>(configR1M);

    if (!WriteRegister(kMagAddress, static_cast<uint8_t>(Magnetometer::ControlRegister::CtrlReg3M), configR3M.value))
    {
        return false;
    }
    pCtrlReg3MConfig_ = std::make_unique<Magnetometer::CtrlReg3M::Configuration>(configR3M);

    return true;
}

bool LSM9DS1::ReadAcceleration(Math::Vector3<int16_t>& rawValues) const
{
    if (!pCtrlReg6XLConfig_)
    {
        return false;
    }

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
    if (!pCtrlReg1GConfig_)
    {
        return false;
    }

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
    Math::Vector3<int16_t> rawValues;
    if (!ReadGyroscope(rawValues))
    {
        return false;
    }

    dps = Convert::ToDegreesPerSecond(rawValues, Gyroscope::CtrlReg1G::GetScale(*pCtrlReg1GConfig_));

    return true;
}

bool LSM9DS1::ReadGyroscopeRps(Math::Vector3<float>& rps) const
{
    Math::Vector3<int16_t> rawValues;
    if (!ReadGyroscope(rawValues))
    {
        return false;
    }

    rps = Convert::ToRadiansPerSecond(rawValues, Gyroscope::CtrlReg1G::GetScale(*pCtrlReg1GConfig_));

    return true;
}

Math::Vector3<int16_t> LSM9DS1::ApplyMagnetometerCalibration(const Math::Vector3<int16_t>& rawMag) const
{
    // If no calibration is enabled, return the raw values unchanged.
    if (!optMagCalibration_ || !optMagCalibration_->enabled)
    {
        return rawMag;
    }

    // Apply the hard-iron offset (centering the circle).
    // Using int32_t internally to prevent any chance of underflow/overflow during math.
    int32_t centeredX = static_cast<int32_t>(rawMag.x) - optMagCalibration_->offsetX;
    int32_t centeredY = static_cast<int32_t>(rawMag.y) - optMagCalibration_->offsetY;
    int32_t centeredZ = static_cast<int32_t>(rawMag.z) - optMagCalibration_->offsetZ;

    // Apply the soft-iron multiplier scale and bit-shift back down by 10 bits (/1024).
    // Note: Calibration is bit-shifted up by 10 bits to keep us out of fraction territory.
    // We want integers. We need to shift down.
    int32_t calibratedX = (centeredX * optMagCalibration_->scaleX) >> 10;
    int32_t calibratedY = (centeredY * optMagCalibration_->scaleY) >> 10;
    int32_t calibratedZ = (centeredZ * optMagCalibration_->scaleZ) >> 10;

    // Clamp values to fit back cleanly into int16_t limits.
    constexpr int32_t minLimit = static_cast<int32_t>(rawMin_);
    constexpr int32_t maxLimit = static_cast<int32_t>(rawMax_);
    return {
        static_cast<int16_t>(std::clamp(calibratedX, minLimit, maxLimit)),
        static_cast<int16_t>(std::clamp(calibratedY, minLimit, maxLimit)),
        static_cast<int16_t>(std::clamp(calibratedZ, minLimit, maxLimit))
    };
}

bool LSM9DS1::UpdateCalibrationLimits()
{
    Math::Vector3<int16_t> rawValues;
    if (!ReadMagnetometer(rawValues))
    {
        return false;
    }

    rawMagMin_.x = std::min(rawMagMin_.x, rawValues.x);
    rawMagMax_.x = std::max(rawMagMax_.x, rawValues.x);

    rawMagMin_.y = std::min(rawMagMin_.y, rawValues.y);
    rawMagMax_.y = std::max(rawMagMax_.y, rawValues.y);

    rawMagMin_.z = std::min(rawMagMin_.z, rawValues.z);
    rawMagMax_.z = std::max(rawMagMax_.z, rawValues.z);

    return true;
}

void LSM9DS1::LoadCalibration(MagnetometerCalibration&& calibration)
{
    optMagCalibration_ = std::move(calibration);
}

bool LSM9DS1::ReadMagnetometer(Math::Vector3<int16_t>& rawValues) const
{
    if (!pCtrlReg3MConfig_ || !pCtrlReg2MConfig_)
    {
        return false;
    }

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

bool LSM9DS1::ReadMagnetometerCalibrated(Math::Vector3<int16_t>& calValues) const
{
    Math::Vector3<int16_t> rawValues;
    if (!ReadMagnetometer(rawValues))
    {
        return false;
    }

    calValues = ApplyMagnetometerCalibration(rawValues);

    return true;
}

bool LSM9DS1::ReadMagnetometerMilliGauss(Math::Vector3<float>& magMilliGauss) const
{
    Math::Vector3<int16_t> rawValues;
    if (!ReadMagnetometer(rawValues))
    {
        return false;
    }

    magMilliGauss = Convert::ToMilliGauss(rawValues, Magnetometer::CtrlReg2M::GetScale(*pCtrlReg2MConfig_));

    return true;
}

bool LSM9DS1::ReadMagnetometerMilliGaussCalibrated(Math::Vector3<float>& mgCalibrated) const
{
    Math::Vector3<int16_t> rawValues;
    if (!ReadMagnetometer(rawValues))
    {
        return false;
    }

    mgCalibrated = Convert::ToMilliGauss(ApplyMagnetometerCalibration(rawValues), Magnetometer::CtrlReg2M::GetScale(*pCtrlReg2MConfig_));

    return true;
}

const Math::Vector3<int16_t> LSM9DS1::GetMinMagLimits() const
{
    return rawMagMin_;
}

const Math::Vector3<int16_t> LSM9DS1::GetMaxMagLimits() const
{
    return rawMagMax_;
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
