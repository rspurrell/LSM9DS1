# LSM9DS1 IMU Library

A standalone modern C++ library for interfacing with the **STMicroelectronics LSM9DS1 9-DOF IMU** over I²C on Linux-based single-board computers and embedded computing platforms.

The LSM9DS1 combines:

* 3-axis accelerometer
* 3-axis gyroscope
* 3-axis magnetometer

The accelerometer/gyroscope and magnetometer operate as two independent I²C devices, while the library provides a single interface for communicating with both.

## Features

* Accelerometer readings in:
  * Raw 16-bit values
  * G-force
  * m/s²

* Gyroscope readings in:
  * Raw 16-bit values
  * Degrees per second
  * Radians per second

* Magnetometer readings in:
  * Raw 16-bit values
  * Calibrated raw values
  * Milligauss
  * Calibrated milligauss

* Configurable accelerometer and gyroscope
* Configurable magnetometer
* Magnetometer hard-iron and soft-iron calibration
* Mahony AHRS orientation filter
* Device identification and verification
* Linux I²C device interface

## Requirements

* Linux-based single-board computer or embedded system
* C++ compiler supporting the C++ standard used by the project
* LSM9DS1 connected through an I²C interface
* Linux I²C device enabled and accessible by the application

The I²C device path is supplied when constructing the `LSM9DS1` object and defaults to `/dev/i2c-1`.

## Basic Usage

Create an `LSM9DS1` instance using the appropriate Linux I²C device, open the device, and initialize the sensors with the desired configurations.

```cpp
LSM9DS1 imu("/dev/i2c-1");

if (!imu.Open())
{
    return 1;
}

Accelerometer::CtrlReg6XL::Configuration accelConfig(
    Accelerometer::CtrlReg6XL::OutputDataRate::Hz_119,
    Accelerometer::CtrlReg6XL::Scale::G_2,
    Accelerometer::CtrlReg6XL::Bandwidth::Automatic,
    Accelerometer::CtrlReg6XL::AntiAliasFilter::Hz_408);

Gyroscope::CtrlReg1G::Configuration gyroConfig(
    Gyroscope::CtrlReg1G::OutputDataRate::Hz_119,
    Gyroscope::CtrlReg1G::Scale::Dps_245,
    Gyroscope::CtrlReg1G::Bandwidth::BW_00);

if (!imu.InitializeAccelGyro(accelConfig, gyroConfig))
{
    return 1;
}

Magnetometer::CtrlReg1M::Configuration configR1M(
    Magnetometer::CtrlReg1M::TemperatureCompensation::Enabled,
    Magnetometer::CtrlReg1M::OperatingModeXYAxis::HighPerformance,
    Magnetometer::CtrlReg1M::OutputDataRate::Hz_10,
    Magnetometer::CtrlReg1M::FastOutputDataRate::Disabled,
    Magnetometer::CtrlReg1M::SelfTest::Disabled);
Magnetometer::CtrlReg2M::Configuration configR2M(
    Magnetometer::CtrlReg2M::Scale::Gauss_4,
    Magnetometer::CtrlReg2M::Reboot::Normal,
    Magnetometer::CtrlReg2M::SoftReset::Normal);
Magnetometer::CtrlReg3M::Configuration configR3M(
    Magnetometer::CtrlReg3M::I2CInterface::Enabled,
    Magnetometer::CtrlReg3M::LowPowerMode::Disabled,
    Magnetometer::CtrlReg3M::SpiInterfaceMode::WriteOnly,
    Magnetometer::CtrlReg3M::OperatingMode::ContinuousConversion);
Magnetometer::CtrlReg4M::Configuration configR4M(
    Magnetometer::CtrlReg4M::OperatingModeZAxis::HighPerformance,
    Magnetometer::CtrlReg4M::Endianness::LittleEndian);
Magnetometer::CtrlReg5M::Configuration configR5M(
    Magnetometer::CtrlReg5M::FastRead::Disabled,
    Magnetometer::CtrlReg5M::BlockDataUpdate::Blocked);

if (!imu.InitializeMagnetometer(configR1M, configR2M, configR3M, configR4M, configR5M))
{
    return 1;
}
```

The magnetometer is initialized separately using its five register configuration structures.

## Reading Sensor Data

Sensor values are returned through `Math::Vector3` objects. The library provides unit-specific methods so applications can read converted values directly.

```cpp
Math::Vector3<float> acceleration;
Math::Vector3<float> gyroscope;
Math::Vector3<float> magnetometer;

if (imu.ReadAccelerationMetersPerSecondSquared(acceleration))
{
    std::cout << "Acceleration: "
              << acceleration.x << ", "
              << acceleration.y << ", "
              << acceleration.z << " m/s²\n";
}

if (imu.ReadGyroscopeRps(gyroscope))
{
    std::cout << "Gyroscope: "
              << gyroscope.x << ", "
              << gyroscope.y << ", "
              << gyroscope.z << " rad/s\n";
}

if (imu.ReadMagnetometerMilliGaussCalibrated(magnetometer))
{
    std::cout << "Magnetometer: "
              << magnetometer.x << ", "
              << magnetometer.y << ", "
              << magnetometer.z << " mG\n";
}
```

The library also provides methods for obtaining raw sensor values and alternative units such as G and degrees per second.

## Magnetometer Calibration

Magnetometer calibration can be performed by collecting minimum and maximum readings while rotating the IMU through a full 3D range of motion. The resulting calibration can then be loaded into the driver and applied to subsequent magnetometer readings.

```cpp
bool g_keepRunning = true;
while (g_keepRunning)
{
    // Move the sensor through a true 3D rotation.
    if (!imu.UpdateCalibrationLimits())
    {
        return 1;
    }
}

// After collecting calibration data:
LSM9DS1::MagnetometerCalibration calibration(
    imu.GetMinMagLimits(),
    imu.GetMaxMagLimits());
// Save calibration struct data for future execution.

// Apply magnetometer calibration.
imu.LoadCalibration(std::move(calibration));
```

Calibrated readings can then be obtained using:

```cpp
Math::Vector3<float> magnetometer;

imu.ReadMagnetometerMilliGaussCalibrated(magnetometer);
```

## Mahony AHRS

The library includes a [Mahony AHRS orientation filter](https://ahrs.readthedocs.io/en/latest/filters/mahony.html) implementation for sensor fusion using the LSM9DS1's accelerometer, gyroscope, and magnetometer data.

The filter maintains orientation internally as a quaternion and provides the resulting orientation as Euler angles. Gyroscope measurements must be provided in radians per second, while accelerometer and magnetometer values can be supplied in the units supported by the filter.

### Basic Usage

Create the filter with proportional and integral feedback gains:

```cpp
Math::MahonyAHRS mahony(1.0f, 0.0f);
```

For each sensor update, calculate the elapsed time since the previous update and pass the sensor measurements to `Update()`:

```cpp
auto lastTime = std::chrono::steady_clock::now();

while (true)
{
    // Read IMU data...

    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = currentTime - lastTime;
    lastTime = currentTime;

    mahony.Update(
        rps.x, rps.y, rps.z,
        acc.x, acc.y, acc.z,
        mg.x, mg.y, -mg.z,
        elapsed.count());

    auto euler = mahony.GetEulerAngles();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
```

`GetEulerAngles()` returns the estimated orientation as roll, pitch, and yaw angles. Yaw is referenced to magnetic north.

The proportional gain controls how quickly the filter corrects orientation errors, while the integral gain can be used to compensate for accumulated gyroscope bias. The default gains are `1.0f` and `0.0f`, respectively.

## Device Verification

The library can verify both devices that make up the LSM9DS1:

```cpp
if (!imu.VerifyAccelGyro())
{
    // Accelerometer/gyroscope not detected.
}

if (!imu.VerifyMagnetometer())
{
    // Magnetometer not detected.
}
```

Individual `WHO_AM_I` values are also available through `WhoAmIAccelGyro()` and `WhoAmIMagnetometer()`.

## License

See the repository license for terms of use.
