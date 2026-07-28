// Provides the sensor conversion function declarations.
#include "lsm9ds1_utils.hpp"

// Provides the standard exception type used for invalid scale configurations.
#include <stdexcept>

namespace Convert
{
    Math::Vector3<float> ToMetersPerSecondSquared(const Math::Vector3<int16_t>& rawValues, Accelerometer::CtrlReg6XL::Scale scale)
    {
        static constexpr float kAccelG = 9.80665F;
        float sensitivity;

        switch (scale)
        {
            case Accelerometer::CtrlReg6XL::Scale::G_2:
                sensitivity = 0.000061F * kAccelG;
                break;

            case Accelerometer::CtrlReg6XL::Scale::G_4:
                sensitivity = 0.000122F * kAccelG;
                break;

            case Accelerometer::CtrlReg6XL::Scale::G_8:
                sensitivity = 0.000244F * kAccelG;
                break;

            case Accelerometer::CtrlReg6XL::Scale::G_16:
                sensitivity = 0.000732F * kAccelG;
                break;

            default:
                throw std::invalid_argument("Invalid accelerometer scale.");
        }

        return Math::Vector3<float>
        {
            static_cast<float>(rawValues.x) * sensitivity,
            static_cast<float>(rawValues.y) * sensitivity,
            static_cast<float>(rawValues.z) * sensitivity
        };
    }

    Math::Vector3<float> ToGs(const Math::Vector3<int16_t>& rawValues, Accelerometer::CtrlReg6XL::Scale scale)
    {
        float sensitivity;

        switch (scale)
        {
            case Accelerometer::CtrlReg6XL::Scale::G_2:
                sensitivity = 0.000061F;
                break;

            case Accelerometer::CtrlReg6XL::Scale::G_4:
                sensitivity = 0.000122F;
                break;

            case Accelerometer::CtrlReg6XL::Scale::G_8:
                sensitivity = 0.000244F;
                break;

            case Accelerometer::CtrlReg6XL::Scale::G_16:
                sensitivity = 0.000732F;
                break;

            default:
                throw std::invalid_argument("Invalid accelerometer scale.");
        }

        return Math::Vector3<float>
        {
            static_cast<float>(rawValues.x) * sensitivity,
            static_cast<float>(rawValues.y) * sensitivity,
            static_cast<float>(rawValues.z) * sensitivity
        };
    }

    Math::Vector3<float> ToDegreesPerSecond(const Math::Vector3<int16_t>& rawValues, Gyroscope::CtrlReg1G::Scale scale)
    {
        float sensitivity;

        switch (scale)
        {
            case Gyroscope::CtrlReg1G::Scale::Dps_245:
                sensitivity = 0.00875F;
                break;

            case Gyroscope::CtrlReg1G::Scale::Dps_500:
                sensitivity = 0.01750F;
                break;

            case Gyroscope::CtrlReg1G::Scale::Dps_2000:
                sensitivity = 0.07000F;
                break;

            default:
                throw std::invalid_argument("Invalid gyroscope scale.");
        }

        return Math::Vector3<float>
        {
            static_cast<float>(rawValues.x) * sensitivity,
            static_cast<float>(rawValues.y) * sensitivity,
            static_cast<float>(rawValues.z) * sensitivity
        };
    }

    Math::Vector3<float> ToRadiansPerSecond(const Math::Vector3<int16_t>& rawValues, Gyroscope::CtrlReg1G::Scale scale)
    {
        constexpr float kDegreesToRadians = 0.01745329251994329577f; // PI / 180
        Math::Vector3<float> dps = ToDegreesPerSecond(rawValues, scale);
        dps.x = dps.x * kDegreesToRadians;
        dps.y = dps.y * kDegreesToRadians;
        dps.z = dps.z * kDegreesToRadians;
        return dps;
    }

    Math::Vector3<float> ToMilliGauss(const Math::Vector3<int16_t>& rawValues, Magnetometer::CtrlReg2M::Scale scale)
    {
        float sensitivity;

        switch (scale)
        {
            case Magnetometer::CtrlReg2M::Scale::Gauss_4:
                sensitivity = 0.14F;
                break;

            case Magnetometer::CtrlReg2M::Scale::Gauss_8:
                sensitivity = 0.29F;
                break;

            case Magnetometer::CtrlReg2M::Scale::Gauss_12:
                sensitivity = 0.43F;
                break;

            case Magnetometer::CtrlReg2M::Scale::Gauss_16:
                sensitivity = 0.58F;
                break;

            default:
                throw std::invalid_argument("Invalid magnetometer scale.");
        }

        return Math::Vector3<float>
        {
            static_cast<float>(rawValues.x) * sensitivity,
            static_cast<float>(rawValues.y) * sensitivity,
            static_cast<float>(rawValues.z) * sensitivity
        };
    }
}