#include "mahony_ahrs.hpp"

namespace Math
{
    void MahonyAHRS::Update(
        float gx, float gy, float gz,
        float ax, float ay, float az,
        float mx, float my, float mz, float dt
    )
    {
        float recipNorm;
        float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
        float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
        float q2q2 = q2 * q2, q2q3 = q2 * q3;
        float q3q3 = q3 * q3;

        float hx, hy, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;

        // Normalizing Accelerometer Data
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
        }

        // Normalizing Magnetometer Data
        if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
            recipNorm = 1.0f / std::sqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;
        }

        // Reference direction of Earth's magnetic field
        hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
        hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
        bx = std::sqrt((hx * hx) + (hy * hy));
        bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

        // Estimated direction of gravity and magnetic field vectors
        vx = 2.0f * (q1q3 - q0q2);
        vy = 2.0f * (q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;
        wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
        wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
        wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

        // Vector cross product sum of error indicators
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

        // Integral feedback calculation
        if(ki > 0.0f) {
            eIntX += ex * dt;
            eIntY += ey * dt;
            eIntZ += ez * dt;
            gx += ki * eIntX;
            gy += ki * eIntY;
            gz += ki * eIntZ;
        }

        // Proportional feedback calculation
        gx += kp * ex;
        gy += kp * ey;
        gz += kp * ez;

        // Integrate rate of change of quaternion
        float pa = q1, pb = q2, pc = q3;
        q0 += (-pa * gx - pb * gy - pc * gz) * (0.5f * dt);
        q1 += (q0 * gx + pb * gz - pc * gy) * (0.5f * dt);
        q2 += (q0 * gy - pa * gz + pc * gx) * (0.5f * dt);
        q3 += (q0 * gz + pa * gy - pb * gx) * (0.5f * dt);

        // Normalize quaternion
        recipNorm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    Math::Vector3<float> MahonyAHRS::GetEulerAngles()
    {
        return Vector3<float>(
            // Roll
            std::atan2(2.0f * (q0 * q1 + q2 * q3),
                1.0f - 2.0f * (q1 * q1 + q2 * q2)
            ) * (180.0f / M_PI),

            // Pitch
            std::asin(2.0f * (q0 * q2 - q3 * q1)) * (180.0f / M_PI),

            // Yaw
            std::atan2(2.0f * (q0 * q3 + q1 * q2),
                1.0f - 2.0f * (q2 * q2 + q3 * q3)
            ) * (180.0f / M_PI)
        );
    }
}