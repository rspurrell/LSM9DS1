/******************************************************************************
 * @file    lsm9ds1_registers.hpp
 * @brief   Registers and bit definitions for the STMicroelectronics LSM9DS1 9-DOF IMU.
 ******************************************************************************/

#pragma once

#include <cstdint> // Provides fixed-width integer types (uint8_t, uint16_t, etc.).

#pragma region Gyroscope control registers
namespace Gyroscope
{
    /**
     * @brief Hardware register addresses for the LSM9DS1 Gyroscope control registers.
     */
    enum class ControlRegister : uint8_t
    {
        /**
        * @brief Register 1: Gyroscope Control - CTRL_REG1_G: 0x10. Initialization register for the gyroscope.
        * @details This register configures the output data rate, full-scale range, and bandwidth of the gyroscope.
        * The register layout is as follows:
        * Bit  7        6        5       4       3      2      1       0
        * +--------+--------+--------+-------+-------+-----+-------+-------+
        * | ODR_G2 | ODR_G1 | ODR_G0 | FS_G1 | FS_G0 |  0  | BW_G1 | BW_G0 |
        * +--------+--------+--------+-------+-------+-----+-------+-------+
        * Bits 7-5: ODR_G   (Output data rate) [Default value: 000]
        * Bits 4-3: FS_G    (Full scale selection) [Default value: 00]
        * Bit 2:    Reserved [Default value: 0]
        * Bits 1-0: BW_G    (Bandwidth) [Default value: 00]
        */
        CtrlReg1G = 0x10, // Register 1: Gyro Control

        /**
         * @brief Register 2: Gyroscope Control - CTRL_REG2_G: 0x11
         * @details This register manages the filtering path selections for both the primary output data
         * registers/FIFO and the internal interrupt generation block.
         * The register layout is as follows:
         * Bit
         *   7   6   5   4       3          2          1          0
         * +---+---+---+---+----------+----------+----------+----------+
         * | 0 | 0 | 0 | 0 | INT_SEL1 | INT_SEL0 | OUT_SEL1 | OUT_SEL0 |
         * +---+---+---+---+----------+----------+----------+----------+
         * Bits 7-4: Reserved  [Default value: 0000]
         * Bits 3-2: INT_SEL   (Gyroscope interrupt signal selection multiplexer) [Default value: 00]
         * Bits 1-0: OUT_SEL   (Gyroscope data output selection multiplexer) [Default value: 00]
         */
        CtrlReg2G = 0x11, // Register 2: Gyro Control

        /**
         * @brief Register 3: Gyroscope Control - CTRL_REG3_G: 0x12
         * @details This register configures the power mode and high-pass filter settings for the gyroscope.
         * The register layout is as follows:
         * Bit  7         6    5   4      3        2         1         0
         * +---------+-------+---+---+--------+---------+---------+---------+
         * | LP_mode | HP_EN | 0 | 0 | HPCF_G3| HPCF_G2 | HPCF_G1 | HPCF_G0 |
         * +---------+-------+---+---+--------+---------+---------+---------+
         * Bit 7:    LP_mode   (Low-power mode enable) [Default value: 0]
         * Bit 6:    HP_EN     (High-pass filter enable) [Default value: 0]
         * Bits 5-4: Reserved  [Default value: 00]
         * Bits 3-0: HPCF_G    (Gyroscope high-pass filter cutoff frequency selection) [Default value: 0000]
         */
        CtrlReg3G = 0x12 // Register 3: Gyro Power and Filter Control
    };

    namespace CtrlReg1G
    {
        /**
         * @brief Gyroscope output data rate configuration.
         * @details These bits (ODR_G[2:0]) reside in CTRL_REG1_G (10h) [Bits 7:5]. They define the output data
         * rate of the gyroscope. The default value is 000, which corresponds to a power-down mode. The other
         * options are 14.9 Hz (001), 59.5 Hz (010), 119 Hz (011), 238 Hz (100), 476 Hz (101), and 952 Hz (110).
         * The output data rate affects the frequency at which new gyroscope data is available for reading.
         */
        enum class OutputDataRate : uint8_t
        {
            PowerDown = 0x00,
            Hz_14_9 = 0x20,  // 14.9 Hz output data rate.
            Hz_59_5 = 0x40,  // 59.5 Hz output data rate.
            Hz_119 = 0x60, // 119 Hz output data rate.
            Hz_238 = 0x80, // 238 Hz output data rate.
            Hz_476 = 0xA0, // 476 Hz output data rate.
            Hz_952 = 0xC0  // 952 Hz output data rate.
        };

        /**
        * @brief Gyroscope full-scale range configuration.
        * @details These bits (FS_G[1:0]) reside in CTRL_REG1_G (10h) [Bits 4:3]. They define the full-scale
        * range of the gyroscope output data. The default value is 00, which corresponds to a full-scale
        * range of ±245 degrees per second (dps). The other options are ±500 dps (01) and ±2000 dps (11).
        * The full-scale range affects the sensitivity of the gyroscope readings, with larger ranges allowing
        * for the measurement of faster rotations but with lower resolution.
        */
        enum class Scale : uint8_t
        {
            Dps_245 = 0x00,  // Default. ±245 degrees per second.
            Dps_500 = 0x08, // ±500 degrees per second.
            Dps_2000 = 0x18 // ±2000 degrees per second.
        };

        /**
         * @brief Configuration values for the Gyroscope Low-Pass Filter 2 (LPF2) Bandwidth Selection bits.
         * @details These bits (BW_G[1:0]) reside in CTRL_REG1_G (10h) [Bits 1:0]. They work dynamically
         * with the selected Output Data Rate (ODR_G) to define the secondary digital low-pass filter
         * cutoff frequency, provided LPF2 is enabled via OUT_SEL in CTRL_REG2_G.
         */
        enum class Bandwidth : uint8_t
        {
            /**
             * @brief Default. Configures BW_G bits to 00.
             * @details Cutoff frequencies after LPF2 based on selected ODR:
             * - ODR  14.9 Hz: Bypassed (Utilizes LPF1 ~5 Hz cutoff only)
             * - ODR  59.5 Hz: 16 Hz Cutoff
             * - ODR 119.0 Hz: 14 Hz Cutoff
             * - ODR 238.0 Hz: 14 Hz Cutoff
             * - ODR 476.0 Hz: 21 Hz Cutoff
             * - ODR 952.0 Hz: 33 Hz Cutoff
             */
            BW_00 = 0x00,
            /**
             * @brief Configures BW_G bits to 01.
             * @details Cutoff frequencies after LPF2 based on selected ODR:
             * - ODR  14.9 Hz: Bypassed (Utilizes LPF1 ~5 Hz cutoff only)
             * - ODR  59.5 Hz: 16 Hz Cutoff
             * - ODR 119.0 Hz: 31 Hz Cutoff
             * - ODR 238.0 Hz: 29 Hz Cutoff
             * - ODR 476.0 Hz: 28 Hz Cutoff
             * - ODR 952.0 Hz: 40 Hz Cutoff
             */
            BW_01 = 0x01,
            /**
             * @brief Configures BW_G bits to 10.
             * @details Cutoff frequencies after LPF2 based on selected ODR:
             * - ODR  14.9 Hz: Bypassed (Utilizes LPF1 ~5 Hz cutoff only)
             * - ODR  59.5 Hz: 16 Hz Cutoff
             * - ODR 119.0 Hz: 31 Hz Cutoff
             * - ODR 238.0 Hz: 63 Hz Cutoff
             * - ODR 476.0 Hz: 57 Hz Cutoff
             * - ODR 952.0 Hz: 58 Hz Cutoff
             */
            BW_10 = 0x02,
            /**
             * @brief Configures BW_G bits to 11.
             * @details Cutoff frequencies after LPF2 based on selected ODR:
             * - ODR  14.9 Hz: Bypassed (Utilizes LPF1 ~5 Hz cutoff only)
             * - ODR  59.5 Hz: 16 Hz Cutoff
             * - ODR 119.0 Hz: 31 Hz Cutoff
             * - ODR 238.0 Hz: 78 Hz Cutoff
             * - ODR 476.0 Hz: 100 Hz Cutoff
             * - ODR 952.0 Hz: 100 Hz Cutoff
             */
            BW_11 = 0x03
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg1G register
         */
        enum class BitMask : uint8_t
        {
            Bandwidth = 0x03,
            Scale = 0x18,
            OutputDataRate = 0xE0,
        };

        /**
         * @brief Configuration structure for the Gyroscope Output Data Rate, Full-Scale Range, and Bandwidth Selection.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(OutputDataRate odr, Scale scale, Bandwidth bandwidth)
                : value(static_cast<uint8_t>(odr) | static_cast<uint8_t>(scale) | static_cast<uint8_t>(bandwidth))
            {}
        };

        constexpr Scale GetScale(Configuration config)
        {
            return static_cast<Scale>(config.value & static_cast<uint8_t>(BitMask::Scale));
        }
    }

    namespace CtrlReg2G
    {
        /**
        * @brief Configuration values for the Gyroscope Interrupt Selection Multiplexer bits.
        * @details These bits (INT_SEL[1:0]) reside in CTRL_REG2_G (11h) [Bits 3:2]. They govern
        * the internal digital signal routing specifically for the hardware interrupt generation engine.
        */
        enum class InterruptFilterPath : uint8_t
        {
            /**
            * @brief Default. Routes raw data through LPF1 only to the interrupt engine.
            * @details Both the High-Pass Filter (HPF) and Low-Pass Filter 2 (LPF2) are bypassed
            * for interrupt thresholds. Employs minimal processing latency.
            */
            LPF1_ONLY = 0x00,

            /**
            * @brief Routes data through LPF1 and then the High-Pass Filter (HPF) to the interrupt engine.
            * @details Low-Pass Filter 2 (LPF2) is bypassed. Useful for generating interrupts based
            * on dynamic, sudden movements while filtering steady-state thermal or gravitational drift.
            */
            LPF1_HPF = 0x04,

            /**
            * @brief Routes data through LPF1 and then Low-Pass Filter 2 (LPF2) to the interrupt engine.
            * @details The High-Pass Filter (HPF) is bypassed. Smoothes high-frequency structural noise
            * or chassis vibration to avoid generating false threshold interrupts.
            */
            LPF1_LPF2 = 0x08,

            /**
            * @brief Routes data through the full filter chain (LPF1 -> HPF -> LPF2) to the interrupt engine.
            * @details Maximizes interrupt path filtering. Evaluates threshold alerts after isolating low-frequency
            * drift and high-frequency noise spikes.
            */
            LPF1_HPF_LPF2 = 0x0C
        };

        /**
         * @brief Configuration values for the Gyroscope Output Selection Multiplexer bits.
         * @details These bits (OUT_SEL[1:0]) reside in CTRL_REG2_G (11h) [Bits 1:0]. They govern
         * the hardware digital signal routing to the main output registers and the FIFO buffer.
         */
        enum class OutputFilterPath : uint8_t
        {
            /**
             * @brief Default. Routes data through LPF1 only.
             * @details Both the High-Pass Filter (HPF) and Low-Pass Filter 2 (LPF2) are bypassed.
             * Provides raw data updates with the absolute minimum processing latency.
             */
            LPF1_ONLY = 0x00,

            /**
             * @brief Routes data through LPF1 and then the High-Pass Filter (HPF).
             * @details Low-Pass Filter 2 (LPF2) is bypassed. Ideal for application algorithms
             * requiring steady-state bias tracking (drift elimination) without secondary phase-lag.
             */
            LPF1_HPF = 0x01,

            /**
             * @brief Routes data through LPF1 and then Low-Pass Filter 2 (LPF2).
             * @details The High-Pass Filter (HPF) is bypassed. Employs aggressive high-frequency
             * low-pass smoothing to stabilize volatile readings.
             */
            LPF1_LPF2 = 0x02,

            /**
             * @brief Routes data through the full filter chain: LPF1 -> HPF -> LPF2.
             * @details Enables maximum data filtering. Removes low-frequency thermal or baseline drift
             * along with high-frequency mechanical jitters before writing to the output registers.
             */
            LPF1_HPF_LPF2 = 0x03
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg2G register
         */
        enum class BitMask : uint8_t
        {
            OutputFilterPath = 0x03,
            InterruptFilterPath = 0x0C,
        };

        /**
         * @brief Configuration structure for the Gyroscope Interrupt Selection Multiplexer and Output Selection Multiplexer.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(InterruptFilterPath imux, OutputFilterPath omux)
                : value(static_cast<uint8_t>(imux) | static_cast<uint8_t>(omux))
            {}
        };
    }

    namespace CtrlReg3G
    {
        /**
         * @brief Power configuration mode selector for the Gyroscope.
         * @details This enum sets the LP_mode bit (Bit 7) within CTRL_REG3_G (12h).
         */
        enum class PowerMode : uint8_t
        {
            /**
             * @brief Default. Configures Gyroscope to Normal Power Mode.
             * @details Full performance operational state.
             */
            Normal = 0x00,

            /**
             * @brief Configures Gyroscope to Eco Low-Power Mode.
             * @details Drastically lowers current consumption. This is only functional
             * if the selected ODR is 14.9 Hz, 59.5 Hz, or 119 Hz.
             */
            Low_Power = 0x80
        };

        /**
         * @brief Master hardware toggle switch for the High-Pass Filter (HPF).
         * @details This enum sets the HP_EN bit (Bit 6) within CTRL_REG3_G (12h).
         */
        enum class HighPassFilterState : uint8_t
        {
            /**
             * @brief Default. Bypasses the High-Pass filter framework entirely.
             */
            Disabled = 0x00,

            /**
             * @brief Activates the High-Pass filter framework pipeline block.
             * @details Signal path multiplexers in CTRL_REG2_G must also be oriented
             * toward HPF tracking for this calculation to affect outputs or interrupts.
             */
            Enabled = 0x40
        };

        /**
         * @brief High-Pass Filter (HPF) Cutoff Frequency Configuration.
         * @details These values set the HPCF_G[3:0] bits (Bits 3:0) within CTRL_REG3_G (12h).
         * The resulting baseline cutoff limits slide proportionally based on the active ODR.
         */
        enum class HighPassFilterCutoff : uint8_t
        {
            /**
             * @brief Default Cutoff Configuration.
             * - ODR  14.9 Hz: 1.0 Hz Cutoff
             * - ODR  59.5 Hz: 4.0 Hz Cutoff
             * - ODR 119.0 Hz: 8.0 Hz Cutoff
             * - ODR 238.0 Hz: 15.0 Hz Cutoff
             * - ODR 476.0 Hz: 30.0 Hz Cutoff
             * - ODR 952.0 Hz: 57.0 Hz Cutoff
             */
            HPCF_0000 = 0x00,

            /**
             * @brief Step 1 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.5 Hz Cutoff
             * - ODR  59.5 Hz: 2.0 Hz Cutoff
             * - ODR 119.0 Hz: 4.0 Hz Cutoff
             * - ODR 238.0 Hz: 8.0 Hz Cutoff
             * - ODR 476.0 Hz: 15.0 Hz Cutoff
             * - ODR 952.0 Hz: 30.0 Hz Cutoff
             */
            HPCF_0001 = 0x01,

            /**
             * @brief Step 2 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.2 Hz Cutoff
             * - ODR  59.5 Hz: 1.0 Hz Cutoff
             * - ODR 119.0 Hz: 2.0 Hz Cutoff
             * - ODR 238.0 Hz: 4.0 Hz Cutoff
             * - ODR 476.0 Hz: 8.0 Hz Cutoff
             * - ODR 952.0 Hz: 15.0 Hz Cutoff
             */
            HPCF_0010 = 0x02,

            /**
             * @brief Step 3 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.1 Hz Cutoff
             * - ODR  59.5 Hz: 0.5 Hz Cutoff
             * - ODR 119.0 Hz: 1.0 Hz Cutoff
             * - ODR 238.0 Hz: 2.0 Hz Cutoff
             * - ODR 476.0 Hz: 4.0 Hz Cutoff
             * - ODR 952.0 Hz: 8.0 Hz Cutoff
             */
            HPCF_0011 = 0x03,

            /**
             * @brief Step 4 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.05 Hz Cutoff
             * - ODR  59.5 Hz: 0.2 Hz Cutoff
             * - ODR 119.0 Hz: 0.5 Hz Cutoff
             * - ODR 238.0 Hz: 1.0 Hz Cutoff
             * - ODR 476.0 Hz: 2.0 Hz Cutoff
             * - ODR 952.0 Hz: 4.0 Hz Cutoff
             */
            HPCF_0100 = 0x04,

            /**
             * @brief Step 5 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.02 Hz Cutoff
             * - ODR  59.5 Hz: 0.1 Hz Cutoff
             * - ODR 119.0 Hz: 0.2 Hz Cutoff
             * - ODR 238.0 Hz: 0.5 Hz Cutoff
             * - ODR 476.0 Hz: 1.0 Hz Cutoff
             * - ODR 952.0 Hz: 2.0 Hz Cutoff
             */
            HPCF_0101 = 0x05,

            /**
             * @brief Step 6 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.01 Hz Cutoff
             * - ODR  59.5 Hz: 0.05 Hz Cutoff
             * - ODR 119.0 Hz: 0.1 Hz Cutoff
             * - ODR 238.0 Hz: 0.2 Hz Cutoff
             * - ODR 476.0 Hz: 0.5 Hz Cutoff
             * - ODR 952.0 Hz: 1.0 Hz Cutoff
             */
            HPCF_0110 = 0x06,

            /**
             * @brief Step 7 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.005 Hz Cutoff
             * - ODR  59.5 Hz: 0.02 Hz Cutoff
             * - ODR 119.0 Hz: 0.05 Hz Cutoff
             * - ODR 238.0 Hz: 0.1 Hz Cutoff
             * - ODR 476.0 Hz: 0.2 Hz Cutoff
             * - ODR 952.0 Hz: 0.5 Hz Cutoff
             */
            HPCF_0111 = 0x07,

            /**
             * @brief Step 8 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.002 Hz Cutoff
             * - ODR  59.5 Hz: 0.01 Hz Cutoff
             * - ODR 119.0 Hz: 0.02 Hz Cutoff
             * - ODR 238.0 Hz: 0.05 Hz Cutoff
             * - ODR 476.0 Hz: 0.1 Hz Cutoff
             * - ODR 952.0 Hz: 0.2 Hz Cutoff
             */
            HPCF_1000 = 0x08,

            /**
             * @brief Step 9 Cutoff Configuration.
             * - ODR  14.9 Hz: 0.001 Hz Cutoff
             * - ODR  59.5 Hz: 0.005 Hz Cutoff
             * - ODR 119.0 Hz: 0.01 Hz Cutoff
             * - ODR 238.0 Hz: 0.02 Hz Cutoff
             * - ODR 476.0 Hz: 0.05 Hz Cutoff
             * - ODR 952.0 Hz: 0.1 Hz Cutoff
             */
            HPCF_1001 = 0x09
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg3G register
         */
        enum class BitMask : uint8_t
        {
            HighPassFilterCutoff = 0x0F,
            HighPassFilterState = 0x40,
            PowerMode = 0x80,
        };

        /**
         * @brief Configuration structure for the Gyroscope Power Mode, High-Pass Filter State, and High-Pass Filter Cutoff Frequency.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(PowerMode mode, HighPassFilterState hpfState, HighPassFilterCutoff hpfCutoff)
                : value(static_cast<uint8_t>(mode) | static_cast<uint8_t>(hpfState) | static_cast<uint8_t>(hpfCutoff))
            {}
        };
    }

    /**
     * @brief Hardware register addresses for the LSM9DS1 Angular Rate (Gyroscope) outputs.
     * @details These registers hold the raw 16-bit signed (twos-complement) angular rate data
     * for each axis, split into two 8-bit registers (Low and High bytes).
     */
    enum class OutputRegister : uint8_t
    {
        /// Gyroscope sensor X-axis output register (Low Byte).
        Out_X_L_G = 0x18,
        /// Gyroscope sensor X-axis output register (High Byte).
        Out_X_H_G = 0x19,
        /// Gyroscope sensor Y-axis output register (Low Byte).
        Out_Y_L_G = 0x1A,
        /// Gyroscope sensor Y-axis output register (High Byte).
        Out_Y_H_G = 0x1B,
        /// Gyroscope sensor Z-axis output register (Low Byte).
        Out_Z_L_G = 0x1C,
        /// Gyroscope sensor Z-axis output register (High Byte).
        Out_Z_H_G = 0x1D
    };
}
#pragma endregion Gyroscope control registers

#pragma region Accelerometer control registers
namespace Accelerometer
{
    /**
     * @brief Hardware register addresses for the LSM9DS1 Accelerometer control registers.
     */
    enum class ControlRegister : uint8_t
    {
        /**
        * @brief Register 6: Accelerometer Control - CTRL_REG6_XL: 0x20. Initialization register for the accelerometer.
        * @details This register configures the output data rate, full-scale range, and bandwidth of the accelerometer.
        * The register layout is as follows:
        * Bit  7         6         5         4        3           2          1        0
        * +---------+---------+---------+--------+--------+-------------+--------+--------+
        * | ODR_XL2 | ODR_XL1 | ODR_XL0 | FS1_XL | FS0_XL | BW_SCAL_ODR | BW_XL1 | BW_XL0 |
        * +---------+---------+---------+--------+--------+-------------+--------+--------+
        * Bits 7-5: ODR_XL   (Output data rate and power mode selection) [Default value: 000]
        * Bits 4-3: FS_XL    (Full scale selection) [Default value: 00]
        * Bit 2: BW_SCAL_ODR (Bandwidth selection) [Default value: 0]
        * Bits 1-0: BW_XL    (Anti-aliasing filter bandwidth selection) [Default value: 00]
        */
        CtrlReg6XL = 0x20 // Register 6: Accelerometer Control
    };

    namespace CtrlReg6XL
    {
        /**
         * @brief Accelerometer output data rate configuration.
         * @details These bits (ODR_XL[2:0]) reside in CTRL_REG6_XL (20h) [Bits 7:5]. They define the output
         * data rate and power mode of the accelerometer. The default value is 000, which corresponds to a
         * power-down mode. The other options are 10 Hz (001), 50 Hz (010), 119 Hz (011), 238 Hz (100),
         * 476 Hz (101), and 952 Hz (110). The output data rate affects the frequency at which new
         * accelerometer data is available for reading.
         */
        enum class OutputDataRate : uint8_t
        {
            PowerDown = 0x00,
            Hz_10 = 0x20,  // 10 Hz output data rate.
            Hz_50 = 0x40,  // 50 Hz output data rate.
            Hz_119 = 0x60, // 119 Hz output data rate.
            Hz_238 = 0x80, // 238 Hz output data rate.
            Hz_476 = 0xA0, // 476 Hz output data rate.
            Hz_952 = 0xC0  // 952 Hz output data rate.
        };

        /**
         * @brief Accelerometer full-scale configuration.
         * @details These bits (FS_XL[1:0]) reside in CTRL_REG6_XL (20h) [Bits 4:3]. They define the
         * full-scale range of the accelerometer. The default value is 00, which corresponds to a
         * range of ±2g. The other options are ±16g (01), ±4g (10), and ±8g (11).
         */
        enum class Scale : uint8_t
        {
            G_2 = 0x00,  // Default. ±2g
            G_16 = 0x08, // ±16g
            G_4 = 0x10,  // ±4g
            G_8 = 0x18   // ±8g
        };

        /**
         * @brief Accelerometer bandwidth configuration.
         * @details This bit (BW_SCAL_ODR[0]) resides in CTRL_REG6_XL (20h) [Bit 2].
         * It defines the bandwidth of the accelerometer's anti-aliasing filter.
         */
        enum class Bandwidth : uint8_t
        {
            /**
             * @brief Bandwidth selection is determined by the output data rate (ODR) selection.
             * @details This is the default behavior.
             * The bandwidth is set as follows based on the ODR:
             * - ODR = 952 Hz, 50 Hz, 10 Hz: BW = 408 Hz
             * - ODR = 476 Hz: BW = 211 Hz
             * - ODR = 238 Hz: BW = 105 Hz
             * - ODR = 119 Hz: BW = 50 Hz
             */
            Automatic = 0x00, // Default. Bandwidth based on output data rate selection.
            /**
             * @brief Bandwidth selection is determined by the anti-aliasing filter bandwidth selection.
             * @details This allows for more precise control over the accelerometer's filtering
             * characteristics. Through the BW_XL[1:0] bits, the user can select specific cutoff
             * frequencies for the anti-aliasing filter.
             */
            Manual = 0x04 // Bandwidth based on anti-aliasing filter bandwidth selection.
        };

        /**
         * @brief Accelerometer anti-aliasing filter bandwidth configuration.
         * @details These bits (BW_XL[1:0]) reside in CTRL_REG6_XL (20h) [Bits 1:0]. They define the
         * cutoff frequency of the accelerometer's anti-aliasing filter. The default value is 00,
         * which corresponds to a cutoff frequency of 408 Hz. The other options are 211 Hz (01),
         * 105 Hz (10), and 50 Hz (11). The anti-aliasing filter is used to reduce high-frequency
         * noise in the accelerometer readings.
         */
        enum class AntiAliasFilter : uint8_t
        {
            Hz_408 = 0x00, // Default. 408 Hz
            Hz_211 = 0x01, // 211 Hz
            Hz_105 = 0x02, // 105 Hz
            Hz_50 = 0x03   // 50 Hz
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg6XL register
         */
        enum class BitMask : uint8_t
        {
            AntiAliasFilter = 0x03,
            Bandwidth = 0x04,
            Scale = 0x18,
            OutputDataRate = 0xE0,
        };

        /**
         * @brief Configuration structure for the Accelerometer Output Data Rate, Full-Scale Range, Bandwidth, and Anti-Alias Filter.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(OutputDataRate odr, Scale scale, Bandwidth bandwidth, AntiAliasFilter filter)
                : value(static_cast<uint8_t>(odr) | static_cast<uint8_t>(scale) | static_cast<uint8_t>(bandwidth) | static_cast<uint8_t>(filter))
            {}
        };

        constexpr Scale GetScale(Configuration config)
        {
            return static_cast<Scale>(config.value & static_cast<uint8_t>(BitMask::Scale));
        }
    };

    /**
     * @brief Hardware register addresses for the LSM9DS1 Linear Acceleration (Accelerometer) outputs.
     * @details These registers hold the raw 16-bit signed (twos-complement) acceleration data
     * for each axis, split into two 8-bit registers (Low and High bytes).
     */
    enum class OutputRegister : uint8_t
    {
        /// Accelerometer sensor X-axis output register (Low Byte).
        Out_X_L_XL = 0x28,
        /// Accelerometer sensor X-axis output register (High Byte).
        Out_X_H_XL = 0x29,
        /// Accelerometer sensor Y-axis output register (Low Byte).
        Out_Y_L_XL = 0x2A,
        /// Accelerometer sensor Y-axis output register (High Byte).
        Out_Y_H_XL = 0x2B,
        /// Accelerometer sensor Z-axis output register (Low Byte).
        Out_Z_L_XL = 0x2C,
        /// Accelerometer sensor Z-axis output register (High Byte).
        Out_Z_H_XL = 0x2D
    };
}
#pragma endregion Accelerometer control registers

#pragma region Magnetometer control registers
namespace Magnetometer
{
    /**
     * @brief Hardware register addresses for the LSM9DS1 Magnetometer control registers.
     */
    enum class ControlRegister : uint8_t
    {
        /**
         * @brief Register 1: Magnetometer Control - CTRL_REG1_M: 0x20.
         * @details This register configures the temperature compensation, X/Y-axis operating mode,
         * output data rate, and fast output data rate options of the magnetometer.
         * The register layout is as follows:
         * Bit   7       6     5     4     3     2        1       0
         * +----------+-----+-----+-----+-----+-----+----------+----+
         * | TEMP_COMP| OM1 | OM0 | DO2 | DO1 | DO0 | FAST_ODR | ST |
         * +----------+-----+-----+-----+-----+-----+----------+----+
         * Bit 7:    TEMP_COMP (Temperature compensation enable) [Default value: 0]
         * Bits 6-5: OM[1:0] (X/Y-axis operating mode) [Default value: 00]
         * Bits 4-2: DO[2:0] (Output data rate selection) [Default value: 100]
         * Bit 1:    FAST_ODR (Fast output data rate selection) [Default value: 0]
         * Bit 0:    ST (Self-test enable) [Default value: 0]
         */
        CtrlReg1M = 0x20,

        /**
         * @brief Register 2: Magnetometer Control - CTRL_REG2_M: 0x21.
         * @details This register configures the full-scale range, memory reboot function, and software reset function
         * of the magnetometer.
         * The register layout is as follows:
         * Bit 7    6     5    4      3         2      1   0
         * +-----+-----+-----+---+--------+----------+---+---+
         * |   0 | FS1 | FS0 | 0 | REBOOT | SOFT_RST | 0 | 0 |
         * +-----+-----+-----+---+--------+----------+---+---+
         * Bit 7: Reserved [Default value: 0]
         * Bits 6-5: FS[1:0] (Full-scale configuration) [Default value: 00]
         * Bit 4: Reserved [Default value: 0]
         * Bit 3: REBOOT (Reboot memory content) [Default value: 0]
         * Bit 2: SOFT_RST (Configuration and user register reset) [Default value: 0]
         * Bits 1-0: Reserved [Default value: 00]
         */
        CtrlReg2M = 0x21,

        /**
         * @brief Register 3: Magnetometer Control - CTRL_REG3_M: 0x22. Initialization register for the magnetometer.
         * @details This register configures the I2C interface, low-power mode, SPI serial interface mode, and operating mode
         * of the magnetometer.
         * The register layout is as follows:
         * Bit    7        6    5   4   3    2     1     0
         * +-------------+---+----+---+---+-----+-----+-----+
         * | I2C_DISABLE | 0 | LP | 0 | 0 | SIM | MD1 | MD0 |
         * +-------------+---+----+---+---+-----+-----+-----+
         * Bit 7: I2C_DISABLE (Disable I2C interface) [Default value: 0]
         * Bit 6: Reserved    [Default value: 0]
         * Bit 5: LP          (Low-power mode configuration) [Default value: 0]
         * Bits 4-3: Reserved [Default value: 00]
         * Bit 2: SIM         (SPI serial interface mode selection) [Default value: 0]
         * Bits 1-0: MD[1:0]  (Operating mode selection) [Default value: 11]
         */
        CtrlReg3M = 0x22,

        /**
         * @brief Register 4: Magnetometer Control - CTRL_REG4_M: 0x23.
         * @details This register configures the Z-axis operating mode and data byte ordering of the magnetometer.
         * The register layout is as follows:
         * Bit 7   6   5   4     3      2     1    0
         * + ----+---+---+---+------+------+-----+---+
         * |   0 | 0 | 0 | 0 | OMZ1 | OMZ0 | BLE | 0 |
         * + ----+---+---+---+------+------+-----+---+
         * Bits 7-4: Reserved [Default value: 0000]
         * Bits 3-2: OMZ[1:0] (Z-axis operative mode selection) [Default value: 00]
         * Bit 1: BLE (Big/Little Endian data selection) [Default value: 0]
         * Bit 0: Reserved [Default value: 0]
         */
        CtrlReg4M = 0x23,

        /**
         * @brief Register 5: Magnetometer Control - CTRL_REG5_M: 0x24.
         * @details This register configures the block data update and fast read functions of the magnetometer.
         * The register layout is as follows:
         * Bit   7        6    5   4   3   2   1   0
         * +-----------+-----+---+---+---+---+---+---+
         * | FAST_READ | BDU | 0 | 0 | 0 | 0 | 0 | 0 |
         * +-----------+-----+---+---+---+---+---+---+
         * Bit 7: BDU         (Block data update) [Default value: 0]
         * Bit 6: FAST_READ   (Fast read enable) [Default value: 0]
         * Bits 5-0: Reserved [Default value: 000000]
         */
        CtrlReg5M = 0x24
    };

    namespace CtrlReg1M
    {
        /**
         * @brief Magnetometer temperature compensation configuration.
         * @details This bit (TEMP_COMP) resides in CTRL_REG1_M (20h) [Bit 7].
         * It enables or disables temperature compensation for the magnetometer.
         */
        enum class TemperatureCompensation : uint8_t
        {
            Disabled = 0x00,
            Enabled = 0x80
        };

        /**
         * @brief Magnetometer X/Y-axis operating mode configuration.
         * @details These bits (OM[1:0]) reside in CTRL_REG1_M (20h) [Bits 6:5].
         * They define the operating mode of the X and Y axes of the magnetometer.
         */
        enum class OperatingModeXYAxis : uint8_t
        {
            LowPerformance = 0x00,      // Default. Low-performance
            MediumPerformance = 0x20,   // Medium-performance
            HighPerformance = 0x40,     // High-performance
            UltraHighPerformance = 0x60 // Ultra-high-performance
        };

        /**
         * @brief Magnetometer output data rate configuration.
         * @details These bits (DO[2:0]) reside in CTRL_REG1_M (20h) [Bits 4:2].
         * They define the output data rate of the magnetometer.
         */
        enum class OutputDataRate : uint8_t
        {
            Hz_0_625 = 0x00, // 0.625 Hz
            Hz_1_25 = 0x04,  // 1.25 Hz
            Hz_2_5 = 0x08,   // 2.5 Hz
            Hz_5 = 0x0C,     // 5 Hz
            Hz_10 = 0x10,    // Default. 10 Hz
            Hz_20 = 0x14,    // 20 Hz
            Hz_40 = 0x18,    // 40 Hz
            Hz_80 = 0x1C     // 80 Hz
        };

        /**
         * @brief Magnetometer fast output data rate configuration.
         * @details This bit (FAST_ODR) resides in CTRL_REG1_M (20h) [Bit 1].
         * It enables the fast output data rate mode.
         */
        enum class FastOutputDataRate : uint8_t
        {
            Disabled = 0x00,
            Enabled = 0x02
        };

        /**
         * @brief Magnetometer self-test configuration.
         * @details This bit (ST) resides in CTRL_REG1_M (20h) [Bit 0].
         * It enables or disables the magnetometer self-test function.
         */
        enum class SelfTest : uint8_t
        {
            Disabled = 0x00,
            Enabled = 0x01
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg1M register.
         */
        enum class BitMask : uint8_t
        {
            SelfTest = 0x01,
            FastOutputDataRate = 0x02,
            OutputDataRate = 0x1C,
            OperatingModeXYAxis = 0x60,
            TemperatureCompensation = 0x80,
        };

        /**
         * @brief Configuration structure for the Magnetometer Temperature Compensation, X/Y-axis Operating Mode,
         * Output Data Rate, Fast Output Data Rate, and Self-Test settings.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(
                TemperatureCompensation temperatureCompensation,
                OperatingModeXYAxis operatingModeXYAxis,
                OutputDataRate odr,
                FastOutputDataRate fastOdr,
                SelfTest selfTest)
                : value(
                    static_cast<uint8_t>(temperatureCompensation) |
                    static_cast<uint8_t>(operatingModeXYAxis) |
                    static_cast<uint8_t>(odr) |
                    static_cast<uint8_t>(fastOdr) |
                    static_cast<uint8_t>(selfTest))
            {}
        };

        constexpr TemperatureCompensation GetTemperatureCompensation(Configuration config)
        {
            return static_cast<TemperatureCompensation>(config.value & static_cast<uint8_t>(BitMask::TemperatureCompensation));
        }

        constexpr OperatingModeXYAxis GetOperatingModeXYAxis(Configuration config)
        {
            return static_cast<OperatingModeXYAxis>(config.value & static_cast<uint8_t>(BitMask::OperatingModeXYAxis));
        }

        constexpr OutputDataRate GetOutputDataRate(Configuration config)
        {
            return static_cast<OutputDataRate>(config.value & static_cast<uint8_t>(BitMask::OutputDataRate));
        }
    }

    namespace CtrlReg2M
    {
        /**
         * @brief Magnetometer full-scale configuration.
         * @details These bits (FS[1:0]) reside in CTRL_REG2_M (21h) [Bits 6:5].
         * They define the full-scale range of the magnetometer. The default value is 00,
         * which corresponds to a range of ±4 gauss. The other options are ±8 gauss (01),
         * ±12 gauss (10), and ±16 gauss (11).
         */
        enum class Scale : uint8_t
        {
            Gauss_4 = 0x00,  // Default. ±4 gauss.
            Gauss_8 = 0x20,  // ±8 gauss.
            Gauss_12 = 0x40, // ±12 gauss.
            Gauss_16 = 0x60  // ±16 gauss.
        };

        /**
         * @brief Magnetometer memory reboot configuration.
         * @details This bit (REBOOT) resides in CTRL_REG2_M (21h) [Bit 3].
         * It controls whether the memory content is rebooted. The default value is 0,
         * which selects normal operating mode. Setting this bit to 1 reboots the memory content.
         */
        enum class Reboot : uint8_t
        {
            Normal = 0x00, // Default. Normal operating mode.
            Reboot = 0x08  // Reboot memory content.
        };

        /**
         * @brief Magnetometer software reset configuration.
         * @details This bit (SOFT_RST) resides in CTRL_REG2_M (21h) [Bit 2].
         * It controls the configuration and user register reset function. The default value is 0.
         * Setting this bit to 1 initiates a reset operation.
         */
        enum class SoftReset : uint8_t
        {
            Normal = 0x00, // Default. Normal operation.
            Reset = 0x04   // Initiate software reset operation.
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg2M register.
         */
        enum class BitMask : uint8_t
        {
            SoftReset = 0x04,
            Reboot = 0x08,
            Scale = 0x60,
        };

        /**
         * @brief Configuration structure for the Magnetometer Full-Scale Range, Memory Reboot, and Software Reset settings.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(Scale scale, Reboot reboot, SoftReset softReset)
                : value(static_cast<uint8_t>(scale) | static_cast<uint8_t>(reboot) | static_cast<uint8_t>(softReset))
            {}
        };

        constexpr Scale GetScale(Configuration config)
        {
            return static_cast<Scale>(config.value & static_cast<uint8_t>(BitMask::Scale));
        }

        constexpr Reboot GetReboot(Configuration config)
        {
            return static_cast<Reboot>(config.value & static_cast<uint8_t>(BitMask::Reboot));
        }

        constexpr SoftReset GetSoftReset(Configuration config)
        {
            return static_cast<SoftReset>(config.value & static_cast<uint8_t>(BitMask::SoftReset));
        }
    }

    namespace CtrlReg3M
    {
        /**
         * @brief Magnetometer I2C interface configuration.
         * @details This bit (I2C_DISABLE) resides in CTRL_REG3_M (22h) [Bit 7].
         * It enables or disables the I2C interface of the magnetometer.
         */
        enum class I2CInterface : uint8_t
        {
            Enabled = 0x00,  // Default. I2C interface enabled.
            Disabled = 0x80  // I2C interface disabled.
        };

        /**
         * @brief Magnetometer low-power mode configuration.
         * @details This bit (LP) resides in CTRL_REG3_M (22h) [Bit 5].
         * When enabled, the magnetometer output data rate is forced to 0.625 Hz and the system
         * performs the minimum number of averages for each channel. When disabled, the magnetic
         * data rate is configured by the DO[2:0] bits in CTRL_REG1_M (20h).
         */
        enum class LowPowerMode : uint8_t
        {
            Disabled = 0x00, // Default. Output data rate is configured by CTRL_REG1_M DO[2:0].
            Enabled = 0x20   // Output data rate is forced to 0.625 Hz with minimum averaging.
        };

        /**
         * @brief Magnetometer SPI serial interface mode configuration.
         * @details This bit (SIM) resides in CTRL_REG3_M (22h) [Bit 2].
         * It selects the SPI serial interface mode. The default value enables SPI write-only
         * operations. Setting this bit enables both SPI read and write operations.
         */
        enum class SpiInterfaceMode : uint8_t
        {
            WriteOnly = 0x00, // Default. SPI write operations enabled.
            ReadWrite = 0x04  // SPI read and write operations enabled.
        };

        /**
         * @brief Magnetometer operating mode configuration.
         * @details These bits (MD[1:0]) reside in CTRL_REG3_M (22h) [Bits 1:0].
         * They define the operating mode of the magnetometer. The default value is 11,
         * which places the magnetometer in power-down mode.
         */
        enum class OperatingMode : uint8_t
        {
            ContinuousConversion = 0x00, // Continuous-conversion mode.
            SingleConversion = 0x01,     // Single-conversion mode.
            PowerDown_10 = 0x02,         // Power-down mode.
            PowerDown_11 = 0x03          // Default. Power-down mode.
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg3M register.
         */
        enum class BitMask : uint8_t
        {
            I2CInterface = 0x80,
            LowPowerMode = 0x20,
            SpiInterfaceMode = 0x04,
            OperatingMode = 0x03
        };

        /**
         * @brief Configuration structure for the Magnetometer I2C Interface, Low-Power Mode,
         * SPI Interface Mode, and Operating Mode settings.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(
                I2CInterface i2cInterface,
                LowPowerMode lowPowerMode,
                SpiInterfaceMode spiInterfaceMode,
                OperatingMode operatingMode)
                : value(
                    static_cast<uint8_t>(i2cInterface) |
                    static_cast<uint8_t>(lowPowerMode) |
                    static_cast<uint8_t>(spiInterfaceMode) |
                    static_cast<uint8_t>(operatingMode))
            {}
        };

        constexpr I2CInterface GetI2CInterface(Configuration config)
        {
            return static_cast<I2CInterface>(config.value & static_cast<uint8_t>(BitMask::I2CInterface));
        }

        constexpr LowPowerMode GetLowPowerMode(Configuration config)
        {
            return static_cast<LowPowerMode>(config.value & static_cast<uint8_t>(BitMask::LowPowerMode));
        }

        constexpr SpiInterfaceMode GetSpiInterfaceMode(Configuration config)
        {
            return static_cast<SpiInterfaceMode>(config.value & static_cast<uint8_t>(BitMask::SpiInterfaceMode));
        }

        constexpr OperatingMode GetOperatingMode(Configuration config)
        {
            return static_cast<OperatingMode>(config.value & static_cast<uint8_t>(BitMask::OperatingMode));
        }
    }

    namespace CtrlReg4M
    {
        /**
         * @brief Magnetometer Z-axis operating mode configuration.
         * @details These bits (OMZ[1:0]) reside in CTRL_REG4_M (23h) [Bits 3:2].
         * They define the operating mode of the Z-axis of the magnetometer. The default value is 00,
         * which corresponds to low-performance mode. The other options are medium-performance mode (01),
         * high-performance mode (10), and ultra-high-performance mode (11).
         */
        enum class OperatingModeZAxis : uint8_t
        {
            LowPerformance = 0x00,      // Default. Low-performance mode.
            MediumPerformance = 0x04,   // Medium-performance mode.
            HighPerformance = 0x08,     // High-performance mode.
            UltraHighPerformance = 0x0C // Ultra-high-performance mode.
        };

        /**
         * @brief Magnetometer data endianness configuration.
         * @details This bit (BLE) resides in CTRL_REG4_M (23h) [Bit 1].
         * It selects the byte order of the magnetometer output data. When disabled, the least
         * significant byte is stored at the lower register address. When enabled, the most
         * significant byte is stored at the lower register address.
         */
        enum class Endianness : uint8_t
        {
            LittleEndian = 0x00, // Default. Least significant byte at lower address.
            BigEndian = 0x02     // Most significant byte at lower address.
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg4M register.
         */
        enum class BitMask : uint8_t
        {
            OperatingModeZAxis = 0x0C,
            Endianness = 0x02
        };

        /**
         * @brief Configuration structure for the Magnetometer Z-axis Operating Mode and Data Endianness settings.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(OperatingModeZAxis operatingModeZAxis, Endianness endianness)
                : value(static_cast<uint8_t>(operatingModeZAxis) | static_cast<uint8_t>(endianness))
            {}
        };

        constexpr OperatingModeZAxis GetOperatingModeZAxis(Configuration config)
        {
            return static_cast<OperatingModeZAxis>(config.value & static_cast<uint8_t>(BitMask::OperatingModeZAxis));
        }

        constexpr Endianness GetEndianness(Configuration config)
        {
            return static_cast<Endianness>(config.value & static_cast<uint8_t>(BitMask::Endianness));
        }
    }

    namespace CtrlReg5M
    {
        /**
         * @brief Magnetometer high-byte fast-read configuration.
         * @details This bit (FAST_READ) resides in CTRL_REG5_M (24h) [Bit 7].
         * When disabled, the complete magnetometer output data can be read.
         * When enabled, only the high part of the DATA OUT registers is read,
         * increasing reading efficiency.
         */
        enum class FastRead : uint8_t
        {
            Disabled = 0x00, // Default. FAST_READ disabled.
            Enabled = 0x80   // FAST_READ enabled. Only the high part of DATA OUT is read.
        };

        /**
         * @brief Magnetometer block data update configuration.
         * @details This bit (BDU) resides in CTRL_REG5_M (24h) [Bit 6].
         * When disabled, the magnetometer output registers are continuously updated.
         * When enabled, the output registers are not updated until both the MSB and LSB
         * registers of the magnetic data have been read.
         */
        enum class BlockDataUpdate : uint8_t
        {
            Continuous = 0x00, // Default. Output registers continuously updated.
            Blocked = 0x40     // Output registers not updated until MSB and LSB have been read.
        };

        /**
         * @brief The bitmask for each segment of the CtrlReg5M register.
         */
        enum class BitMask : uint8_t
        {
            FastRead = 0x80,
            BlockDataUpdate = 0x40
        };

        /**
         * @brief Configuration structure for the Magnetometer FAST_READ and Block Data Update settings.
         */
        struct Configuration
        {
            const uint8_t value;

            constexpr Configuration(FastRead fastRead, BlockDataUpdate blockDataUpdate)
                : value(static_cast<uint8_t>(fastRead) | static_cast<uint8_t>(blockDataUpdate))
            {}
        };

        constexpr FastRead GetFastRead(Configuration config)
        {
            return static_cast<FastRead>(config.value & static_cast<uint8_t>(BitMask::FastRead));
        }

        constexpr BlockDataUpdate GetBlockDataUpdate(Configuration config)
        {
            return static_cast<BlockDataUpdate>(config.value & static_cast<uint8_t>(BitMask::BlockDataUpdate));
        }
    }

    /**
     * @brief Hardware register addresses for the LSM9DS1 Magnetometer output registers.
     * @details These registers hold the raw 16-bit signed (twos-complement) magnetic field data
     * for each axis, split into two 8-bit registers (Low and High bytes).
     */
    enum class OutputRegister : uint8_t
    {
        /// Magnetometer sensor X-axis output register (Low Byte).
        Out_X_L_M = 0x28,
        /// Magnetometer sensor X-axis output register (High Byte).
        Out_X_H_M = 0x29,
        /// Magnetometer sensor Y-axis output register (Low Byte).
        Out_Y_L_M = 0x2A,
        /// Magnetometer sensor Y-axis output register (High Byte).
        Out_Y_H_M = 0x2B,
        /// Magnetometer sensor Z-axis output register (Low Byte).
        Out_Z_L_M = 0x2C,
        /// Magnetometer sensor Z-axis output register (High Byte).
        Out_Z_H_M = 0x2D
    };
}
#pragma endregion Magnetometer control registers