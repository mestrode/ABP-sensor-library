/*!
 * @file SensorABP.hpp
 * 
 * This is a library for the Honywell ABP-Series Pressure Sensor
 * 
 * Refered Documentation by Honeywell
 * https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/ja/products/sensors/pressure-sensors/common/documents/sps-siot-spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-ciid-45843.pdf
 * 
 * Written by mestrode (contact via Github)
 * 
 * @section license License
 * 
 * GPL 2.0, all text above must be included in any redistribution
 * 
*/

#pragma once

#include <Arduino.h>

class SensorAbp {
private:
    // Pressure conversation
    static constexpr float P_phy_max = 60.0; // mbar
    static constexpr float P_phy_min = 0.0;  // mbar
    static constexpr uint16_t P_raw_max = 0x3999; // 90% of 2^14
    static constexpr uint16_t P_raw_min = 0x0666; // 10% of 2^14

    // Temperature conversation
    static constexpr float T_phy_max = 150.0; // Kelvin
    static constexpr float T_phy_min = -50.0; // Kelvin
    static constexpr uint16_t T_raw_max = 2047; // 2^11-1
    static constexpr uint16_t T_raw_min = 0; // 0

public:
    enum sensorAbpStatus_t
    {
        not_initialized = -1,
        normal = 0,
        cmd_mode = 1,
        stale_data = 2,
        diagnostic = 3,
    };

    sensorAbpStatus_t status = not_initialized;
    float pressure = 0.0; // mbar
    float temperature = 0.0; // Celsius

    SensorAbp(PinName pin_SCK, PinName pin_MOSI, PinName pin_MISO, PinName pin_SS);

    sensorAbpStatus_t read();
    sensorAbpStatus_t readPressure();
    sensorAbpStatus_t readPressureTemperature8();
    sensorAbpStatus_t readPressureTemperature11();

private:
    PinName _pin_SS;

    float convertRawPressure(const uint16_t P_raw);
    float convertRawTemperature11(const uint16_t T_raw);
};
