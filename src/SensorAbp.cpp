/*!
 * @file SensorABP.cpp
 * 
 * @mainpage mestrode ABP Library
 * 
 * @section intro_sec Introduction
 * 
 * This is a library for the Honywell ABP-Series Pressure Sensor
 * 
 * Refered Documentation by Honeywell
 * https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/ja/products/sensors/pressure-sensors/common/documents/sps-siot-spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-ciid-45843.pdf
 * 
 * @section author Author
 * 
 * Written by mestrode (contact via Github)
 * 
 * @section license License
 * 
 * GPL 2.0, all text above must be included in any redistribution
 * 
*/

#include "SensorAbp.hpp"

#include <SPI.h>

SensorAbp::SensorAbp(PinName pin_SCK, PinName pin_MOSI, PinName pin_MISO, PinName pin_SS):
    _pin_SS(pin_SS)
{
    pin_mode(pin_SCK, OUTPUT);
    pin_mode(pin_MOSI, OUTPUT);
    pin_mode(pin_MISO, INPUT);
    pin_mode(pin_SS, OUTPUT);
    digitalWrite(pin_SS, HIGH);

    SPI.begin();
}

/// @brief read Sensor values
/// @details results are provided in: status, pressure, temperature
/// @return sensor status (see datasheet)
SensorAbp::sensorAbpStatus_t SensorAbp::read()
{
    digitalWrite(_pin_SS, LOW);
    SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0););
    uint16_t response_status_pressure = SPI.transfer16(0x0000);
    uint16_t response_temp = SPI.transfer16(0x0000);
    SPI.endTransaction();
    digitalWrite(_pin_SS, HIGH);

    // decode status
    status = (sensorAbpStatus_t)(response_status_pressure >> (6 + 8));

    // decode and convert pressure
    uint16_t raw_pressure = response_status_pressure & 0x3FFF;
    pressure = convertRawPressure(raw_pressure);

    // decode and convert temperature
    uint16_t raw_temperature = response_temp >> 5;
    temperature = convertRawTemperature(raw_temperature);

    return status;
}

/// @brief Convert Pressure raw --> physical value
/// @param P_raw Sensor raw value in digit
/// @return physical value in mbar
float SensorAbp::convertRawPressure(uint16_t P_raw)
{
    constexpr float P_phy_diff = P_phy_max - P_phy_min;
    constexpr uint16_t P_raw_diff = P_raw_max - P_raw_min;
    constexpr float fact = P_phy_diff / (float)(P_raw_diff);

    float P_phy = (float)(P_raw - P_raw_min) * fact;
    P_phy = P_phy + P_phy_min;

    return P_phy;
}

/// @brief Convert Temperature raw --> physical value
/// @param T_raw Sensor raw value in digit
/// @return physical value in Celsius
float SensorAbp::convertRawTemperature(uint16_t T_raw)
{
    constexpr float T_phy_diff = T_phy_max - T_phy_min;
    constexpr uint16_t T_raw_diff = T_raw_max - T_raw_min;
    constexpr float T_fact = T_phy_diff / (float)(T_raw_diff);

    float T_phy = (float)(T_raw) * T_fact;
    T_phy = T_phy + T_phy_min;

    return T_phy;
}
