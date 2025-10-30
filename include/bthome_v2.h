/*
 * BTHomeV2 Protocol Constants and Encoding
 * 
 * BTHome is an open standard for generic information to be encoded 
 * in BLE advertisements in an efficient way.
 * https://bthome.io
 * 
 * Based on: https://github.com/mhaberler/BTHomeV2-ESP32-example
 * This header defines object IDs and encoding formats for BTHomeV2
 */

#ifndef BTHOME_V2_H
#define BTHOME_V2_H

#include <cstdint>
#include <cstring>

// ============ BTHomeV2 BLE Advertisement Format ============
#define BTHOME_FLAG1                     0x02      // BLE Flags
#define BTHOME_FLAG2                     0x01      // Discoverable mode
#define BTHOME_FLAG3                     0x06      // General discoverable + BR/EDR not supported

#define BTHOME_SERVICE_UUID              0xFCD2    // BTHome Service UUID (little-endian: 0xD2FC)
#define BTHOME_UUID_BYTE1                0xD2      // UUID first byte
#define BTHOME_UUID_BYTE2                0xFC      // UUID second byte

#define BTHOME_SERVICE_DATA              0x16      // Service Data AD Type
#define BTHOME_COMPLETE_NAME             0x09      // Complete Local Name AD Type
#define BTHOME_SHORT_NAME                0x08      // Shortened Local Name AD Type

// ============ BTHomeV2 Encryption Flags ============
#define BTHOME_NO_ENCRYPT                0x40      // Not encrypted, no trigger-based device
#define BTHOME_NO_ENCRYPT_TRIGGER        0x44      // Not encrypted, trigger-based device
#define BTHOME_ENCRYPT                   0x41      // Encrypted, not trigger-based
#define BTHOME_ENCRYPT_TRIGGER           0x45      // Encrypted, trigger-based device

// ============ BTHomeV2 Object IDs ============

// Standard Object IDs (from BTHome v2 spec)
#define BTHOME_OID_PACKET                0x00      // uint8
#define BTHOME_OID_BATTERY_PERCENT       0x01      // uint8, %
#define BTHOME_OID_TEMPERATURE_PRECISE   0x02      // sint16, 0.01 °C
#define BTHOME_OID_HUMIDITY_PRECISE      0x03      // uint8, %
#define BTHOME_OID_PRESSURE              0x04      // uint24, Pa
#define BTHOME_OID_ILLUMINANCE           0x05      // uint24
#define BTHOME_OID_MASS                  0x06      // uint16
#define BTHOME_OID_MASSLB                0x07      // uint16
#define BTHOME_OID_DEWPOINT              0x08      // sint16, 0.01 °C
#define BTHOME_OID_POWER                 0x0B      // uint24, W
#define BTHOME_OID_ENERGY                0x0A      // uint32, kWh
#define BTHOME_OID_VOLTAGE               0x0C      // uint16, 0.001 V
#define BTHOME_OID_CO2                   0x12      // uint16, ppm
#define BTHOME_OID_TVOC                  0x13      // uint16
#define BTHOME_OID_HUMIDITY              0x2E      // uint8, %
#define BTHOME_OID_MOISTURE              0x2F      // uint8, %

// Using STANDARD BTHomeV2 object IDs for proper Home Assistant recognition
#define BTHOME_OID_PRESSURE              0x04      // uint24, 0.01 hPa (always positive)
#define BTHOME_OID_COUNT4                0x3E      // uint32, counter (always positive)
#define BTHOME_OID_DISTANCE_M            0x41      // uint16, 0.1 m (altitude in meters * 10)
#define BTHOME_OID_SPEED                 0x44      // uint16, 0.01 m/s (positive values work great!)
#define BTHOME_OID_ACCELERATION          0x51      // uint16, 0.001 m/s² (positive values work great!)

// Event/State Object IDs
#define BTHOME_OID_GENERIC_BOOLEAN       0x0F      // uint8 (0/1)
#define BTHOME_OID_POWER_ON              0x10      // uint8 (0/1)

// Extended Object IDs
#define BTHOME_OID_COUNT                 0x09      // uint8
#define BTHOME_OID_COUNT2                0x3D      // uint16
#define BTHOME_OID_COUNT4                0x3E      // uint32
#define BTHOME_OID_ROTATION              0x3F      // uint16
#define BTHOME_OID_DISTANCE              0x40      // uint16
#define BTHOME_OID_DISTANCEM             0x41      // uint16
#define BTHOME_OID_DURATION              0x42      // uint16
#define BTHOME_OID_CURRENT               0x43      // uint16
#define BTHOME_OID_SPEED                 0x44      // uint16
#define BTHOME_OID_TEMPERATURE           0x45      // sint16, 0.01 °C
#define BTHOME_OID_UV                    0x46      // uint8
#define BTHOME_OID_VOLUME1               0x47      // uint16
#define BTHOME_OID_VOLUME2               0x48      // uint32
#define BTHOME_OID_VOLUMEFR              0x49      // uint16
#define BTHOME_OID_VOLTAGE1              0x4A      // uint16, 0.01 V
#define BTHOME_OID_GAS                   0x4B      // uint24
#define BTHOME_OID_GAS4                  0x4C      // uint32
#define BTHOME_OID_ENERGY4               0x4D      // uint32
#define BTHOME_OID_VOLUME                0x4E      // uint32
#define BTHOME_OID_WATER                 0x4F      // uint32
#define BTHOME_OID_TEXT                  0x53      // string
#define BTHOME_OID_RAW                   0x54      // raw bytes

// State Object IDs
#define BTHOME_STATE_BATTERY_LOW         0x15      // uint8
#define BTHOME_STATE_CHARGING            0x16      // uint8
#define BTHOME_STATE_CO                  0x17      // uint8
#define BTHOME_STATE_COLD                0x18      // uint8
#define BTHOME_STATE_CONNECTIVITY        0x19      // uint8
#define BTHOME_STATE_DOOR                0x1A      // uint8
#define BTHOME_STATE_GARAGE_DOOR         0x1B      // uint8
#define BTHOME_STATE_GAS_DETECTED        0x1C      // uint8
#define BTHOME_STATE_HEAT                0x1D      // uint8
#define BTHOME_STATE_LIGHT               0x1E      // uint8
#define BTHOME_STATE_LOCK                0x1F      // uint8
#define BTHOME_STATE_MOISTURE            0x20      // uint8
#define BTHOME_STATE_MOTION              0x21      // uint8
#define BTHOME_STATE_MOVING              0x22      // uint8
#define BTHOME_STATE_OCCUPANCY           0x23      // uint8
#define BTHOME_STATE_PLUG                0x24      // uint8
#define BTHOME_STATE_PRESENCE            0x25      // uint8
#define BTHOME_STATE_PROBLEM             0x26      // uint8
#define BTHOME_STATE_RUNNING             0x27      // uint8
#define BTHOME_STATE_SAFETY              0x28      // uint8
#define BTHOME_STATE_SMOKE               0x29      // uint8
#define BTHOME_STATE_SOUND               0x2A      // uint8
#define BTHOME_STATE_TAMPER              0x2B      // uint8
#define BTHOME_STATE_VIBRATION           0x2C      // uint8
#define BTHOME_STATE_WINDOW              0x2D      // uint8

// Event Object IDs
#define BTHOME_EVENT_BUTTON              0x3A      // uint8
#define BTHOME_EVENT_BUTTON_PRESS        0x01
#define BTHOME_EVENT_BUTTON_DOUBLE_PRESS 0x02
#define BTHOME_EVENT_BUTTON_TRIPLE_PRESS 0x03
#define BTHOME_EVENT_BUTTON_LONG_PRESS   0x04
#define BTHOME_EVENT_DIMMER              0x3C      // uint8
#define BTHOME_EVENT_DIMMER_NONE         0x00
#define BTHOME_EVENT_DIMMER_LEFT         0x01
#define BTHOME_EVENT_DIMMER_RIGHT        0x02


// ============ Encoding Helpers ============

/**
 * Encode a uint8 value into buffer at offset
 */
inline void bthome_encode_uint8(uint8_t *buffer, uint8_t &offset, uint8_t value) {
    buffer[offset++] = value;
}

/**
 * Encode a uint16 value (little-endian) into buffer at offset
 */
inline void bthome_encode_uint16(uint8_t *buffer, uint8_t &offset, uint16_t value) {
    buffer[offset++] = value & 0xFF;
    buffer[offset++] = (value >> 8) & 0xFF;
}

/**
 * Encode a sint16 value (little-endian) into buffer at offset
 */
inline void bthome_encode_sint16(uint8_t *buffer, uint8_t &offset, int16_t value) {
    bthome_encode_uint16(buffer, offset, (uint16_t)value);
}

/**
 * Encode a uint24 value (little-endian) into buffer at offset
 */
inline void bthome_encode_uint24(uint8_t *buffer, uint8_t &offset, uint32_t value) {
    buffer[offset++] = value & 0xFF;
    buffer[offset++] = (value >> 8) & 0xFF;
    buffer[offset++] = (value >> 16) & 0xFF;
}

/**
 * Encode a uint32 value (little-endian) into buffer at offset
 */
inline void bthome_encode_uint32(uint8_t *buffer, uint8_t &offset, uint32_t value) {
    buffer[offset++] = value & 0xFF;
    buffer[offset++] = (value >> 8) & 0xFF;
    buffer[offset++] = (value >> 16) & 0xFF;
    buffer[offset++] = (value >> 24) & 0xFF;
}

/**
 * Encode a float value (IEEE 754) into buffer at offset
 */
inline void bthome_encode_float(uint8_t *buffer, uint8_t &offset, float value) {
    uint32_t *ptr = (uint32_t *)(&value);
    uint32_t val = *ptr;
    bthome_encode_uint32(buffer, offset, val);
}

/**
 * Calculate scaled int16 from float (0.01 scale)
 * Used for velocity, acceleration, etc.
 */
inline int16_t bthome_float_to_scaled_int16(float value) {
    return (int16_t)(value * 100.0f);
}

#endif // BTHOME_V2_H
