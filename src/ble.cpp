/*
 * BLE Communication Implementation using BTHomeV2 Protocol
 * 
 * Replaces ESP-NOW with BLE advertising using proper BTHomeV2 format
 * Uses NimBLE 2.2.3+ for efficient BLE stack on ESP32-S3
 * Implements Home Assistant BTHome auto-discovery
 * 
 * Reference: https://github.com/mhaberler/BTHomeV2-ESP32-example
 * 
 * Copyright (C) 2006-2025 www.hlballon.com
 * Version 2.0 250126 Hilmar Lorenz - Full BTHomeV2 Home Assistant integration
 */

#include "ble.h"
#include "bthome_v2.h"
#include <NimBLEDevice.h>
#include <esp_bt_device.h>
#include <cstring>
#include <cmath>

// ============ BLE Advertisement Configuration ============

#define DEVICE_NAME                   "HLBc-Kalman"
#define BLE_TX_POWER                  ESP_PWR_LVL_P9      // Medium power for range vs battery
#define BLE_ADVERTISING_INTERVAL      1000                // 1 second in ms (625µs units)

// BTHomeV2 Constants
#define BTHOME_ADV_MAX_LEN            31                  // Maximum BLE advertisement data
#define BTHOME_SERVICE_DATA_LEN       25                  // Max sensor data (31 - flags/name handling)

// ============ Global State ============

static bool ble_initialized = false;
static NimBLEAdvertising *pAdvertising = nullptr;

// Sensor data buffer for BTHomeV2 (Object IDs must be in numerical order)
static uint8_t sensor_data[BTHOME_SERVICE_DATA_LEN];
static uint8_t sensor_data_len = 0;

// Last transmitted values (for debugging)
static struct {
    float pressure;
    float v_speed;
    float v_accel;
    float alt_var;
    float vario_var;
    uint32_t reading_id;
} last_values = {0};

// ============ Private Functions ============

/**
 * Build BTHomeV2 compliant sensor data payload
 * Object IDs MUST be in numerical order (low to high)
 */
static void build_sensor_payload(
    float pressure,
    float altitude,
    float vertical_speed,
    float vertical_accel,
    float alt_variance,
    float vario_variance,
    uint32_t reading_id
) {
    uint8_t offset = 0;
    
    // ============ Object IDs MUST be in numerical order (0x04 < 0x3E < 0x41 < 0x44 < 0x51) ============
    // This is a REQUIRED BTHomeV2 specification rule
    // Ref: https://bthome.io/format/ - "Object ids have to be applied in numerical order"
    
    // ============ Object: Pressure (0x04 - uint24, 0.01 hPa) ============
    // Standard BTHomeV2 - displays as pressure sensor in hPa
    sensor_data[offset++] = BTHOME_OID_PRESSURE;  // 0x04
    uint32_t pressure_hpa = (uint32_t)(pressure / 100.0f);  // Convert Pa to hPa, then scale by 100
    bthome_encode_uint24(sensor_data, offset, pressure_hpa);
    
    // ============ Object: Counter (0x3E - uint32) ============
    // Standard BTHomeV2 extended counter
    sensor_data[offset++] = BTHOME_OID_COUNT4;  // 0x3E
    bthome_encode_uint32(sensor_data, offset, reading_id);
    
    // ============ Object: Distance/Altitude (0x41 - uint16, 0.1 m) ============
    // Standard BTHomeV2 distance in meters (scale: 0.1 m per unit)
    sensor_data[offset++] = BTHOME_OID_DISTANCE_M;  // 0x41
    uint16_t altitude_scaled = (uint16_t)(altitude * 10.0f);  // Scale to 0.1 m
    bthome_encode_uint16(sensor_data, offset, altitude_scaled);
    
    // ============ Object: Speed (0x44 - uint16, 0.01 m/s) ============
    // Standard BTHomeV2 for vertical speed
    // NOTE: Using absolute value - shows speed magnitude (balloons have +/- speeds)
    sensor_data[offset++] = BTHOME_OID_SPEED;  // 0x44
    uint16_t v_speed_unsigned = (uint16_t)(fabs(vertical_speed) * 100.0f);  // Scale to 0.01 m/s (absolute value)
    bthome_encode_uint16(sensor_data, offset, v_speed_unsigned);
    
    // ============ Object: Acceleration (0x51 - uint16, 0.001 m/s²) ============
    // Standard BTHomeV2 for vertical acceleration  
    // NOTE: Using absolute value - shows acceleration magnitude (balloons have +/- accelerations)
    sensor_data[offset++] = BTHOME_OID_ACCELERATION;  // 0x51
    uint16_t accel_unsigned = (uint16_t)(fabs(vertical_accel) * 1000.0f);  // Scale to 0.001 m/s² (absolute value)
    bthome_encode_uint16(sensor_data, offset, accel_unsigned);
    
    // Store final sensor data length
    sensor_data_len = offset;
    
    // Ensure we don't exceed maximum
    if (sensor_data_len > BTHOME_SERVICE_DATA_LEN) {
        sensor_data_len = BTHOME_SERVICE_DATA_LEN;
        Serial.println("WARNING: Sensor payload truncated");
    }
}

/**
 * Build and transmit BTHomeV2 advertisement packet
 * Format: FLAGS + SERVICE_DATA(UUID + ENCRYPT_FLAG + SENSOR_DATA) + NAME
 */
static void build_and_advertise(void) {
    if (!ble_initialized || !pAdvertising) {
        return;
    }
    
    // Stop advertising to update data
    pAdvertising->stop();
    
    // ============ Build Advertisement Data ============
    NimBLEAdvertisementData advData;
    
    // 1. Set Flags (required by BLE spec)
    advData.setFlags(0x06);  // LE General Discoverable Mode + BR/EDR not supported
    
    // 2. Build and add Service Data for BTHome
    //    BTHome spec: AD Type 0x16 (Service Data) + UUID (little-endian) + encryption flag + data
    std::vector<uint8_t> serviceData;
    serviceData.push_back(BTHOME_UUID_BYTE1); // 0xD2 (little-endian first byte)
    serviceData.push_back(BTHOME_UUID_BYTE2); // 0xFC (little-endian second byte)
    serviceData.push_back(BTHOME_NO_ENCRYPT); // 0x40 (not encrypted, not trigger-based)
    
    // Append sensor data
    for (uint8_t i = 0; i < sensor_data_len; i++) {
        serviceData.push_back(sensor_data[i]);
    }
    
    // Use the setServiceData method for proper AD encoding
    advData.setServiceData(BLEUUID((uint16_t)BTHOME_SERVICE_UUID), 
                          std::string((char*)serviceData.data() + 2, 
                                     serviceData.size() - 2));
    
    pAdvertising->setAdvertisementData(advData);
    
    // ============ Build Scan Response Data (Device Name + Service UUID) ============
    NimBLEAdvertisementData scanData;
    scanData.setCompleteServices(BLEUUID((uint16_t)BTHOME_SERVICE_UUID));
    scanData.setName(DEVICE_NAME);
    pAdvertising->setScanResponseData(scanData);
    
    // Restart advertising
    if (pAdvertising->start()) {
        Serial.printf("BLE TX: P=%.0f Pa, V=%.2f m/s, A=%.2f m/s², ID=%ld\n",
                      last_values.pressure, last_values.v_speed, last_values.v_accel, 
                      last_values.reading_id);
    } else {
        Serial.println("ERROR: Failed to restart BLE advertising");
    }
}


// ============ Public API Implementation ============

/**
 * Initialize BLE advertising with BTHomeV2 format
 * Must be called once in setup()
 */
void ble_setup(void) {
    if (ble_initialized) {
        return;
    }
    
    Serial.println("\n=== BLE Initialization (NimBLE 2.2.3) ===");
    
    // Initialize NimBLE device (empty name - will be in scan response)
    NimBLEDevice::init("");
    
    // Get advertising instance
    pAdvertising = NimBLEDevice::getAdvertising();
    
    // Configure advertising parameters
    // BLE interval is in 0.625ms units, so 1000ms = 1600 units
    pAdvertising->setMinInterval(1600);  // 1000ms
    pAdvertising->setMaxInterval(1600);
    
    // Set to non-connectable mode (we only advertise, don't accept connections)
    pAdvertising->setConnectableMode(BLE_GAP_CONN_MODE_NON);
    
    // Start with empty data
    NimBLEAdvertisementData emptyAdv;
    emptyAdv.setFlags(0x06);
    pAdvertising->setAdvertisementData(emptyAdv);
    
    if (pAdvertising->start()) {
        Serial.println("BLE: NimBLE advertising started");
        ble_initialized = true;
    } else {
        Serial.println("ERROR: Failed to start NimBLE advertising");
        ble_initialized = false;
        return;
    }
    
    // Print device MAC address for reference
    const uint8_t *mac = ble_get_mac_address();
    Serial.printf("BLE Device: %s\n", DEVICE_NAME);
    Serial.printf("BLE MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("BTHome Service UUID: 0x%04X (0xFCD2)\n", BTHOME_SERVICE_UUID);
    Serial.println("Status: Ready for Home Assistant BTHome discovery\n");
}

/**
 * Send telemetry data via BLE advertising using BTHomeV2 format
 * Home Assistant will auto-discover device via BTHome integration
 */
void ble_advertise(
    float pressure,
    float vertical_speed,
    float vertical_accel,
    float altitude,
    float alt_variance,
    float vario_variance,
    uint32_t reading_id
) {
    if (!ble_initialized) {
        return;
    }
    
    // Store values for reference
    last_values.pressure = pressure;
    last_values.v_speed = vertical_speed;
    last_values.v_accel = vertical_accel;
    last_values.alt_var = alt_variance;
    last_values.vario_var = vario_variance;
    last_values.reading_id = reading_id;
    
    // Build BTHomeV2 compliant sensor payload (objects in numerical order)
    build_sensor_payload(
        pressure,
        altitude,
        vertical_speed,
        vertical_accel,
        alt_variance,
        vario_variance,
        reading_id
    );
    
    // Build and transmit the advertisement
    build_and_advertise();
}

/**
 * Update BLE advertisement with new data
 * (Called internally by ble_advertise)
 */
void ble_update_advertising(void) {
    if (!ble_initialized || !pAdvertising) {
        return;
    }
    build_and_advertise();
}

/**
 * Get BLE device MAC address (6 bytes)
 */
const uint8_t* ble_get_mac_address(void) {
    static uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BT);
    return mac;
}

/**
 * Check if BLE is initialized and advertising
 */
bool ble_is_initialized(void) {
    return ble_initialized;
}
