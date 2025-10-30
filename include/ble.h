/*
 * BLE Communication Module for Kalman Filter
 * 
 * Replaces ESP-NOW with BLE advertising using BTHomeV2 protocol
 * Copyright (C) 2006-2025 www.hlballon.com
 * 
 * Version 1.0 250123 Hilmar Lorenz - Initial BLE/BTHomeV2 implementation
 */

#ifndef BLE_H
#define BLE_H

#include <cstdint>
#include <cstddef>

// BLE Communication API

/**
 * Initialize BLE advertising
 * Must be called once in setup()
 */
extern void ble_setup(void);

/**
 * Send telemetry data via BLE advertising
 * Encodes data in BTHomeV2 format
 * 
 * @param pressure Pressure in Pa
 * @param vertical_speed Vertical speed in m/s
 * @param vertical_accel Vertical acceleration in m/s²
 * @param altitude Altitude in meters
 * @param alt_variance Altitude variance σ²
 * @param vario_variance Vario variance σ²
 * @param reading_id Counter for transmitted readings
 */
extern void ble_advertise(
    float pressure,
    float vertical_speed,
    float vertical_accel,
    float altitude,
    float alt_variance,
    float vario_variance,
    uint32_t reading_id
);

/**
 * Update BLE advertisement data
 * Called internally by ble_advertise()
 */
extern void ble_update_advertising(void);

/**
 * Get BLE device MAC address
 * @return Pointer to 6-byte MAC address
 */
extern const uint8_t* ble_get_mac_address(void);

/**
 * Check if BLE is initialized
 * @return true if BLE is ready
 */
extern bool ble_is_initialized(void);

#endif // BLE_H
