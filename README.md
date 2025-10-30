# M5S3_BT_kalman-slave

**M5Stamp S3 BTHome-v2 Kalman Slave based on DPS368**

High-precision balloon telemetry system using Kalman filtering and BLE BTHome protocol for Home Assistant integration.

## Overview

This project implements a balloon telemetry sensor using the M5Stamp S3 and DPS368 pressure sensor. It transmits telemetry data via BLE using the BTHome v2 protocol for seamless Home Assistant integration.

### Key Features
- **Kalman Filter**: Estimation of the balloon dynamics: vertical speed and vertical acceleration 
- **BLE BTHome v2**: Standard protocol for Home Assistant auto-discovery
- **DPS368 Sensor**: High-precision pressure measurements of the DPS368 baro sensor
- **Real-time Telemetry**: Pressure, altitude, vertical speed, and acceleration

## BLE BTHome Data Structure

The system uses the BTHome v2 protocol to transmit sensor data via BLE advertisements. This enables automatic discovery by Home Assistant without manual configuration.

### BTHome Advertisement Format

Each BLE advertisement packet contains:

```
┌─────────────────┬────────────────────────────────────────────┐
│ Advertisement   │ Content                                    │
│ Section         │                                            │
├─────────────────┼────────────────────────────────────────────┤
│ BLE Flags       │ 0x06 (LE General Discoverable + BR/EDR    │
│                 │ not supported)                             │
├─────────────────┼────────────────────────────────────────────┤
│ Service Data    │ BTHome UUID (0xFCD2) + Sensor Data        │
├─────────────────┼────────────────────────────────────────────┤
│ Scan Response   │ Device Name + Service UUID                 │
└─────────────────┴────────────────────────────────────────────┘
```

### Service Data Structure

The BTHome service data follows this format:

```
Byte:  0-1    2      3-N
     ┌────┬─────┬──────────────────────────────────┐
     │UUID│Flag │        Sensor Objects           │
     └────┴─────┴──────────────────────────────────┘
```

- **UUID** (2 bytes): `0xD2FC` (BTHome Service UUID, little-endian)
- **Flag** (1 byte): `0x40` (Not encrypted, not trigger-based)
- **Sensor Objects** (Variable): Object ID + Data pairs

### Transmitted Sensor Objects

**Important**: Object IDs must be transmitted in numerical order per BTHome v2 specification.

| Object ID | Type    | Scale      | Description              | Home Assistant Display |
|-----------|---------|------------|--------------------------|------------------------|
| `0x04`    | uint24  | 0.01 hPa   | Atmospheric Pressure     | Pressure sensor (hPa)  |
| `0x3E`    | uint32  | count      | Reading Counter          | Counter sensor         |
| `0x41`    | uint16  | 0.1 m      | Altitude                 | Distance sensor (m)    |
| `0x44`    | uint16  | 0.01 m/s   | Vertical Speed (abs)     | Speed sensor (m/s)     |
| `0x51`    | uint16  | 0.001 m/s² | Vertical Acceleration    | Acceleration sensor    |

### Data Encoding Example

For a balloon at 1000m altitude, 2.5 m/s climb rate:

```
Hex Data: 04 86 1A 01  3E 2A 00 00 00  41 10 27  44 FA 00  51 50 0C
         │─────────│  │───────────│  │────│  │────│  │────│
         │Pressure │  │ Counter   │  │Alt │  │Spd │  │Acc │
         │101,350Pa│  │    42     │  │1000m│ │2.5 │  │3.2 │
```

**Breakdown**:
- **Pressure**: `04 86 1A 01` = Object 0x04, value 101,350 (1013.50 hPa)
- **Counter**: `3E 2A 00 00 00` = Object 0x3E, value 42
- **Altitude**: `41 10 27` = Object 0x41, value 10,000 (1000.0 m)
- **Speed**: `44 FA 00` = Object 0x44, value 250 (2.50 m/s)
- **Acceleration**: `51 50 0C` = Object 0x51, value 3,200 (3.200 m/s²)

### Home Assistant Integration

1. **Enable BTHome Integration**: Add BTHome via HACS or core integration
2. **Device Discovery**: Devices with name "HLBc-Kalman" auto-appear
3. **Entity Names**: 
   - `sensor.hlbc_kalman_pressure`
   - `sensor.hlbc_kalman_counter`
   - `sensor.hlbc_kalman_distance`
   - `sensor.hlbc_kalman_speed`
   - `sensor.hlbc_kalman_acceleration`

### Technical Details

- **BLE Stack**: NimBLE 2.2.3+ for ESP32-S3
- **Advertisement Interval**: 1000ms (1 second)
- **TX Power**: ESP_PWR_LVL_P9 (medium range/battery balance)
- **Connection Mode**: Non-connectable (advertisement only)
- **Data Encoding**: Little-endian, per BTHome specification
