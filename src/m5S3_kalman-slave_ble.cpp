// HLBc Version 2.0 - BLE Edition
// M5Stack M5StampS3
// Modified 211004 by Hilmar & Vincent
// Last modified 220710 by Hilmar
// Last modified 230611 by Hilmar
// Last modified 240929 by Hilmar
// Last modified 250105 by Hilmar
// Last modified 250123 by Hilmar - Migrated to BLE + BTHomeV2
// BLE Advertising Slave with Kalman Filter

#include <M5Unified.h>
#include <Wire.h>
#include "ble.h"
#include "dps368.h"

#include <Preferences.h>

#define I2C_SDA 13 // pin 18
#define I2C_SCL 15 // pin 19

// ============ Configuration ============
#define BLE_UPDATE_INTERVAL_MS    1000   // Send BLE telemetry every 1 second
#define AVERAGING_BUFFER_SIZE     4      // Average over 4 samples (1 second at 250ms cycle)

// ============ Timing ============
unsigned long last_ble_update = 0;

// ============ Global Variables ============
uint32_t readingId = 0;  // Reading counter for BTHomeV2

// ============ Averaging Buffers ============
float pressure_buffer[AVERAGING_BUFFER_SIZE] = {0};
float altitude_buffer[AVERAGING_BUFFER_SIZE] = {0};
float v_baro_buffer[AVERAGING_BUFFER_SIZE] = {0};
float a_baro_buffer[AVERAGING_BUFFER_SIZE] = {0};
uint8_t buffer_index = 0;
bool buffers_full = false;

// ============ Averaging Functions ============
void update_averaging_buffers(void) {
  // Store current values in circular buffers
  pressure_buffer[buffer_index] = pressure;
  altitude_buffer[buffer_index] = alt;
  v_baro_buffer[buffer_index] = v_baro;
  a_baro_buffer[buffer_index] = a_baro;
  
  // Advance buffer index (circular)
  buffer_index = (buffer_index + 1) % AVERAGING_BUFFER_SIZE;
  
  // Mark buffers as full after first complete cycle
  if (buffer_index == 0 && !buffers_full) {
    buffers_full = true;
  }
}

float calculate_average(float* buffer, uint8_t size) {
  float sum = 0.0f;
  uint8_t count = buffers_full ? size : buffer_index;
  
  for (uint8_t i = 0; i < count; i++) {
    sum += buffer[i];
  }
  
  return (count > 0) ? (sum / count) : 0.0f;
}

void init_i2c_gpio(void)
{
  Wire.end();
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin(I2C_SDA, I2C_SCL);
}

void i2cscan(void)
{
  Serial.println("I2C Scan\n");

  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
  delay(5000);
}

void setup(void)
{
  M5.begin();
  Serial.begin(115200);
  
  delay(500); // Allow serial to stabilize
  
  Serial.println("\n\n=== HLBc Kalman Filter - BLE Edition ===\n");

  // Initialize I2C with custom pins
  init_i2c_gpio();

  // Initialize BLE advertising (replaces ESP-NOW)
  ble_setup();

  // Initialize pressure sensor
  init_press_sensor();

  // Initialize Kalman filter
  init_Kalman();
  
  last_ble_update = millis();
  
  Serial.println("Setup complete. Running...\n");
}

void loop(void)
{
  M5.update();
  
  // Update M5 display (if needed)
  // i2cscan();  // Debug: Uncomment to scan I2C bus
  
  // Read pressure sensor and update Kalman filter
  read_press_data();
  
  // Update averaging buffers with current values
  update_averaging_buffers();
  
  // Send BLE telemetry at regular intervals with averaged values
  unsigned long now = millis();
  if ((now - last_ble_update) >= BLE_UPDATE_INTERVAL_MS) {
    
    // Calculate averaged values for smoother transmission
    float avg_pressure = calculate_average(pressure_buffer, AVERAGING_BUFFER_SIZE);
    float avg_altitude = calculate_average(altitude_buffer, AVERAGING_BUFFER_SIZE);
    float avg_v_baro = calculate_average(v_baro_buffer, AVERAGING_BUFFER_SIZE);
    float avg_a_baro = calculate_average(a_baro_buffer, AVERAGING_BUFFER_SIZE);
    
    // Debug: Print values before transmission
    Serial.print("DEBUG BLE TX - P:");
    Serial.print(avg_pressure);
    Serial.print(" Alt:");
    Serial.print(avg_altitude);
    Serial.print(" V:");
    Serial.print(avg_v_baro);
    Serial.print(" A:");
    Serial.print(avg_a_baro);
    Serial.print(" ID:");
    Serial.println(readingId);
    
    // Transmit averaged Kalman filter estimates via BLE using BTHomeV2 format
    ble_advertise(
        avg_pressure,    // Averaged pressure in Pa
        avg_v_baro,      // Averaged vertical speed in m/s
        avg_a_baro,      // Averaged vertical acceleration in m/s²
        avg_altitude,    // Averaged altitude in meters
        altVar,          // Altitude variance
        varioVar,        // Vario variance
        readingId++      // Reading counter (incremented)
    );
    
    last_ble_update = now;
  }
}
