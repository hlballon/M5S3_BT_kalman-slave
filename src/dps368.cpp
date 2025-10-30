// Adafruit DPS368 Driver Variante

#include <M5Unified.h>
#include <hKalF.hpp>
#include <Dps3xx.h>
#include <Wire.h>
#include "dps368.h"
#include "math.h"

Dps3xx DPS368;
HKF hkf;

#define READ_DELAY_TIME 250
#define VARIO_CHECK_TIME 250

 /*
   * pressure measure rate (value from 0 to 7)
   * 2^prs_mr pressure measurement results per second
   */
  int16_t prs_mr = 2;

  /*
   * pressure oversampling rate (value from 0 to 7)
   * 2^prs_osr internal pressure measurements per result
   * A higher value increases precision
   */
  int16_t prs_osr = 7;

  uint8_t pressureCount = 1;
  float pressureF[5];

  uint8_t temperatureCount = 1;
  float temperatureF[5];


unsigned long dps_delay = 0;
int vario_time;

float v_baro = 0.0;
float a_baro = 0.0;

float altVar = 0.01;
float varioVar = 0.01;


unsigned long tKalman = millis();

float finfo_FBaro[5];
float finfo_vSpeed[5];
float finfo_a_vSpeed[5];
float pressure;
float alt;
float alt_feet;
float old_vario_alt;
float vario = 0.0;
float vario_temp;
float start_alt;
float QNH = 1013.25;

int qnh_set_delay;

int init_press_sensor(void)
{
  dps_delay = 0;

// Initialize the DPS368 sensor

    DPS368.begin(Wire);

/*
   * startMeasureBothCont enables background mode
   * temperature and pressure are measured automatically
   * High precision and high measure rates at the same time are not available.
   * Consult Datasheet (or trial and error) for more information
   */
  // int16_t ret = DPS368.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);    
  int16_t ret = DPS368.startMeasurePressureCont(prs_mr, prs_osr);
  if (ret != DPS__SUCCEEDED) {
    Serial.println("Fehler beim Starten der Messung!");
    while (1); // Endlosschleife bei Fehler
  }

  Serial.println("DPS368 sensor initialized!");

return 0;
}

void init_Kalman(void) {
      
      for (int i = 0; i < 5; ++i) {
          finfo_FBaro[i] = alt;
          finfo_vSpeed[i] = 0.0;
          finfo_a_vSpeed[i] = 0.0;
      }
  }
  
float Get_QNH_Altitude(float pressure, float calc_qnh)
{ // return the altitude in meters that corresponds to the given pressure in hundredths of a millibar
  float pressure_fact = pressure / calc_qnh;
  float temp_fact = (273.15 + 15) / 0.0065;
  float air_const = 1 / 5.255;
  return ((1 - pow(pressure_fact, air_const)) * temp_fact);
}


void calc_alt_data(float pressure)
{
  alt = Get_QNH_Altitude(pressure, QNH);
  alt_feet = alt * FEET_FACT / 1000;
}

void calc_vario(void)
{
  if (millis() < 5000)
  {
    old_vario_alt = alt;
    return;
  }

  if (qnh_set_delay > millis())
  {
    old_vario_alt = alt;
    return;
  }

  if (vario_time > millis())
    return;

  vario_time = millis() + VARIO_CHECK_TIME; // VARIO_CHECK_TIME = 250ms
  
  // Calculate raw vertical velocity from altitude change
  vario_temp = alt - old_vario_alt;  // altitude change in meters

  // CRITICAL FIX (v2.4): Convert from meters/250ms to m/s
  // Measurement interval is 250ms = 0.25 seconds
  // To convert: vario_temp [m/0.25s] / 0.25 = vario_temp [m/s]
  // This ensures vario is in same units as v_baro for proper comparison
  vario_temp = vario_temp / 0.25f;  // NOW in m/s! ✅
  
  // Exponential smoothing
  vario = (vario - (vario - vario_temp) / MW_FACT_VARIO);

  old_vario_alt = alt;
}


void read_press_data(void)
{
  if (dps_delay > millis())
    return;
//    Serial.println("3");
  dps_delay = millis() + READ_DELAY_TIME;

/*
   * This function writes the results of continuous measurements to the arrays given as parameters
   * The parameters temperatureCount and pressureCount should hold the sizes of the arrays temperature and pressure when the function is called
   * After the end of the function, temperatureCount and pressureCount hold the numbers of values written to the arrays
   * Note: The Dps3xx cannot save more than 32 results. When its result buffer is full, it won't save any new measurement results
   */
  DPS368.getContResults(temperatureF, temperatureCount, pressureF, pressureCount);

  pressure = pressureF[0] / 100.0f;

  calc_alt_data(pressure);
  calc_vario();
  calc_acc();

  // ESP-NOW removed - BLE version handles transmission in main loop
 } 

 void calc_acc() {
  if (millis() < 10000)
  {
    return;
  }

    float dtACC = (float)(millis() - tKalman) / 1000.0f;

    tKalman = millis();

    altVar  = infoStatistics(finfo_FBaro);

    hkf.accKalman(dtACC, alt, altVar);


        v_baro = hkf.VerticalSpeed();
        a_baro = hkf.VerticalAcceleration();


  for (int i = 0; i < 4; ++i) {
        finfo_vSpeed[i] = finfo_vSpeed[i + 1];
        finfo_a_vSpeed[i] = finfo_a_vSpeed[i + 1];
    }

    finfo_vSpeed[4] = v_baro;
    finfo_a_vSpeed[4] = a_baro;
  
}

//***************** Kalman Statistics Test *****************
// Calculate the variance of the data over a longer history
// This provides more stable measurement noise estimates
// NOTE: Variance represents measurement noise in the altitude measurement
// Higher variance = less confidence in measurement = lower Kalman gain
// Lower variance = more confidence in measurement = higher Kalman gain
float infoStatistics(float f_Elv[5]) {
    float sum = 0.0, variance = 0.0, mean = 0.0;
    int i;
    
    // Calculate mean of 5 altitude samples
    for (i = 0; i < 5; ++i) {
        sum += f_Elv[i];
    }
    mean = sum / 5.0f;
    
    // Calculate variance (sum of squared deviations from mean)
    for (i = 0; i < 5; ++i) {
        variance += pow(f_Elv[i] - mean, 2);
    }
    
    // Use Bessel's correction: divide by (N-1) instead of N
    // This provides unbiased variance estimate for sample variance
    variance = variance / 4.0f;
    const float var_norm = variance;
    
    // Clamp variance to reasonable bounds to prevent filter instability
    // These bounds should match the thresholds used in updateMeasurementNoiseCovarianceMatrix()
    // Lower bound: Minimum measurement noise to prevent filter divergence
    // Upper bound: Avoid trusting overly noisy measurements
    const float VAR_MIN = 0.001f;  // Minimum 0.001 m² variance
    const float VAR_MAX = 0.1f;    // Maximum 0.1 m² variance
    
    if (var_norm < VAR_MIN) return VAR_MIN;
    if (var_norm > VAR_MAX) return VAR_MAX;

    return var_norm;
}

