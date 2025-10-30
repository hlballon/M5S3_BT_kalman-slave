/**
 * @file dps368.h
 * @brief DPS368 Barometric Pressure Sensor Interface - Public API
 * 
 * Defines configuration constants, data structures, and function declarations
 * for the DPS368 sensor interface module.
 * 
 * **Module Responsibility:**
 * - I2C communication with DPS368 barometer
 * - Pressure to altitude conversion
 * - Altitude to vertical velocity estimation
 * - Altitude variance calculation
 * - Kalman filter data preparation and integration
 * - Telemetry serial output
 * 
 * @author Hilmar Lorenz <hilmar@hlballon.com>
 * @version 3.0 - Production release
 * @date October 2025
 * 
 * @copyright GNU General Public License v2.0 or later
 */

#ifndef DPS368_H
#define DPS368_H

// ============================================================================
// UNIT CONVERSION CONSTANTS
// ============================================================================

/// Conversion factor: meters to feet (1 m = 3.28084 ft)
/// Used for alternate altitude display
#define METERS_TO_FEET 3281

/// Pressure smoothing/moving window factor (deprecated, marked for removal)
/// Current implementation does not use this constant
#define PRESSURE_MW_FACTOR 64

/// Vario (vertical speed) smoothing factor
/// Higher value = more smoothing
/// 1 = no smoothing (raw difference), 2 = moderate smoothing
#define VARIO_SMOOTHING_FACTOR 1

// ============================================================================
// BACKWARD COMPATIBILITY ALIASES (v2.5 code compatibility)
// ============================================================================

/// Alias for v2.5 code compatibility: FEET_FACT = METERS_TO_FEET
#define FEET_FACT METERS_TO_FEET

/// Alias for v2.5 code compatibility: MW_FACT_VARIO = VARIO_SMOOTHING_FACTOR
#define MW_FACT_VARIO VARIO_SMOOTHING_FACTOR

/// Alias for v2.5 code compatibility: PRESSURE_MW_FACTOR
#define MW_FACT_PRESS PRESSURE_MW_FACTOR

// ============================================================================
// TIMING CONSTANTS (milliseconds)
// ============================================================================

/// Sensor measurement interval
/// DPS368 configured for ~250ms between readings
#define SENSOR_UPDATE_INTERVAL_MS 250

/// Sensor warm-up/settling time after power-on
/// Filter output unreliable before this time
#define SENSOR_WARMUP_TIME_MS 5000

/// Delay before Kalman filter estimates are valid
/// Allows filter to initialize and settle
#define KALMAN_READY_TIME_MS 10000

// ============================================================================
// DATA ACQUISITION CONSTANTS
// ============================================================================

/// Number of altitude samples maintained for variance calculation
/// Larger = more stable but slower to respond to changes
/// Current: 5 samples × 250ms = 1.25 seconds history
#define ALTITUDE_HISTORY_SIZE 5

/// Number of vertical velocity samples maintained
/// Used for velocity-based analysis and diagnostics
#define VELOCITY_HISTORY_SIZE 5

// ============================================================================
// STATISTICAL BOUNDS
// ============================================================================

/// Minimum altitude variance (m²) for Kalman filter
/// Prevents filter divergence in extremely clean conditions
#define ALT_VARIANCE_MIN 0.001f

/// Maximum altitude variance (m²) for Kalman filter
/// Prevents over-correction in extremely noisy conditions
#define ALT_VARIANCE_MAX 0.1f

// ============================================================================
// EXTERNAL VARIABLES - MEASUREMENT BUFFERS
// ============================================================================

/// Altitude history buffer (meters) - used for variance calculation
extern float finfo_FBaro[ALTITUDE_HISTORY_SIZE];

/// Vertical speed estimates from Kalman filter
extern float finfo_vSpeed[VELOCITY_HISTORY_SIZE];

/// Vertical acceleration estimates from Kalman filter
extern float finfo_a_vSpeed[VELOCITY_HISTORY_SIZE];

// ============================================================================
// EXTERNAL VARIABLES - CURRENT SENSOR MEASUREMENTS
// ============================================================================

/// Current atmospheric pressure in hectopascals (hPa)
extern float pressure;

/// Current altitude in meters above mean sea level
extern float alt;

/// Current altitude in feet (for pilots familiar with feet)
extern float alt_feet;

/// Pressure reference for QNH adjustments (standard: 1013.25 hPa)
extern float QNH;

/// Temperature in degrees Celsius
extern float temp;

// ============================================================================
// EXTERNAL VARIABLES - KALMAN FILTER ESTIMATES
// ============================================================================

/// Vertical speed (velocity) from Kalman filter in m/s
/// Positive = climbing, negative = descending
/// Also called v_baro in some contexts
extern float v_baro;

/// Vertical acceleration from Kalman filter in m/s²
/// Positive = accelerating upward
/// Also called a_baro in some contexts
extern float a_baro;

/// Altitude variance used by Kalman filter (m²)
/// Represents measurement noise in altitude
/// Range: [0.001, 0.1] m²
extern float altVar;

/// Vario variance (currently maintained but not actively used)
extern float varioVar;

// ============================================================================
// EXTERNAL VARIABLES - VERTICAL SPEED ESTIMATION
// ============================================================================

/// Manually calculated vertical speed from altitude changes (m/s)
/// Calculated from 5-sample altitude history
/// Provides independent check of Kalman filter v_baro
extern float vario;

/// Starting altitude at flight commencement
extern float start_alt;

/// Old vario altitude value (for internal calculations)
extern float old_vario_alt;

/// Average pressure over time window
extern float avg_pressure;

/// Estimated pressure value
extern float estimated_pressure;

// ============================================================================
// EXTERNAL VARIABLES - FLIGHT STATE
// ============================================================================

/// Time since Kalman filter update (milliseconds)
extern unsigned long tKalman;

/// Flight time in seconds
extern int flighttime;

/// Time off ground (takeoff time) in seconds
extern int to_time;

/// Time landed in seconds
extern int landing_time;

/// Flight state flag (0 = landed, 1 = airborne)
extern int startflag;

/// Landing counter
extern int landcounter;

/// Vario tone flag (0 = silent, 1 = beeping)
extern int vario_tone_flag;

/// QNH adjustment delay timer
extern int qnh_set_delay;

// ============================================================================
// EXTERNAL VARIABLES - WIND/SPEED ANALYSIS
// ============================================================================

/// Wind speed array for wind analysis
extern int wind_array[21];

/// Speed array for performance analysis
extern int speed_array[21];

// ============================================================================
// EXTERNAL VARIABLES - VARIANCE DIAGNOSTICS
// ============================================================================

/// Double-precision altitude variance (for analysis)
extern double daltVar;

/// Double-precision vario variance (for analysis)
extern double dvarioVar;

/// Kalman filter to vario conversion factor
extern float kf2vario;

/// Variance calculation parameter
extern float varCalc;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

/**
 * @brief Initialize DPS368 barometric pressure sensor
 * 
 * Configures the DPS368 sensor via I2C for continuous pressure measurement.
 * Sets up measurement rate and oversampling parameters.
 * 
 * @return 0 on success, error code on failure
 * 
 * @note Must be called after Wire.begin() for I2C initialization
 * @note This is a blocking operation; wait for completion before calling other functions
 */
extern int init_press_sensor(void);

/**
 * @brief Initialize Kalman filter state and history buffers
 * 
 * Prepares the Kalman filter for operation by initializing
 * history buffers and state variables.
 */
extern void init_Kalman(void);

/**
 * @brief Read pressure data from DPS368 and process
 * 
 * Called continuously in main loop. Acquires new pressure measurement
 * only when sensor update interval (250ms) has elapsed.
 * 
 * This is the main entry point that orchestrates:
 * - Sensor data acquisition
 * - Altitude calculation
 * - Velocity estimation
 * - Kalman filter update
 * - Telemetry transmission
 * 
 * Non-blocking; returns immediately if update interval not yet elapsed.
 */
extern void read_press_data(void);

/**
 * @brief Calculate altitude from pressure
 * 
 * Converts barometric pressure to altitude using standard atmosphere model.
 * 
 * @param pressure Pressure in hPa (hectopascals)
 */
extern void calc_alt_data(float pressure);

/**
 * @brief Estimate vertical speed from altitude history
 * 
 * Calculates manual estimate of vertical velocity based on altitude changes
 * over the last 250ms interval. Applies exponential smoothing.
 * 
 * **Result stored in global variable `vario` (m/s)**
 * 
 * This provides an independent check of the Kalman filter v_baro estimate.
 * Differences between vario and v_baro may indicate:
 * - Filter settling during initialization
 * - Rapid acceleration changes
 * - Sensor noise conditions
 */
extern void calc_vario(void);

/**
 * @brief Update Kalman filter with new measurement
 * 
 * Orchestrates Kalman filter update and result extraction.
 * Called after new altitude measurement is available.
 * 
 * This function:
 * - Calculates altitude variance from history
 * - Calls Kalman filter prediction/update
 * - Extracts v_baro and a_baro results
 * - Prepares data for telemetry transmission
 */
extern void calc_acc(void);

/**
 * @brief Calculate variance of altitude measurements
 * 
 * Computes statistical variance of the 5-sample altitude history.
 * Uses Bessel's correction for unbiased sample variance.
 * 
 * **Result:** Bounded to [0.001, 0.1] m² for filter stability
 * 
 * **Interpretation:**
 * - High variance: noisy measurements, filter trusts predictions more
 * - Low variance: clean measurements, filter trusts measurements more
 * 
 * @param f_Elv Array of 5 altitude samples in meters
 * 
 * @return Variance in m² (bounded to [ALT_VARIANCE_MIN, ALT_VARIANCE_MAX])
 */
extern float infoStatistics(float f_Elv[ALTITUDE_HISTORY_SIZE]);

/**
 * @brief Get QNH-adjusted altitude
 * 
 * Calculates altitude using barometric formula with QNH reference.
 * 
 * @param pressure Pressure in hPa
 * @param calc_qnh QNH reference pressure in hPa
 * 
 * @return Altitude in meters above mean sea level
 * 
 * @note Assumes temperature of 15°C (0.0065 K/m lapse rate)
 * @note QNH typically obtained from aviation weather
 */
extern float Get_QNH_Altitude(float pressure, float calc_qnh);

/**
 * @brief Time calculation function
 * 
 * Updates internal timing variables for flight state tracking.
 */
extern void time_calc(void);

// ============================================================================
// END OF HEADER FILE
// ============================================================================

#endif  // DPS368_H
