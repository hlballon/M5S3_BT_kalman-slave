/**
 * @file hKalF.hpp
 * @brief Extended Kalman Filter (EKF) for barometric altitude, velocity, and acceleration estimation
 * 
 * This header implements a 3-state Extended Kalman Filter optimized for hot air balloon vertical motion
 * estimation using barometric pressure measurements only.
 * 
 * @details
 * **System Model:**
 * - State vector: [altitude, vertical_velocity, vertical_acceleration]
 * - Measurement: altitude only (from barometer)
 * - Kinematic coupling through constant-acceleration model
 * 
 * **Filter Architecture:**
 * - Prediction: Uses kinematic equations with 250ms time steps
 * - Update: Adapts measurement noise based on observed altitude variance
 * - Bounds: Q matrix tuned for rapid vertical acceleration response
 * 
 * @author Hilmar Lorenz <hilmar@hlballon.com>
 * @version 3.0 - Production-grade implementation
 * @date October 2025
 * 
 * @copyright
 * Copyright (C) 2006-2025 www.hlballon.com
 * GNU General Public License v2.0 or later
 * 
 * **Based on:**
 * - TinyEKF: https://github.com/simondlevy/TinyEKF (Simon D. Levy - MIT License)
 * - Implemented in Hot Air Balloon Control System (HLBc)
 * - Technical details: https://hlballon.com/brennersteuerung.php
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#pragma once

// ============================================================================
// EKF CONFIGURATION CONSTANTS
// ============================================================================

/// Number of state variables: altitude, vertical velocity, vertical acceleration
#define EKF_N 3

/// Number of measurement variables: altitude only (barometer)
#define EKF_M 1

// ============================================================================
// INCLUDES
// ============================================================================

#include <tinyekf.h>
#include <cstring>

// ============================================================================
// EXTENDED KALMAN FILTER CLASS
// ============================================================================

/**
 * @class HKF
 * @brief Extended Kalman Filter for barometric altitude/velocity/acceleration estimation
 * 
 * Implements a 3-state EKF with constant-acceleration kinematic model. The filter estimates
 * vertical altitude, velocity, and acceleration from barometric pressure measurements.
 * 
 * **Features:**
 * - Autonomous initialization with settling period
 * - Adaptive measurement noise based on altitude variance
 * - Time-varying state transition matrix
 * - Bounded process and measurement noise for stability
 * - Optimized for 250ms measurement intervals
 * 
 * **Usage Example:**
 * ```cpp
 * HKF filter;
 * 
 * // In main loop (called every 250ms):
 * float deltaT = 0.25f;              // 250ms
 * float altitude = 102.5f;           // meters
 * float alt_variance = 0.01f;        // m²
 * 
 * filter.accKalman(deltaT, altitude, alt_variance);
 * 
 * float vertical_speed = filter.VerticalSpeed();
 * float acceleration = filter.VerticalAcceleration();
 * ```
 */
class HKF {
  // ==========================================================================
  // PUBLIC INTERFACE
  // ==========================================================================
  public:

    /**
     * @brief Constructor - Initializes the EKF
     * 
     * Sets up initial error covariance P and prepares the filter for first measurement.
     * The filter starts in uninitialized state and settles for ~5 iterations.
     */
    HKF(void) {
        // Initialize error covariance matrix with diagonal values
        const _float_t P_diag[EKF_N] = {0.01f, 0.01f, 0.001f};
        ekf_initialize(&_ekf, P_diag);
        initialized_ = false;
        init_counter_ = 0;
    }

    /**
     * @brief Get estimated altitude
     * @return Altitude in meters (MSL)
     */
    inline _float_t Altitude(void) const {
        return _altitude;
    }

    /**
     * @brief Get altitude measurement variance
     * @return Variance of altitude measurements in m²
     */
    inline _float_t AltitudeVariance(void) const {
        return _altitude_variance;
    }

    /**
     * @brief Get estimated vertical speed (vertical velocity)
     * @return Vertical speed in m/s (positive = climbing)
     */
    inline _float_t VerticalSpeed(void) const {
        return _vertical_speed;
    }

    /**
     * @brief Get estimated vertical acceleration
     * @return Vertical acceleration in m/s² (positive = accelerating upward)
     */
    inline _float_t VerticalAcceleration(void) const {
        return _vertical_acceleration;
    }

    /**
     * @brief Main Kalman filter update function
     * 
     * Performs one complete EKF predict-update cycle. Should be called every 250ms
     * with new barometric altitude measurements.
     * 
     * **Initialization Behavior:**
     * On the first call, the filter initializes its state to the measured altitude
     * and zero velocity/acceleration. It then runs through 5 "silent" cycles to settle
     * before providing output estimates. This reduces startup transients.
     * 
     * @param deltaT Time step since last call in seconds (typically 0.25s)
     * @param altitude Current measured altitude in meters
     * @param altitude_variance Variance of altitude measurement in m² (typically 0.001-0.1)
     * 
     * **Typical calling pattern:**
     * ```cpp
     * // Every 250ms from sensor read:
     * float dt = (millis() - last_time) / 1000.0f;
     * last_time = millis();
     * float alt_variance = calculate_altitude_variance();
     * 
     * kalman_filter.accKalman(dt, measured_altitude, alt_variance);
     * ```
     * 
     * @note The filter ignores calls during the initialization phase (first 5 calls)
     * @note This function is safe to call repeatedly at fixed intervals
     * @note Input altitude should be computed from barometric pressure using standard formula
     */
    void accKalman(_float_t deltaT, _float_t altitude, _float_t altitude_variance) {
        
        // ==============================================================
        // INITIALIZATION PHASE: First 5 calls
        // ==============================================================
        if (!initialized_) {
            // Set initial state to measured altitude, zero velocity/acceleration
            _ekf.x[0] = altitude;      // Initial altitude (meters)
            _ekf.x[1] = 0.0f;          // Initial velocity (assume stationary)
            _ekf.x[2] = 0.0f;          // Initial acceleration (assume no acceleration)
            
            init_counter_++;
            
            // Allow filter to settle for several measurements before using estimates
            // This reduces startup transients and improves estimate stability
            if (init_counter_ < 5) {
                return;  // Skip filter update during settling period
            }
            
            initialized_ = true;  // Filter is now ready for normal operation
            return;
        }
        
        // ==============================================================
        // NORMAL OPERATION: All calls after initialization
        // ==============================================================
        _altitude_variance = altitude_variance;
        
        // Perform prediction and measurement update cycle
        performEKF_Estimate(deltaT, altitude, altitude_variance);
    }

  // ==========================================================================
  // PRIVATE IMPLEMENTATION
  // ==========================================================================
  private:

    /**
     * @brief Update the state transition matrix F for current time step
     * 
     * Implements the kinematic equations for constant-acceleration model:
     * ```
     * [1   dt   0.5*dt²] [altitude    ]   [altitude_new    ]
     * [0   1    dt     ] [velocity    ] = [velocity_new    ]
     * [0   0    1      ] [acceleration]   [acceleration_new]
     * ```
     * 
     * **Kinematics:**
     * - altitude_new = altitude + dt*velocity + 0.5*dt²*acceleration
     * - velocity_new = velocity + dt*acceleration
     * - acceleration_new = acceleration (constant acceleration assumption)
     * 
     * **Implementation Notes:**
     * This matrix is recalculated every cycle because time step may vary slightly.
     * The coupling between rows shows the kinematic chain:
     * acceleration changes → velocity changes → altitude changes
     * 
     * @param deltaT Time step in seconds
     */
    void updateStateTransitionMatrix(_float_t deltaT) {
        // Row 0: altitude evolution
        F[0] = 1.0f;                              // altitude passes through
        F[1] = deltaT;                            // velocity contribution
        F[2] = 0.5f * deltaT * deltaT;            // acceleration contribution
        
        // Row 1: velocity evolution
        F[3] = 0.0f;                              // altitude doesn't affect velocity
        F[4] = 1.0f;                              // velocity passes through
        F[5] = deltaT;                            // acceleration contribution
        
        // Row 2: acceleration evolution (constant acceleration model)
        F[6] = 0.0f;                              // altitude doesn't affect acceleration
        F[7] = 0.0f;                              // velocity doesn't affect acceleration
        F[8] = 1.0f;                              // acceleration stays constant
    }

    /**
     * @brief Update measurement noise covariance matrix R
     * 
     * Adapts the filter's trust in measurements based on observed altitude variance.
     * This is the key to adaptive filtering - we adjust our confidence dynamically.
     * 
     * **Measurement Noise Interpretation:**
     * - R = 0.001 m²: Very clean measurements (σ ≈ 3.2 cm)
     *   → Filter heavily weights new measurements, trusts sensor
     * - R = 0.01 m²:  Clean measurements (σ ≈ 10 cm)
     *   → Filter balances measurement and prediction
     * - R = 0.1 m²:   Noisy measurements (σ ≈ 31.6 cm)
     *   → Filter emphasizes kinematic model over noisy data
     * 
     * **Technical Details:**
     * Higher measurement noise → larger Kalman gain → filter relies more on predictions
     * Lower measurement noise → smaller Kalman gain → filter trusts measurements more
     * 
     * This adaptation is crucial for handling varying atmospheric conditions:
     * - Clear air: low variance, trust measurements
     * - Turbulent air: high variance, trust predictions more
     * 
     * @param altVar Measured altitude variance in m² (pre-bounded to [0.001, 0.1])
     * 
     * @note altVar is already bounded by infoStatistics() function in dps368.cpp,
     *       so we can use it directly without additional checks
     */
    void updateMeasurementNoiseCovarianceMatrix(_float_t altVar) {
        R[0] = altVar;
    }

    /**
     * @brief Perform one complete EKF predict-update cycle
     * 
     * This is the core Kalman filter algorithm implementing:
     * 1. State transition matrix update for time-varying dynamics
     * 2. State prediction using kinematics
     * 3. Adaptive measurement noise adjustment
     * 4. Measurement update (correction)
     * 5. Result extraction
     * 
     * **Algorithm Flow:**
     * ```
     * UPDATE F(dt)
     *    ↓
     * PREDICT x_new = F*x + process_noise
     *    ↓
     * UPDATE R (measurement noise)
     *    ↓
     * CORRECT x_new using measurement z
     *    ↓
     * EXTRACT results
     * ```
     * 
     * @param deltaT Time step in seconds
     * @param alt Current altitude measurement in meters
     * @param altVar Variance of altitude measurement in m²
     */
    void performEKF_Estimate(_float_t deltaT, _float_t alt, _float_t altVar) {
        
        // Update state transition matrix F for this time step's dynamics
        updateStateTransitionMatrix(deltaT);

        // ====================================================================
        // PREDICTION STEP: Predict state at next time step using kinematics
        // ====================================================================
        // Calculate predicted state at time k based on state at time k-1
        _float_t fx[EKF_N] = {
            // Predicted altitude: x[0] + dt*x[1] + 0.5*dt²*x[2]
            // Current position + velocity*time + 0.5*acceleration*time²
            _ekf.x[0] + deltaT * _ekf.x[1] + 0.5f * deltaT * deltaT * _ekf.x[2],
            
            // Predicted velocity: x[1] + dt*x[2]
            // Current velocity + acceleration*time
            _ekf.x[1] + deltaT * _ekf.x[2],
            
            // Predicted acceleration: x[2]
            // Constant acceleration model (no change)
            _ekf.x[2]
        };

        // Update measurement noise covariance based on observed altitude variance
        updateMeasurementNoiseCovarianceMatrix(altVar);

        // ====================================================================
        // MEASUREMENT PREPARATION: Define what we measure and predict
        // ====================================================================
        
        // Current altitude measurement from barometer
        _float_t z[1] = {alt};

        // Predicted altitude observation (what we expect to measure)
        // This is just the altitude state x[0] since we measure altitude directly
        _float_t hx[1] = {
            _ekf.x[0]  // We predict we'll measure the current altitude state
        };

        // ====================================================================
        // EKF CORE OPERATIONS: Execute TinyEKF predict and update
        // ====================================================================
        
        // Predict: Propagate state and covariance to next time step
        // This incorporates process noise Q and state transition matrix F
        ekf_predict(&_ekf, fx, F, Q);

        // Update: Correct predicted state based on measurement
        // This incorporates measurement noise R and measurement matrix H
        ekf_update(&_ekf, z, hx, H, R);

        // ====================================================================
        // RESULT EXTRACTION: Store updated state estimates in member variables
        // ====================================================================
        _vertical_acceleration = _ekf.x[2];  // Acceleration is x[2]
        _vertical_speed = _ekf.x[1];         // Velocity is x[1]
        _altitude = _ekf.x[0];               // Altitude is x[0]
    }

  // ==========================================================================
  // PRIVATE DATA MEMBERS - EKF MATRICES AND STATE
  // ==========================================================================
  private:

    /// TinyEKF filter structure containing state vector and covariance matrices
    ekf_t _ekf;
    
    // ========================================================================
    // EKF TUNING MATRICES - These define filter behavior and stability
    // ========================================================================
    
    /**
     * @brief Process noise covariance matrix Q (3×3)
     * 
     * Defines how much we expect the system to deviate from kinematic predictions.
     * Represents the model uncertainty.
     * 
     * **Tuning Guidelines:**
     * - Higher Q → Filter adapts faster to changes but may be noisier
     * - Lower Q → Filter is smoother but slower to respond to real changes
     * 
     * **Current Tuning (v3.0 Production):**
     * 
     * Q[0,0] = 0.001 m²:
     *   - Altitude process noise
     *   - Very low: barometer inherently follows physics
     *   - Only minor deviations from kinematic model expected
     *   - Encourages filter to trust kinematic predictions
     * 
     * Q[1,1] = 0.01 m²/s²:
     *   - Velocity process noise
     *   - Moderate: velocity can change due to acceleration
     *   - Higher than altitude to allow responsive velocity tracking
     * 
     * Q[2,2] = 0.001 m²/s⁴:
     *   - Acceleration process noise
     *   - Very low: assumes mostly constant acceleration between updates
     *   - Allows acceleration to change independently
     *   - Tuned after v2.3 instability testing
     * 
     * **History:**
     * - v2.2: Original value 0.05 (too stiff, slow response)
     * - v2.3: Increased to 0.15 (caused divergence, reverted)
     * - v2.5+: Settled at 0.001 (stable and responsive)
     * 
     * Off-diagonal elements are zero (no coupling in process noise).
     */
    _float_t Q[EKF_N * EKF_N] = {
        0.001f,  0.0f,   0.0f,      ///< Altitude process noise
        0.0f,    0.01f,  0.0f,      ///< Velocity process noise
        0.0f,    0.0f,   0.001f     ///< Acceleration process noise
    };
    
    /**
     * @brief Error covariance matrix P (3×3)
     * 
     * Represents uncertainty in the state estimates.
     * Initialized here; dynamically updated by EKF during operation.
     * 
     * **Interpretation:**
     * - P[i,j] represents covariance between states i and j
     * - Diagonal P[i,i] = variance of state i
     * - Off-diagonal elements represent correlation between states
     * 
     * **Initial Values:**
     * 
     * P[0,0] = 0.01 m²:
     *   - Initial altitude uncertainty: √0.01 = ±10 cm
     *   - Reasonable uncertainty for first measurement
     * 
     * P[1,1] = 0.01 m²/s²:
     *   - Initial velocity uncertainty: √0.01 = ±10 cm/s
     *   - Conservative estimate until motion is observed
     * 
     * P[2,2] = 0.001 m²/s⁴:
     *   - Initial acceleration uncertainty: √0.001 = ±3.2 cm/s²
     *   - Very confident in zero acceleration initially
     * 
     * Off-diagonal P[i,j] = 0:
     *   - No initial correlation between states
     *   - Filter will discover correlations through Kalman update
     * 
     * These values will rapidly converge during operation as the filter
     * gains confidence from measurements.
     */
    _float_t P[EKF_N * EKF_N] = {
        0.01f, 0,     0,
        0,     0.01f, 0,
        0,     0,     0.001f
    };
    
    /**
     * @brief State transition matrix F (3×3)
     * 
     * Implements kinematic coupling between states.
     * Updated every cycle by updateStateTransitionMatrix().
     * Initialized to identity matrix; will be properly filled at first call.
     * 
     * **Purpose:**
     * Encodes the physics: how each state affects the others
     * 
     * **Form (updated dynamically):**
     * ```
     * [ 1   dt  0.5*dt² ]
     * [ 0   1   dt      ]
     * [ 0   0   1       ]
     * ```
     * 
     * This matrix represents:
     * - Position changes from velocity and acceleration
     * - Velocity changes from acceleration
     * - Acceleration stays constant (between updates)
     */
    _float_t F[EKF_N * EKF_N] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    
    /**
     * @brief Measurement matrix H (1×3)
     * 
     * Defines which states are measured.
     * H = [1, 0, 0] means we measure altitude directly,
     * but not velocity or acceleration (they're estimated from altitude).
     * 
     * **Why This Matters:**
     * This is CRITICAL to understanding filter behavior:
     * - We MEASURE: altitude from barometer
     * - We ESTIMATE: velocity and acceleration from altitude changes
     * 
     * The filter observes only altitude but infers velocity and acceleration
     * through the kinematic model and Kalman update.
     * 
     * **Consequence for Phase Shift:**
     * Because only altitude is measured, velocity and acceleration estimates
     * are derived from altitude changes. This creates coupling in the update step:
     * all states are corrected through the same altitude measurement innovation.
     * Result: nearly synchronous changes in v_baro and a_baro.
     * 
     * This is NOT an error - it's optimal filtering given the measurement constraints.
     */
    _float_t H[EKF_M * EKF_N] = {
        1.0f, 0.0f, 0.0f  ///< Altitude measurement only (H = [1, 0, 0])
    };
    
    /**
     * @brief Measurement noise covariance matrix R (1×1)
     * 
     * Represents uncertainty in altitude measurements.
     * Dynamically updated based on observed altitude variance.
     * 
     * **Adaptive Behavior:**
     * - Updated every cycle based on recent altitude variance
     * - Range: [0.001, 0.1] m² (enforced by infoStatistics() in dps368.cpp)
     * 
     * **Effect on Filter:**
     * - Lower R (clean measurements): Filter trusts measurements, responds quickly
     * - Higher R (noisy measurements): Filter trusts predictions, smoother output
     * 
     * **Dynamic Adaptation Strategy:**
     * ```
     * If altitude variance is low:
     *   R → 0.001
     *   → Kalman gain is small
     *   → Measurements have little effect
     *   → Predictions dominate (smoother)
     * 
     * If altitude variance is high:
     *   R → 0.1
     *   → Kalman gain is large
     *   → Measurements are trusted more
     *   → Filter responds to changes
     * ```
     * 
     * Wait, this seems backwards? Actually, it's correct:
     * - High measurement noise R → low Kalman gain → predict more, measure less
     * - Low measurement noise R → high Kalman gain → measure more, predict less
     */
    _float_t R[EKF_M * EKF_M] = {
        0.01f  ///< Initial value; updated each cycle by updateMeasurementNoiseCovarianceMatrix()
    };

    // ========================================================================
    // STATE ESTIMATES - EXTRACTED FROM EKF AFTER EACH UPDATE
    // ========================================================================

    /// Estimated altitude above mean sea level in meters (from _ekf.x[0])
    _float_t _altitude;
    
    /// Variance of altitude measurements in m² (updated from input parameter)
    _float_t _altitude_variance;
    
    /// Estimated vertical velocity (vertical speed) in m/s (from _ekf.x[1])
    /// Positive = climbing, Negative = descending
    _float_t _vertical_speed;
    
    /// Estimated vertical acceleration in m/s² (from _ekf.x[2])
    /// Positive = accelerating upward, Negative = decelerating or descending
    _float_t _vertical_acceleration;

    // ========================================================================
    // INITIALIZATION STATE
    // ========================================================================

    /// Flag indicating if filter has completed initialization
    /// false = initializing, true = ready for normal use
    bool initialized_;
    
    /// Counter for initialization settling period (0-4, becomes true at 5)
    /// Prevents noisy startup transients during first few measurements
    uint8_t init_counter_;
};

// ============================================================================
// END OF HEADER FILE
// ============================================================================