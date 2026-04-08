/**
 * @file    launch_control.h
 * @brief   Launch control system for FSAE EV acceleration event
 */

#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include <stdint.h>



// Overall gear ratio: motor RPM / wheel RPM
#define LC_GEAR_RATIO           0.2750275f

// tire radius in meters
#define LC_TIRE_RADIUS_M        .2f

// CAN ID for front wheel speed broadcast
#define LC_AIM_WHEEL_SPEED_CAN_ID   0x300

// LC debug CAN message ID
#define LC_DEBUG_CAN_ID         0x557

#define LC_SLIP_TARGET          0.10f
#define LC_CROSSOVER_SPEED_MS   2f
#define LC_EXIT_SPEED_MS        20f

// Minimum throttle position to trigger launch. Driver gotta be rlly bout it
#define LC_THROTTLE_TRIGGER     0.90f

#define LC_KP                   5000f
#define LC_KI                   500f

#define LC_SLIP_RATE_MAX        3f
#define LC_SLIP_RATE_CUT_MULT   0.5f

#define LC_SLIP_FILTER_ALPHA    0.3f

// AiM front wheel speed CAN message timeout (ms).
#define LC_SENSOR_TIMEOUT_MS    100

// Assumed control loop period
#define LC_DT_S                 0.010f

// Precomputed conversion: RPM → m/s = RPM * (2π/60) * tire_radius / gear_ratio
#define LC_RPM_TO_MS  ((2.0f * 3.14159265f / 60.0f) * LC_TIRE_RADIUS_M / LC_GEAR_RATIO)
// Precomputed inverse: m/s → RPM
#define LC_MS_TO_RPM  (LC_GEAR_RATIO * 60.0f / (2.0f * 3.14159265f * LC_TIRE_RADIUS_M))
// Precomputed: km/h×10 → m/s = val / 36.0
#define LC_KMH10_TO_MS (1.0f / 36.0f)

typedef enum {
    LC_STATE_IDLE,              
    LC_STATE_ARMED,             
    LC_STATE_LAUNCHING_OPENLOOP,
    LC_STATE_LAUNCHING_CLOSEDLOOP 
} lc_state_t;


typedef struct {
    lc_state_t state;           
    float      slip_ratio;      // Current filtered slip ratio
    float      slip_ratio_raw;  // Unfiltered slip ratio
    float      slip_rate;       // dλ/dt (slip rate of change)
    float      pi_p_term;       // Proportional term output
    float      pi_i_term;       // Integral term (accumulated)
    float      pi_output;       // Combined PI output (correction)
    uint32_t   lc_torque;       // Torque limit from LC (×10 units)
    uint32_t   map_torque;      // Open-loop map torque (×10 units)
    float      vehicle_speed;   // Estimated vehicle speed (m/s)
    float      rear_wheel_speed; // Rear wheel speed from motor (m/s)
    uint8_t    sensor_healthy;  // 1 = front wheel speed CAN is live
    uint8_t    slip_rate_cut;   // 1 = emergency slip rate cut active
} lc_debug_t;



void lc_init(void);


uint32_t lc_update(uint32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined);
void lc_feed_wheel_speed(uint16_t front_left_speed_x10, uint16_t front_right_speed_x10);
void lc_feed_tick(uint32_t tick_ms);
const lc_debug_t* lc_get_debug(void);
lc_state_t lc_get_state(void);
extern volatile uint8_t launch_control_enable;

#endif /* LAUNCH_CONTROL_H */
