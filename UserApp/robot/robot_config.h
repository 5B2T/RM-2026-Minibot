#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
// Current F407 car configuration. Sizes/units must not be copied from school hardware.
#define ROBOT_CONTROL_PERIOD_MS 10U
#define ROBOT_COMM_POLL_MS 5U
#define ROBOT_RADIO_RETRY_MS 100U
#define ROBOT_REMOTE_TIMEOUT_MS 100U
#define ROBOT_WHEEL_DIAMETER_M 0.060f
// 13 encoder lines * 30 reduction * x4 decoding, per output-shaft revolution.
#define ROBOT_ENCODER_CPR 1560.0f
// Default PI gains for all four wheels, based on M2 suspended-wheel tuning.
#define ROBOT_SPEED_KP 1200.0f
#define ROBOT_SPEED_KI 10000.0f
// Physical M1..M4; individual channels can be overridden after further tuning.
// Gains use wheel-rim m/s, PWM counts and integral(error * seconds).
// M2 verified at 0.10..0.35 m/s forward and 0.15..0.30 m/s reverse;
// floor load, the other wheels and the full 0.5 m/s limit remain unverified.
#define ROBOT_SPEED_KP_BY_CHANNEL {ROBOT_SPEED_KP,ROBOT_SPEED_KP,ROBOT_SPEED_KP,ROBOT_SPEED_KP}
#define ROBOT_SPEED_KI_BY_CHANNEL {ROBOT_SPEED_KI,ROBOT_SPEED_KI,ROBOT_SPEED_KI,ROBOT_SPEED_KI}
#define ROBOT_WHEEL_SPEED_LIMIT_MPS 0.5f
// Body axes: +x forward, +y left, +wz counterclockwise viewed from above.
// Positive wheel speed means that wheel drives the vehicle forward.
// Front is at the bottom of the supplied sketch: FL bottom-right, FR bottom-left,
// RL top-right, RR top-left. Proposed wiring allocation, not measured wiring.
#define ROBOT_CHANNEL_FL 0U
#define ROBOT_CHANNEL_FR 1U
#define ROBOT_CHANNEL_RL 2U
#define ROBOT_CHANNEL_RR 3U
#define ROBOT_WHEEL_CHANNEL_MAP {ROBOT_CHANNEL_FL,ROBOT_CHANNEL_FR,ROBOT_CHANNEL_RL,ROBOT_CHANNEL_RR}
// Provisional center distances: 250 mm rim gap + 60 mm diameter;
// 234 mm side gap + assumed 30 mm wheel width. Replace with measured values.
#define ROBOT_CHASSIS_WHEELBASE_M 0.310f
#define ROBOT_CHASSIS_TRACK_M 0.264f
// Indexed by physical M1..M4, each sign must be +1 or -1.
// +1 preserves original software polarity, which has NOT been checked on hardware.
#define ROBOT_MOTOR_POLARITY {1,1,1,1}
#define ROBOT_ENCODER_POLARITY {1,1,1,1}
// SSD1306 four-pin module, seven-bit address. No external reset GPIO.
#define ROBOT_OLED_ADDRESS 0x3cU
#define ROBOT_DISPLAY_POWER_WAIT_MS 200U
#define ROBOT_DISPLAY_POLL_MS 5U
#define ROBOT_DISPLAY_REFRESH_MS 500U
#define ROBOT_BUTTON_DEBOUNCE_MS 30U
#define ROBOT_DISPLAY_RETRY_MS 1000U
// R100 (0.1 ohm), A0/A1 open jumpers, seven-bit address.
// +/-320 mV covers +/-3.2 A; 0.1 mA register scaling covers that range.
#define ROBOT_INA219_ADDRESS 0x40U
#define ROBOT_INA219_SHUNT_OHM 0.1f
#define ROBOT_INA219_CURRENT_LSB_A 0.0001f
#define ROBOT_POWER_SAMPLE_MS 100U
#define ROBOT_POWER_CONVERSION_TIMEOUT_MS 100U
#define ROBOT_POWER_RETRY_MS 1000U
#define ROBOT_POWER_STALE_MS 500U
// Confirmed ordinary Li-ion / Li-polymer 3S, 2500 mAh, full pack 12.6 V.
#define ROBOT_BATTERY_SERIES_CELLS 3U
#define ROBOT_BATTERY_CAPACITY_MAH 2500.0f
#define ROBOT_BATTERY_FILTER_MS 3000U
// Provisional single-cell voltage/SOC points, NOT a measured manufacturer curve.
// Replace after characterizing this pack. No internal-resistance compensation,
// coulomb counting or persistence; loaded voltage and reboot affect the estimate.
// The 0% point is a display reference, not a protection/discharge cutoff.
#define ROBOT_BATTERY_SOC_CURVE {{3.30f,0},{3.60f,5},{3.70f,10},{3.75f,20}, \
    {3.80f,30},{3.85f,40},{3.90f,50},{3.95f,60},{4.00f,70},{4.10f,90},{4.20f,100}}
#endif
