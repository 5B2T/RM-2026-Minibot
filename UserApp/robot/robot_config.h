#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
// Current F407 car configuration. Sizes/units must not be copied from school hardware.
#define ROBOT_CONTROL_PERIOD_MS 10U
// Default car speeds; user confirmed floor-test performance on 2026-10-16.
#define ROBOT_DEFAULT_LINEAR_SPEED_MPS 0.54f
#define ROBOT_DEFAULT_TURN_RADPS 1.8f
#define ROBOT_CHASSIS_TEST_SPEED_MPS ROBOT_DEFAULT_LINEAR_SPEED_MPS
#define ROBOT_CHASSIS_TEST_TURN_RADPS ROBOT_DEFAULT_TURN_RADPS
#define ROBOT_CHASSIS_TEST_UI_TIMEOUT_MS 100U
#define ROBOT_CHASSIS_TEST_NO_ENCODER_MS 500U
#define ROBOT_CHASSIS_TEST_BACK_HOLD_MS 1000U
#define ROBOT_CHASSIS_TEST_REFRESH_MS 100U
#define ROBOT_CHASSIS_TEST_SQUARE_SIDE_MS 1000U
#define ROBOT_CHASSIS_TEST_SQUARE_LAPS 2U
#define ROBOT_COMM_POLL_MS 5U
#define ROBOT_RADIO_RETRY_MS 100U
#define ROBOT_REMOTE_TIMEOUT_MS 100U
#define ROBOT_WHEEL_DIAMETER_M 0.060f
// 13 encoder lines * 30 reduction * x4 decoding, per output-shaft revolution.
#define ROBOT_ENCODER_CPR 1560.0f
// Fallback PI gains from the original M2 tuning; measured channel overrides below.
#define ROBOT_SPEED_KP 1200.0f
#define ROBOT_SPEED_KI 10000.0f
// Physical M1..M4; all four tuned with wheels fitted and suspended.
// Gains use wheel-rim m/s, PWM counts and integral(error * seconds).
// Replacement M2 verified at +/-0.10, +/-0.25 and +/-0.35 m/s on 2026-10-15.
// M3/M4 verified at +/-0.10, +/-0.25 and +/-0.35 m/s on 2026-10-15.
// M1 verified at +/-0.10, +/-0.25 and +/-0.35 m/s on 2026-10-15.
// Higher-speed closed-loop operation and floor load require validation.
#define ROBOT_SPEED_KP_BY_CHANNEL {1600.0f,ROBOT_SPEED_KP,1200.0f,1600.0f}
#define ROBOT_SPEED_KI_BY_CHANNEL {20000.0f,16000.0f,16000.0f,20000.0f}
// M1 suspended open-loop: PWM 750 gave ~0.537 m/s; other wheels/load pending.
#define ROBOT_WHEEL_SPEED_LIMIT_MPS 0.54f
// Body axes: +x forward, +y left, +wz counterclockwise viewed from above.
// Positive wheel speed means that wheel drives the vehicle forward.
// Front is at the bottom of the supplied sketch: FL bottom-right, FR bottom-left,
// RL top-right, RR top-left. Proposed wiring allocation, not measured wiring.
#define ROBOT_CHANNEL_FL 0U
#define ROBOT_CHANNEL_FR 1U
#define ROBOT_CHANNEL_RL 2U
#define ROBOT_CHANNEL_RR 3U
#define ROBOT_WHEEL_CHANNEL_MAP {ROBOT_CHANNEL_FL,ROBOT_CHANNEL_FR,ROBOT_CHANNEL_RL,ROBOT_CHANNEL_RR}
// Center distances from supplied clearances: 250 mm fore/aft gap + 60 mm diameter;
// 234 mm side gap + measured 33.5 mm wheel width. Ground calibration pending.
#define ROBOT_CHASSIS_WHEELBASE_M 0.310f
#define ROBOT_CHASSIS_TRACK_M 0.2675f
// Indexed by physical M1..M4, each sign must be +1 or -1.
// +1 preserves original software polarity, which has NOT been checked on hardware.
#define ROBOT_MOTOR_POLARITY {1,1,1,1}
// M4 forward wheel motion was verified with negative raw encoder counts.
#define ROBOT_ENCODER_POLARITY {1,1,1,-1}
// SSD1306 four-pin module, seven-bit address. No external reset GPIO.
#define ROBOT_OLED_ADDRESS 0x3cU
#define ROBOT_DISPLAY_POWER_WAIT_MS 200U
#define ROBOT_DISPLAY_POLL_MS 5U
#define ROBOT_DISPLAY_REFRESH_MS 500U
#define ROBOT_BUTTON_DEBOUNCE_MS 30U
// J1 shoulder, J2 elbow, J3 wrist pitch, J4 wrist roll, J5 gripper.
// Electrical travel for pulse conversion; mechanical limits are separate below.
// User-specified nominal travel: JX6225 300 deg; MG90D 270 deg.
// Supplied screenshots conflict (295 deg / 200+/-5 deg respectively): verify on hardware.
// No old shoulder offset or continuous timing calibration. User confirmed original
// J1 direction: logical 0/300 deg -> 500/2500 us, increasing angle raises the arm.
#define ROBOT_ARM_SERVO_CONFIG { \
    {SERVO_POSITION,500,1500,2500,300.0f,1}, \
    {SERVO_POSITION,500,1500,2500,300.0f,1}, \
    {SERVO_POSITION,500,1500,2500,270.0f,1}, \
    {SERVO_POSITION,500,1500,2500,180.0f,1}, \
    {SERVO_POSITION,500,1500,2500,180.0f,1} }
// User-confirmed structural limits, 2026-10-22. Never rescale the PWM travel to these.
#define ROBOT_ARM_POSITION_MIN_DEG {10.0f,23.0f,13.0f,0.0f,0.0f}
#define ROBOT_ARM_POSITION_MAX_DEG {150.0f,180.0f,270.0f,180.0f,30.0f}
// Positive jog / stick-up raises J1/J2/J3; J3 is higher at the SMALLER angle.
#define ROBOT_ARM_JOG_DIRECTION {1,1,-1,1,1}
// SW6 / INIT: J1, parallel J2/J3/J4, then open gripper.
// Fold: J3 + close gripper together, then J2 -> J4 -> J1.
#define ROBOT_ARM_POSES_CALIBRATED 1U
#define ROBOT_ARM_GRIPPER_CALIBRATED 1U
#define ROBOT_ARM_GRIPPER_CLOSED_DEG 0.0f
#define ROBOT_ARM_GRIPPER_OPEN_DEG 30.0f
#define ROBOT_ARM_DEPLOY_DEG {90.0f,103.0f,270.0f,0.0f,ROBOT_ARM_GRIPPER_OPEN_DEG}
#define ROBOT_ARM_SEQUENCE_SETTLE_MS 200U
#define ROBOT_ARM_SEQUENCE_MAX_MS 20000U
// User manually places these four joint angles BEFORE power-on. This is an assumed
// reference, not measured homing; still no PWM until explicit control.
#define ROBOT_ARM_PRESET_DEG {10.0f,23.0f,13.0f,180.0f,0.0f}
#define ROBOT_ARM_POSITION_RATE_DPS 90.0f
#define ROBOT_ARM_COMMAND_MAX_MS 5000U
#define ROBOT_ARM_DEBUG_TIMEOUT_MS 300U
#define ROBOT_ARM_MAX_SAMPLE_S 0.1f
#define ROBOT_ARM_BASE_PWM_LIMIT 300
// 11PPR * 150 reduction * x4. Confirm actual gearing before angle calibration.
#define ROBOT_ARM_BASE_CPR 6600.0f
#define ROBOT_ARM_BASE_MOTOR_SIGN 1
#define ROBOT_ARM_BASE_ENCODER_SIGN 1
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
