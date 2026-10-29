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
#define ROBOT_CHASSIS_TEST_REFRESH_MS 100U
#define ROBOT_CHASSIS_TEST_SQUARE_SIDE_MS 1000U
#define ROBOT_CHASSIS_TEST_SQUARE_LAPS 2U
#define ROBOT_COMM_POLL_MS 5U
#define ROBOT_RADIO_RETRY_MS 100U
#define ROBOT_REMOTE_TIMEOUT_MS 100U
// Pot telemetry arrives every ~100 ms and never counts as a command frame.
#define ROBOT_TELEMETRY_TIMEOUT_MS 300U
#define ROBOT_WHEEL_DIAMETER_M 0.060f
// 13 encoder lines * 30 reduction * x4 decoding, per output-shaft revolution.
#define ROBOT_ENCODER_CPR 1560.0f
// Fallback PI gains from the original M2 tuning; measured channel overrides below.
#define ROBOT_SPEED_KP 1200.0f
#define ROBOT_SPEED_KI 10000.0f
// Chassis array [M1,M2,M3,M4], retuned with wheels installed/suspended2026-10-29.
// Gains use wheel-rim m/s, PWM counts and integral(error * seconds).
// All four verified at +/-0.10, +/-0.30 and +/-0.54m/s; floor load still pending.
#define ROBOT_SPEED_KP_BY_CHANNEL {1400.0f,1200.0f,1600.0f,1600.0f}
#define ROBOT_SPEED_KI_BY_CHANNEL {18000.0f,16000.0f,20000.0f,20000.0f}
#define ROBOT_WHEEL_SPEED_LIMIT_MPS 0.54f
// Body axes: +x forward, +y left, +wz counterclockwise viewed from above.
// Positive wheel speed means that wheel drives the vehicle forward.
// Front is at the bottom of the supplied sketch: FL bottom-right, FR bottom-left,
// RL top-right, RR top-left. Proposed wiring allocation, not measured wiring.
// Final TB mapping changes hardware channels, not wheel identities. M5 is the base.
#define ROBOT_CHANNEL_FL 0U
#define ROBOT_CHANNEL_FR 1U
#define ROBOT_CHANNEL_RL 2U
#define ROBOT_CHANNEL_RR 3U
#define ROBOT_WHEEL_CHANNEL_MAP {ROBOT_CHANNEL_FL,ROBOT_CHANNEL_FR,ROBOT_CHANNEL_RL,ROBOT_CHANNEL_RR}
// Center distances from supplied clearances: 250 mm fore/aft gap + 60 mm diameter;
// 234 mm side gap + measured 33.5 mm wheel width. Ground calibration pending.
#define ROBOT_CHASSIS_WHEELBASE_M 0.310f
#define ROBOT_CHASSIS_TRACK_M 0.2675f
// Indexed by chassis motors [M1,M2,M3,M4], each sign must be +1 or -1.
// 2026-10-29: after sender-axis correction, user confirms rear M3/M4 reversed.
// Correct rear drive and feedback together; front M1/M2 remain unchanged.
#define ROBOT_MOTOR_POLARITY {-1,-1,1,1}
#define ROBOT_ENCODER_POLARITY {-1,-1,1,1}
// SSD1306 four-pin module, seven-bit address. No external reset GPIO.
#define ROBOT_OLED_ADDRESS 0x3cU
#define ROBOT_DISPLAY_POWER_WAIT_MS 200U
#define ROBOT_DISPLAY_POLL_MS 5U
#define ROBOT_DISPLAY_REFRESH_MS 500U
#define ROBOT_BUTTON_DEBOUNCE_MS 30U
#define ROBOT_BUTTON_HOLD_MS 700U
#define ROBOT_BUTTON_DOUBLE_MS 300U
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
// User-confirmed structural limits, updated 2026-10-28. Preserve electrical PWM scaling.
#define ROBOT_ARM_POSITION_MIN_DEG {5.0f,23.0f,22.0f,0.0f,6.0f}
#define ROBOT_ARM_POSITION_MAX_DEG {150.0f,180.0f,270.0f,180.0f,45.0f}
// Positive jog / stick-up raises J1/J2/J3; J3 is higher at the SMALLER angle.
#define ROBOT_ARM_JOG_DIRECTION {1,1,-1,1,1}
// SW6 / INIT: J1, parallel J2/J3/J4, then open gripper.
// Fold: J3 + wrist-clear + close gripper + base zero, then J2 -> J4 -> J1.
#define ROBOT_ARM_POSES_CALIBRATED 1U
#define ROBOT_ARM_GRIPPER_CALIBRATED 1U
#define ROBOT_ARM_GRIPPER_CLOSED_DEG 6.0f
#define ROBOT_ARM_GRIPPER_OPEN_DEG 45.0f
#define ROBOT_ARM_DEPLOY_DEG {95.0f,103.0f,190.0f,0.0f,ROBOT_ARM_GRIPPER_OPEN_DEG}
#define ROBOT_ARM_FOLD_WRIST_CLEAR_DEG 0.0f
#define ROBOT_ARM_BASE_RETURN_TIMEOUT_MS 12000U
#define ROBOT_ARM_SEQUENCE_SETTLE_MS 200U
#define ROBOT_ARM_SEQUENCE_MAX_MS 20000U
// User manually places these four joint angles BEFORE power-on. This is an assumed
// reference, not measured homing; still no PWM until explicit control.
#define ROBOT_ARM_PRESET_DEG {5.0f,23.0f,22.0f,90.0f,ROBOT_ARM_GRIPPER_CLOSED_DEG}
#define ROBOT_ARM_POSITION_RATE_DPS 90.0f
#define ROBOT_ARM_COMMAND_MAX_MS 5000U
#define ROBOT_ARM_DEBUG_TIMEOUT_MS 300U
#define ROBOT_ARM_MAX_SAMPLE_S 0.1f
#define ROBOT_ARM_RECOVERY_WAIT_MS 2000U
#define ROBOT_ARM_SEQUENCE_TRIGGER_RATE .15f
#define ROBOT_ARM_SEQUENCE_OVERRIDE_RATE .25f
#define ROBOT_ARM_SEQUENCE_OVERRIDE_MS 200U
#define ROBOT_ARM_BASE_PWM_LIMIT 700
// 11PPR * 150 reduction * x4. Confirm actual gearing before angle calibration.
#define ROBOT_ARM_BASE_CPR 6600.0f
#define ROBOT_ARM_BASE_MOTOR_SIGN 1
#define ROBOT_ARM_BASE_ENCODER_SIGN -1
// RPM uses provisional 6600 CPR. Kd=0; gains in PWM per RPM and PWM per RPM-second.
// Loaded M5/TB5 speed target: 15 RPM = 90 deg/s, ramped at 20 RPM/s.
// 2026-10-27 bench fit: speed/static feedforward 28/40, filter 30 ms, fault lease 300 ms.
#define ROBOT_ARM_BASE_CONTROL_CONFIG {40.0f,120.0f,28.0f,40.0f,15.0f,ROBOT_ARM_BASE_PWM_LIMIT,20.0f,.03f,.12f,.3f}
// Position mode only, relative to the manually aligned boot zero. No angular wrap.
// +/-180 deg, 90 deg/s reference slew; P outer loop returns RPM to the speed PI.
// Enter radio follow only within 3deg and0.4RPM. Faster P response for teaching control;
// smaller arrival/hold band permits sub-degree corrections, still capped at joystick15RPM.
#define ROBOT_ARM_BASE_POSITION_CONFIG {-180.0f,180.0f,15.0f,90.0f,.6f,2.0f,.3f,.7f,3.0f,.4f}
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
