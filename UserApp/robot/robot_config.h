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
// Commissioning values; gains and limit require vehicle verification.
#define ROBOT_SPEED_KP 600.0f
#define ROBOT_SPEED_KI 80.0f
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
#define ROBOT_DISPLAY_PAGE_MS 5000U
#define ROBOT_DISPLAY_RETRY_MS 1000U
#endif
