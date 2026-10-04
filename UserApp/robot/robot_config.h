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
#endif
