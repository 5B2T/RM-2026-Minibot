#ifndef REMOTE_CONFIG_H
#define REMOTE_CONFIG_H
#define REMOTE_USE_HSI 1U
#define REMOTE_INPUT_PERIOD_MS 5U
#define REMOTE_INPUT_STALE_MS 30U
#define REMOTE_TX_PERIOD_MS 20U
#define REMOTE_LINK_STALE_MS 80U
#define REMOTE_RADIO_RETRY_MS 500U
#define REMOTE_DEBOUNCE_MS 20U
#define REMOTE_CENTER_WAIT_MS 500U
#define REMOTE_AXIS_DEADZONE 160
#define REMOTE_CENTER_MIN 1536
#define REMOTE_CENTER_MAX 2560
#define REMOTE_CENTER_STABILITY 40
#define REMOTE_TURN_ACCEL_RAD_S2 2.0f
#define REMOTE_SPEED_MPS .54f
#define REMOTE_TURN_RAD_S 1.8f
// ADC order: PA1, PA2, PA3, PA6, PA4. Left stick corrected from field observation:
// old left -> forward; new forward = ADC0+, new left = ADC1+.
#define REMOTE_LEFT_X_INDEX 1U
#define REMOTE_LEFT_Y_INDEX 0U
#define REMOTE_RIGHT_X_INDEX 2U
#define REMOTE_RIGHT_Y_INDEX 3U
// Right stick up decreases ADC3; invert Y only so up commands positive joint motion.
#define REMOTE_AXIS_SIGNS {1,-1,1,-1}
#define REMOTE_ADC_REFERENCE_V 3.3f
#define REMOTE_POWER_DIVIDER 3.0f
#endif
