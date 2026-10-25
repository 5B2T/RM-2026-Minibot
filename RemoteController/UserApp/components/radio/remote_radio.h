#ifndef REMOTE_RADIO_SERVICE_H
#define REMOTE_RADIO_SERVICE_H
#include <stdint.h>
#include "command/telemetry_protocol.h"
void RemoteRadio_Init(uint32_t NowMs);
void RemoteRadio_Step(uint32_t NowMs);
// Chip 2 only: queue the latest pot telemetry. The service sends it in the gaps between
// command frames, so a telemetry frame never displaces a command or its ACK lease.
void RemoteRadio_SetTelemetry(const RemoteTelemetry_FrameTypeDef *Frame);
#endif
