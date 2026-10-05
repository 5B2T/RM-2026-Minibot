#ifndef BATTERY_SOC_H
#define BATTERY_SOC_H
#include <stdint.h>

typedef struct {float CellVoltageV,Percent;} BatterySoc_Point;
typedef struct
{
    const BatterySoc_Point *Curve; // Immutable, ascending single-cell curve; caller-owned lifetime.
    uint8_t PointCount,SeriesCells;
    float CapacityMah;
    uint32_t FilterTimeMs,StaleMs;
} BatterySoc_Config;
typedef struct
{
    float Percent,RemainingMah; // Approximate, based on nominal capacity, not measured capacity.
    uint8_t Valid;
} BatterySoc_Result;
typedef struct
{
    BatterySoc_Config Config;
    float FilteredVoltageV;
    uint32_t TickMs;
    uint8_t Configured,Valid;
} BatterySoc;

uint8_t BatterySoc_Init(BatterySoc *Self,const BatterySoc_Config *Config);
void BatterySoc_Invalidate(BatterySoc *Self);
// Only call for a new valid source-voltage sample; tick differences support wraparound.
void BatterySoc_Update(BatterySoc *Self,float VoltageV,uint32_t NowMs);
BatterySoc_Result BatterySoc_Get(const BatterySoc *Self,uint32_t NowMs);
#endif
