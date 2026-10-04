#ifndef INA219_H
#define INA219_H
#include <stdint.h>
typedef uint8_t (*INA219_WriteFn)(uint8_t,const uint8_t *,uint16_t);
typedef uint8_t (*INA219_ReadFn)(uint8_t,uint8_t,uint8_t *,uint16_t);
typedef enum {INA219_PENDING=0,INA219_SAMPLE,INA219_IO_ERROR,INA219_CONFIG_ERROR,
    INA219_OVERFLOW,INA219_TIMEOUT,INA219_INVALID_SAMPLE} INA219_Result;
typedef struct {float VoltageV,CurrentA,PowerW;} INA219_Sample;
// Single service-task owner. 32 V ADC scaling, +/-320 mV shunt range,
// 16-sample averaging for both ADCs, triggered conversion for a coherent sample.
// CurrentLSB must cover the full shunt range; effective scaling follows calibration readback.
uint8_t INA219_Init(INA219_WriteFn Write,INA219_ReadFn Read,uint8_t Address,
    float ShuntOhm,float CurrentLSB,uint32_t ConversionTimeoutMs);
// At most one register transaction per call; never busy-waits for a conversion.
// Output changes only for INA219_SAMPLE. New conversion starts after next Step call.
INA219_Result INA219_Step(uint32_t NowMs,INA219_Sample *Output);
#endif
