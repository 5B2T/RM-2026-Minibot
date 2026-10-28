#include "fault_handler.h"
#include "board_config.h"
// Retained for debugger diagnosis until an explicit reset. No runtime recovery
// is attempted after CPU/memory corruption: the assumed arm reference may be lost.
volatile struct {uint32_t Reason,Cfsr,Hfsr;} BSP_FatalRecord;
void BSP_EmergencyHalt(uint32_t Reason)
{
    __disable_irq();
    // Fixed F407 board resources, safe even before HAL handles are initialized.
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;
    BOARD_STBY_1_PORT->BSRR=(uint32_t)BOARD_STBY_1_PIN<<16;
    BOARD_STBY_2_PORT->BSRR=(uint32_t)BOARD_STBY_2_PIN<<16;
    BOARD_STBY_3_PORT->BSRR=(uint32_t)BOARD_STBY_3_PIN<<16;
    if(RCC->APB2ENR&RCC_APB2ENR_TIM1EN){TIM1->CCR1=0;TIM1->CCR2=0;TIM1->CCR3=0;TIM1->CCR4=0;}
    if(RCC->APB1ENR&RCC_APB1ENR_TIM13EN)TIM13->CCR1=0;
    // Freeze servo signals at their last pulse: do not release a loaded arm.
    // With interrupts off the autonomous timer PWM holds that last command.
    BSP_FatalRecord.Reason=Reason;BSP_FatalRecord.Cfsr=SCB->CFSR;BSP_FatalRecord.Hfsr=SCB->HFSR;
    for(;;){}
}
