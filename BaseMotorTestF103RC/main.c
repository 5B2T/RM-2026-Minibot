#include "stm32f1xx_hal.h"
#include "motor_test.h"
#include <stdio.h>
#include <limits.h>

static TIM_HandleTypeDef Pwm,Encoder;
static UART_HandleTypeDef Uart;
static MotorTest Test;
// Ozone/Watch: write 1=forward, 2=reverse, 3=one cycle, 4=stop; consumed back to 0.
volatile uint32_t g_test_command,g_tick_ms,g_ready,g_error,g_encoder_ab;
volatile int32_t g_pwm,g_encoder_delta,g_encoder_count;
static uint16_t LastEncoder;
static uint32_t SampleTick,ReportTick;

static void MotorOutput(int PwmValue)
{
    if(PwmValue!=0 && PwmValue==g_pwm)return;
    // Output transitions always disable STBY before altering direction/CCR.
    GPIOB->BRR=GPIO_PIN_10;
    TIM4->CCR1=0;
    if(PwmValue==0) { GPIOB->BRR=GPIO_PIN_0|GPIO_PIN_1;g_pwm=0;return; }
    if(PwmValue>0) { GPIOB->BRR=GPIO_PIN_1;GPIOB->BSRR=GPIO_PIN_0; }
    else { GPIOB->BRR=GPIO_PIN_0;GPIOB->BSRR=GPIO_PIN_1; }
    TIM4->CCR1=(uint32_t)(PwmValue>0?PwmValue:-PwmValue);
    GPIOB->BSRR=GPIO_PIN_10;
    g_pwm=PwmValue;
}
static void Fail(uint32_t Reason)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_TIM4_CLK_ENABLE();
    MotorOutput(0);g_ready=0;g_error=Reason;
    __disable_irq();while(1){}
}
static void Check(HAL_StatusTypeDef Result,uint32_t Reason) { if(Result!=HAL_OK)Fail(Reason); }
void SysTick_Handler(void) { HAL_IncTick(); }
void HardFault_Handler(void) { Fail(90); }
void MemManage_Handler(void) { Fail(91); }
void BusFault_Handler(void) { Fail(92); }
void UsageFault_Handler(void) { Fail(93); }

static void ClockInit(void)
{
    RCC_OscInitTypeDef Osc={0};RCC_ClkInitTypeDef Clk={0};
    Osc.OscillatorType=RCC_OSCILLATORTYPE_HSI;Osc.HSIState=RCC_HSI_ON;
    Osc.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
    Osc.PLL.PLLState=RCC_PLL_ON;Osc.PLL.PLLSource=RCC_PLLSOURCE_HSI_DIV2;Osc.PLL.PLLMUL=RCC_PLL_MUL16;
    Check(HAL_RCC_OscConfig(&Osc),1);
    Clk.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    Clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;Clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    Clk.APB1CLKDivider=RCC_HCLK_DIV2;Clk.APB2CLKDivider=RCC_HCLK_DIV1;
    Check(HAL_RCC_ClockConfig(&Clk,FLASH_LATENCY_2),2);
}
static void PeripheralsInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();__HAL_RCC_TIM3_CLK_ENABLE();__HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitTypeDef G={0};
    // PB0 AIN1, PB1 AIN2, PB10 STBY, initially low; no pin remapping needed.
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10,GPIO_PIN_RESET);
    G.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10;G.Mode=GPIO_MODE_OUTPUT_PP;G.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB,&G);
    G.Pin=GPIO_PIN_6;G.Mode=GPIO_MODE_AF_PP;G.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&G); // PB6 PWM; PA8 is left untouched.
    G.Pin=GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA,&G); // PA9 UART TX.
    G.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10;G.Mode=GPIO_MODE_INPUT;G.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA,&G); // PA6/7 C1/C2; PA10 UART RX.

    Pwm.Instance=TIM4;Pwm.Init.Prescaler=31;Pwm.Init.CounterMode=TIM_COUNTERMODE_UP;
    Pwm.Init.Period=999;Pwm.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;Pwm.Init.RepetitionCounter=0;
    Check(HAL_TIM_PWM_Init(&Pwm),3); // 64 MHz / 32 / 1000 = 2 kHz.
    TIM_OC_InitTypeDef OC={0};OC.OCMode=TIM_OCMODE_PWM1;OC.Pulse=0;
    OC.OCPolarity=TIM_OCPOLARITY_HIGH;OC.OCNPolarity=TIM_OCNPOLARITY_HIGH;
    OC.OCFastMode=TIM_OCFAST_DISABLE;OC.OCIdleState=TIM_OCIDLESTATE_RESET;OC.OCNIdleState=TIM_OCNIDLESTATE_RESET;
    Check(HAL_TIM_PWM_ConfigChannel(&Pwm,&OC,TIM_CHANNEL_1),4);
    Check(HAL_TIM_PWM_Start(&Pwm,TIM_CHANNEL_1),5);
    MotorOutput(0);

    Encoder.Instance=TIM3;Encoder.Init.Prescaler=0;Encoder.Init.CounterMode=TIM_COUNTERMODE_UP;
    Encoder.Init.Period=65535;Encoder.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    TIM_Encoder_InitTypeDef E={0};E.EncoderMode=TIM_ENCODERMODE_TI12;
    E.IC1Polarity=TIM_ICPOLARITY_RISING;E.IC1Selection=TIM_ICSELECTION_DIRECTTI;
    E.IC1Prescaler=TIM_ICPSC_DIV1;E.IC1Filter=4;
    E.IC2Polarity=TIM_ICPOLARITY_RISING;E.IC2Selection=TIM_ICSELECTION_DIRECTTI;
    E.IC2Prescaler=TIM_ICPSC_DIV1;E.IC2Filter=4;
    Check(HAL_TIM_Encoder_Init(&Encoder,&E),6);Check(HAL_TIM_Encoder_Start(&Encoder,TIM_CHANNEL_ALL),7);
    LastEncoder=(uint16_t)TIM3->CNT;

    Uart.Instance=USART1;Uart.Init.BaudRate=115200;Uart.Init.WordLength=UART_WORDLENGTH_8B;
    Uart.Init.StopBits=UART_STOPBITS_1;Uart.Init.Parity=UART_PARITY_NONE;Uart.Init.Mode=UART_MODE_TX_RX;
    Uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;Uart.Init.OverSampling=UART_OVERSAMPLING_16;
    Check(HAL_UART_Init(&Uart),8);
}
static void SampleEncoder(uint32_t Now)
{
    if((uint32_t)(Now-SampleTick)<10U)return;
    SampleTick=Now;
    uint16_t Count=(uint16_t)TIM3->CNT;
    g_encoder_delta=MotorTest_EncoderDelta(Count,LastEncoder);LastEncoder=Count;
    int64_t Total=(int64_t)g_encoder_count+g_encoder_delta;
    g_encoder_count=Total>INT32_MAX?INT32_MAX:Total<INT32_MIN?INT32_MIN:(int32_t)Total;
    g_encoder_ab=(GPIOA->IDR>>6)&3U; // Bit0 C1; bit1 C2.
}
static void Report(uint32_t Now)
{
    if((uint32_t)(Now-ReportTick)<200U)return;
    ReportTick=Now;char Text[128];
    int N=snprintf(Text,sizeof(Text),"ms=%lu pwm=%ld enc=%ld d=%ld ab=%lu mode=%u err=%lu\r\n",
        (unsigned long)Now,(long)g_pwm,(long)g_encoder_count,(long)g_encoder_delta,
        (unsigned long)g_encoder_ab,(unsigned)Test.Mode,(unsigned long)g_error);
    if(N>0 && (unsigned)N<sizeof(Text))
        (void)HAL_UART_Transmit(&Uart,(uint8_t *)Text,(uint16_t)N,10);
}
int main(void)
{
    HAL_Init();ClockInit();PeripheralsInit();MotorTest_Init(&Test);g_ready=1;
    static const uint8_t Hello[]="BASE MOTOR TEST F103RC: F/R=1s, T=one cycle, S=stop\r\n";
    (void)HAL_UART_Transmit(&Uart,(uint8_t *)Hello,sizeof(Hello)-1U,10);
    while(1)
    {
        uint32_t Now=HAL_GetTick();g_tick_ms=Now;
        // Bounded polling; no UART/TIM interrupts, FreeRTOS, OLED or radio.
        for(unsigned i=0;i<16U && __HAL_UART_GET_FLAG(&Uart,UART_FLAG_RXNE);i++)
        {
            uint32_t Status=USART1->SR;uint8_t Byte=(uint8_t)USART1->DR;
            if(Status&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE))
                MotorTest_Command(&Test,'S',Now);
            else MotorTest_Command(&Test,Byte,Now);
        }
        uint32_t Command=g_test_command;g_test_command=0;
        static const uint8_t Keys[]={0,'F','R','T','S'};
        if(Command>0 && Command<sizeof(Keys))MotorTest_Command(&Test,Keys[Command],Now);
        MotorOutput(MotorTest_Step(&Test,Now));
        SampleEncoder(Now);Report(Now);HAL_Delay(1);
    }
}
