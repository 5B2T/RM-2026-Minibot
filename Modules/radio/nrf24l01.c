#include "radio/nrf24l01.h"
#include "radio/nrf24l01_registers.h"
#include "radio/bsp_nrf24l01.h"
#include "time/bsp_time.h"
#include <string.h>
#define NRF_TX_TIMEOUT_MS 80U

static const uint8_t Address[RX_ADR_WIDTH]={0xb0,0x43,0x10,0x10,0x01};
static uint8_t Ready;
static uint8_t ReceiveMode;
static uint8_t TxPending,TxPulse;
static uint32_t TxStarted;

static NRF24L01_Result Fault(NRF24L01_Result Result)
{
    Ready=0;
    TxPending=0;
    BSP_NRF24L01_SetCE(0);
    return Result;
}

// One CSN transaction and one timeout for the whole command, up to 33 bytes.
static NRF24L01_Result Transfer(uint8_t Command, const uint8_t *Write,
                               uint8_t *Read, uint8_t Length)
{
    uint8_t Tx[TX_PLOAD_WIDTH+1U],Rx[RX_PLOAD_WIDTH+1U];
    if(Length>TX_PLOAD_WIDTH)return NRF_INVALID_ARGUMENT;
    memset(Tx, NOP, sizeof(Tx));
    Tx[0]=Command;
    if(Write)memcpy(Tx+1, Write, Length);
    if(!BSP_NRF24L01_Transfer(Tx, Rx, Length+1U))return Fault(NRF_SPI_ERROR);
    // STATUS bit 7 is reserved zero: floating MISO/all-ones is not a valid chip reply.
    if(Rx[0]&0x80U)return Fault(NRF_CONFIG_ERROR);
    if(Read)memcpy(Read, Rx+1, Length);
    return NRF_OK;
}

static NRF24L01_Result WriteReg(uint8_t Reg, uint8_t Value)
{
    return Transfer(NRF_WRITE_REG|Reg, &Value, 0, 1);
}

static NRF24L01_Result ReadReg(uint8_t Reg, uint8_t *Value)
{
    return Transfer(NRF_READ_REG|Reg, 0, Value, 1);
}

void NRF24L01_SPI_Init(void)
{
    Ready=0;
    BSP_NRF24L01_SetCE(0);
    BSP_NRF24L01_Init();
    // POR maximum 10.3 ms after VDD >= 1.9 V (Nordic specification v2.0).
    BSP_WaitMs(11);
}

static NRF24L01_Result VerifyConfig(void)
{
    const uint8_t Regs[]={CONFIG,EN_AA,EN_RXADDR,SETUP_AW,SETUP_RETR,
                          RF_CH,RF_SETUP,RX_PW_P0,NRF_DYNPD,NRF_FEATURE};
    const uint8_t Expected[]={ReceiveMode?0x0f:0x0e,1,1,3,0xff,40,0x0f,32,0,0};
    for(uint8_t i=0;i<sizeof(Regs);i++)
    {
        uint8_t Value;
        NRF24L01_Result Result=ReadReg(Regs[i], &Value);
        if(Result!=NRF_OK)return Result;
        // RF_SETUP bit 0 is LNA gain on L01, obsolete on L01+.
        uint8_t Mask=Regs[i]==RF_SETUP?0xfeU:0xffU;
        if((Value&Mask)!=(Expected[i]&Mask))return Fault(NRF_CONFIG_ERROR);
    }
    const uint8_t AddressRegs[]={RX_ADDR_P0,TX_ADDR};
    for(uint8_t i=0;i<sizeof(AddressRegs);i++)
    {
        uint8_t Value[RX_ADR_WIDTH];
        NRF24L01_Result Result=Transfer(AddressRegs[i], 0, Value, sizeof(Value));
        if(Result!=NRF_OK)return Result;
        if(memcmp(Value, Address, sizeof(Value)))return Fault(NRF_CONFIG_ERROR);
    }
    return NRF_OK;
}

NRF24L01_Result NRF24L01_Check(void)
{
    if(!Ready)return NRF_NOT_READY;
    return VerifyConfig();
}

static NRF24L01_Result Configure(uint8_t RxMode)
{
    Ready=0;
    TxPending=0;
    ReceiveMode=RxMode;
    BSP_NRF24L01_SetCE(0);
    // Configure while powered down, then explicitly wait for oscillator start-up.
    const uint8_t Regs[]={CONFIG,EN_AA,EN_RXADDR,SETUP_AW,SETUP_RETR,
                          RF_CH,RF_SETUP,RX_PW_P0,NRF_DYNPD,NRF_FEATURE};
    const uint8_t Values[]={0x0c,1,1,3,0xff,40,0x0f,32,0,0};
    for(uint8_t i=0;i<sizeof(Regs);i++)
    {
        NRF24L01_Result Result=WriteReg(Regs[i], Values[i]);
        if(Result!=NRF_OK)return Result;
    }
    NRF24L01_Result Result=Transfer(NRF_WRITE_REG|RX_ADDR_P0, Address, 0, sizeof(Address));
    if(Result!=NRF_OK)return Result;
    Result=Transfer(NRF_WRITE_REG|TX_ADDR, Address, 0, sizeof(Address));
    if(Result!=NRF_OK)return Result;
    // Flush only during mode initialization, never after receiving a normal packet.
    Result=Transfer(FLUSH_RX, 0, 0, 0);
    if(Result!=NRF_OK)return Result;
    Result=Transfer(FLUSH_TX, 0, 0, 0);
    if(Result!=NRF_OK)return Result;
    Result=WriteReg(STATUS, RX_OK|TX_OK|MAX_TX);
    if(Result!=NRF_OK)return Result;
    Result=WriteReg(CONFIG, RxMode?0x0f:0x0e);
    if(Result!=NRF_OK)return Result;
    BSP_WaitMs(2); // PWR_UP to standby: maximum 1.5 ms.
    Result=VerifyConfig();
    if(Result!=NRF_OK)return Result;
    if(RxMode)
    {
        BSP_NRF24L01_SetCE(1);
        BSP_WaitMs(1); // RX settling: 130 us; API returns only after RX is ready.
    }
    Ready=1;
    return NRF_OK;
}

NRF24L01_Result NRF24L01_RX_Mode(void)
{
    return Configure(1);
}

NRF24L01_Result NRF24L01_TX_Mode(void)
{
    return Configure(0);
}

NRF24L01_Result NRF24L01_RxPacket(uint8_t *RxBuf)
{
    if(!RxBuf)return NRF_INVALID_ARGUMENT;
    if(!Ready || !ReceiveMode)return NRF_NOT_READY;
    uint8_t Fifo;
    NRF24L01_Result Result=ReadReg(NRF_FIFO_STATUS, &Fifo);
    if(Result!=NRF_OK)return Result;
    // Use FIFO state: clearing RX_DR must not hide queued or newly arriving packets.
    if(Fifo&0x8cU)return Fault(NRF_CONFIG_ERROR);
    if(Fifo&1U)return NRF_EMPTY;
    Result=Transfer(RD_RX_PLOAD, 0, RxBuf, RX_PLOAD_WIDTH);
    if(Result!=NRF_OK)return Result;
    return WriteReg(STATUS, RX_OK); // Leave TX interrupt flags alone.
}

NRF24L01_Result NRF24L01_TxPacket(const uint8_t *TxBuf)
{
    if(!TxBuf)return NRF_INVALID_ARGUMENT;
    if(!Ready || ReceiveMode || TxPending)return NRF_NOT_READY;
    BSP_NRF24L01_SetCE(0);
    NRF24L01_Result Result=WriteReg(STATUS, TX_OK|MAX_TX);
    if(Result!=NRF_OK)return Result;
    Result=Transfer(WR_TX_PLOAD, TxBuf, 0, TX_PLOAD_WIDTH);
    if(Result!=NRF_OK)return Result;
    uint32_t Started=BSP_TimeMs();
    BSP_NRF24L01_SetCE(1);
    BSP_WaitMs(1); // CE pulse >= 10 us; wait yields when scheduler is running.
    BSP_NRF24L01_SetCE(0);
    // Poll STATUS rather than waiting forever for an IRQ pin transition.
    for(uint32_t Poll=0;Poll<NRF_TX_TIMEOUT_MS;Poll++)
    {
        uint8_t Status;
        Result=ReadReg(STATUS, &Status);
        if(Result!=NRF_OK)return Result;
        if(Status&(TX_OK|MAX_TX))
        {
            Result=WriteReg(STATUS, TX_OK|MAX_TX);
            if(Result!=NRF_OK)return Result;
            if(Status&MAX_TX)
            {
                Result=Transfer(FLUSH_TX, 0, 0, 0);
                return Result==NRF_OK?NRF_TX_MAX_RETRY:Result;
            }
            return NRF_OK;
        }
        if(BSP_TimeMs()-Started>=NRF_TX_TIMEOUT_MS)break;
        BSP_WaitMs(1);
    }
    Result=Transfer(FLUSH_TX, 0, 0, 0);
    return Result==NRF_OK?Fault(NRF_TX_TIMEOUT):Result;
}
NRF24L01_Result NRF24L01_TxBegin(const uint8_t *TxBuf)
{
    if(TxBuf==NULL)return NRF_INVALID_ARGUMENT;
    if(!Ready || ReceiveMode || TxPending)return NRF_NOT_READY;
    BSP_NRF24L01_SetCE(0);
    NRF24L01_Result Result=WriteReg(STATUS,TX_OK|MAX_TX);
    if(Result!=NRF_OK)return Result;
    Result=Transfer(WR_TX_PLOAD,TxBuf,0,TX_PLOAD_WIDTH);
    if(Result!=NRF_OK)return Result;
    TxStarted=BSP_TimeMs();TxPending=TxPulse=1;
    BSP_NRF24L01_SetCE(1);
    return NRF_PENDING;
}
NRF24L01_Result NRF24L01_TxAbort(void)
{
    if(!TxPending)return Ready?NRF_OK:NRF_NOT_READY;
    BSP_NRF24L01_SetCE(0);
    TxPending=0;
    NRF24L01_Result Result=WriteReg(CONFIG,0x0c); // Stop retries before FLUSH_TX.
    if(Result!=NRF_OK)return Result;
    Result=Transfer(FLUSH_TX,0,0,0);
    if(Result!=NRF_OK)return Result;
    Result=WriteReg(STATUS,TX_OK|MAX_TX);
    if(Result!=NRF_OK)return Result;
    Result=WriteReg(CONFIG,0x0e);
    if(Result!=NRF_OK)return Result;
    BSP_WaitMs(2);
    return NRF_OK;
}
NRF24L01_Result NRF24L01_TxPoll(void)
{
    if(!Ready || !TxPending)return NRF_NOT_READY;
    uint32_t Elapsed=BSP_TimeMs()-TxStarted;
    if(TxPulse)
    {
        if(Elapsed<1)return NRF_PENDING;
        BSP_NRF24L01_SetCE(0);TxPulse=0;
    }
    uint8_t Status;
    NRF24L01_Result Result=ReadReg(STATUS,&Status);
    if(Result!=NRF_OK)return Result;
    if(Status&(TX_OK|MAX_TX))
    {
        TxPending=0;
        Result=WriteReg(STATUS,TX_OK|MAX_TX);
        if(Result!=NRF_OK)return Result;
        if(Status&MAX_TX)
        {
            Result=Transfer(FLUSH_TX,0,0,0);
            return Result==NRF_OK?NRF_TX_MAX_RETRY:Result;
        }
        return NRF_OK;
    }
    if(Elapsed>=NRF_TX_TIMEOUT_MS)
    {
        Result=NRF24L01_TxAbort();
        return Result==NRF_OK?Fault(NRF_TX_TIMEOUT):Result;
    }
    return NRF_PENDING;
}
