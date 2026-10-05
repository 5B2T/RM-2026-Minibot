#include "power/battery_soc.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

uint8_t BatterySoc_Init(BatterySoc *Self,const BatterySoc_Config *Config)
{
    if(Self==NULL)return 0;
    memset(Self,0,sizeof(*Self));
    if(Config==NULL || Config->Curve==NULL || Config->PointCount<2 ||
        Config->SeriesCells==0 || !isfinite(Config->CapacityMah) || Config->CapacityMah<=0 ||
        Config->FilterTimeMs==0 || Config->StaleMs==0)return 0;
    for(unsigned i=0;i<Config->PointCount;i++)
    {
        BatterySoc_Point Point=Config->Curve[i];
        if(!isfinite(Point.CellVoltageV) || Point.CellVoltageV<=0 ||
            !isfinite(Point.Percent) || Point.Percent<0 || Point.Percent>100)return 0;
        if(i && (Point.CellVoltageV<=Config->Curve[i-1].CellVoltageV ||
            Point.Percent<=Config->Curve[i-1].Percent))return 0;
    }
    if(Config->Curve[0].Percent!=0 || Config->Curve[Config->PointCount-1].Percent!=100)return 0;
    Self->Config=*Config;
    Self->Configured=1;
    return 1;
}
void BatterySoc_Invalidate(BatterySoc *Self)
{
    if(Self!=NULL)Self->Valid=0;
}
void BatterySoc_Update(BatterySoc *Self,float VoltageV,uint32_t NowMs)
{
    if(Self==NULL || !Self->Configured)return;
    if(!isfinite(VoltageV) || VoltageV<=0) {Self->Valid=0;return;}
    uint32_t Elapsed=NowMs-Self->TickMs;
    if(!Self->Valid || Elapsed>=Self->Config.StaleMs)
        Self->FilteredVoltageV=VoltageV;
    else
    {
        // Time-based low-pass filter; repeat timestamps do not advance it.
        float Alpha=(float)Elapsed/((float)Self->Config.FilterTimeMs+(float)Elapsed);
        Self->FilteredVoltageV=(1-Alpha)*Self->FilteredVoltageV+Alpha*VoltageV;
    }
    Self->TickMs=NowMs;
    Self->Valid=1;
}
BatterySoc_Result BatterySoc_Get(const BatterySoc *Self,uint32_t NowMs)
{
    BatterySoc_Result Result={0};
    if(Self==NULL || !Self->Configured || !Self->Valid ||
        (uint32_t)(NowMs-Self->TickMs)>=Self->Config.StaleMs)return Result;
    const BatterySoc_Point *Curve=Self->Config.Curve;
    float CellV=Self->FilteredVoltageV/Self->Config.SeriesCells;
    float Percent=0;
    if(CellV>=Curve[Self->Config.PointCount-1].CellVoltageV)Percent=100;
    else if(CellV>Curve[0].CellVoltageV)
    {
        for(unsigned i=1;i<Self->Config.PointCount;i++)
        {
            if(CellV<=Curve[i].CellVoltageV)
            {
                float Fraction=(CellV-Curve[i-1].CellVoltageV)/
                    (Curve[i].CellVoltageV-Curve[i-1].CellVoltageV);
                Percent=Curve[i-1].Percent+Fraction*(Curve[i].Percent-Curve[i-1].Percent);
                break;
            }
        }
    }
    Result.Percent=Percent;
    Result.RemainingMah=Self->Config.CapacityMah*(Percent/100);
    Result.Valid=1;
    return Result;
}
