#include "dwt.h"

static uint32_t last_cycle = 0;

static uint32_t cpu_mhz;

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    DWT->CYCCNT = 0;
    last_cycle = 0;

    cpu_mhz = SystemCoreClock / 1000000U;
}

uint32_t DWT_GetMicros(void)
{
    return DWT->CYCCNT / cpu_mhz;
}

float DWT_GetDeltaTime(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t cycles = now - last_cycle;
    last_cycle = now;

    return (float)cycles / (float)SystemCoreClock;
}

