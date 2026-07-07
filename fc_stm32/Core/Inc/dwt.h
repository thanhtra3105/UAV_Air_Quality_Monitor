#ifndef __DWT_H
#define __DWT_H

#include "stm32h5xx_hal.h"

void DWT_Init(void);
float DWT_GetDeltaTime(void);
uint32_t DWT_GetMicros(void);

#endif
