#include "dwt.h"

static uint32_t last_cycle = 0;

static uint32_t cpu_mhz;

void DWT_Init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	DWT->CYCCNT = 0;
	last_cycle = 0;

	cpu_mhz = SystemCoreClock / 1000000U;
}

uint32_t DWT_GetMicros(void) {
	static uint32_t last_cyccnt = 0;
	static uint64_t total_cycles = 0;

	uint32_t current_cyccnt = DWT->CYCCNT;

	// Phép trừ này tự động xử lý mượt mà khi thanh ghi 32-bit bị tràn
	uint32_t cycle_diff = current_cyccnt - last_cyccnt;

	// Tích lũy vào biến 64-bit để không bao giờ bị mất mốc thời gian
	total_cycles += cycle_diff;
	last_cyccnt = current_cyccnt;

	// Trả về số microgiây bằng cách chia tổng cycle cho tần số (chia sau khi tích lũy)
	return (uint32_t) (total_cycles / cpu_mhz);
}

float DWT_GetDeltaTime(void) {
	uint32_t now = DWT->CYCCNT;
	uint32_t cycles = now - last_cycle;
	last_cycle = now;

	return (float) cycles / (float) SystemCoreClock;
}

