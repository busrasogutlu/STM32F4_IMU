
#ifndef CK_SYSTEM_H_
#define CK_SYSTEM_H_

#include "stm32f4xx.h"

extern uint32_t F_CPU; 											    // CK_TIME_HAL uses

typedef enum{

	SYSTEM_CLK_180MHz,

	SYSTEM_CLK_168MHz

}systemClock_e;

void CK_SYSTEM_SetSystemClock(systemClock_e clk);

uint32_t CK_SYSTEM_GetAPB1Clock(void);

uint32_t CK_SYSTEM_GetAPB2Clock(void);

uint32_t CK_SYSTEM_GetSystemClock(void);

void CK_SYSTEM_TIMER_ClockEnable(TIM_TypeDef* num);

uint32_t CK_SYSTEM_GetTIMERClock(TIM_TypeDef* timer);

uint32_t CK_SYSTEM_GetUARTClock(USART_TypeDef* uart);

#endif
