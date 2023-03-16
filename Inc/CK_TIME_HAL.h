
#ifndef CK_TIME_HAL_H_
#define CK_TIME_HAL_H_

#include "stm32f4xx.h"


// time difference, 32 bits always sufficient
typedef int32_t timeDelta_t;
// millisecond time
typedef uint32_t timeMs_t ;
// microsecond time
typedef uint32_t timeUs_t;
#define TIMEUS_MAX UINT32_MAX

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }

void CK_TIME_SetTimeOut(uint32_t time);

uint32_t CK_TIME_GetTimeOut(void);

void HAL_IncTick(void);

uint32_t HAL_GetTick(void);

uint32_t CK_TIME_GetMicroSec_SYSTICK(void);

uint32_t CK_TIME_GetMilliSec_SYSTICK(void);

uint32_t CK_TIME_GetMicroSec_DWT(void);

uint32_t CK_TIME_GetMilliSec_DWT(void);

uint32_t CK_TIME_GetMicroSec(void);

uint32_t CK_TIME_GetMilliSec(void);

void CK_TIME_DelayMilliSec(uint32_t msec);

void CK_TIME_DelayMicroSec(uint32_t usec);

#endif /* CK_TIME_HAL_H_ */
