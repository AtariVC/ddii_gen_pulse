#ifndef _DWT_H_
#define _DWT_H_

#include "./CM4/system.h"


#define US_TO_DWT(x) ((x) * (SYSTEM_CORE_CLOCK/1000000))
#define MS_TO_DWT(x) ((x) * (SYSTEM_CORE_CLOCK/1000))



static inline void DWT_Init(void) {
  if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}



static inline void Delay_tc(uint32_t tick) {
  uint32_t time0 = DWT->CYCCNT;
  while((DWT->CYCCNT - time0) < tick );
}



static inline void Delay_ms(uint32_t time) {
  uint32_t time0 = DWT->CYCCNT;
  while((DWT->CYCCNT - time0) < MS_TO_DWT(time));
}



static inline void Delay_us(uint32_t time) {
  uint32_t time0 = DWT->CYCCNT;
  while((DWT->CYCCNT - time0) < US_TO_DWT(time));
}



static inline uint32_t  GetDWTCyccnt(void) {
  return DWT->CYCCNT;
}




#endif

