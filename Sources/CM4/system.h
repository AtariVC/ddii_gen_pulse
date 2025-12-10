#ifndef _SYSTEM_H_
#define _SYSTEM_H_


#define SYSTEM_CORE_CLOCK     140000000

#define US_TO_DWT(x) ((x) * (SYSTEM_CORE_CLOCK/1000000))
#define MS_TO_DWT(x) ((x) * (SYSTEM_CORE_CLOCK/1000))

#if defined  (HSE_VALUE)
/* Redefine the HSE value */
#undef HSE_VALUE
#define HSE_VALUE    ((uint32_t)8000000) 
#endif /* HSE_VALUE */

/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M      6
#define PLL_N      210
/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      2

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      7

#define FLASH_ACR_LATENCY_NWS   FLASH_ACR_LATENCY_5WS


#endif


