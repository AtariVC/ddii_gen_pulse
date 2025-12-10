#include <stm32f405.h>
#include "dwt.h"
#include "io_dac.h"



void DAC_Init() {
  RCC->APB1RSTR |= RCC_APB1RSTR_DACRST;
  Delay_us(1);
  RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  DAC->CR = (7<<19) | DAC_CR_TEN2 | (7<<3) | DAC_CR_TEN1 | DAC_CR_BOFF2 | DAC_CR_BOFF1;
  DAC->CR |= DAC_CR_EN2 | DAC_CR_EN1;
}



void DAC_Generate(uint16_t dac1, uint16_t dac2, uint8_t din, uint8_t width_time_us) {
  DAC->DHR12R1 = dac1;
  DAC->DHR12R2 = dac2;
  __disable_irq();
  GPIOB->BSRRL = din << 10;
  DAC->SWTRIGR = 0x3;
  Delay_us(width_time_us);
  DAC->DHR12RD = 0;
  DAC->SWTRIGR = 0x3;
  Delay_us(1);
  GPIOB->BSRRH = 0x1F << 10;
  __enable_irq();
}


