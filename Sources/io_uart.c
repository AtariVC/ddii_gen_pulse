#include <stm32f405.h>
#include "io_uart.h"



#define UART_RX_BUFF_SIZE      512
#define UART_TX_BUFF_SIZE      512

uint8_t UART1RxBuff[UART_RX_BUFF_SIZE];
volatile uint16_t UART1RxBuffTail, UART1RxBuffHead;
uint8_t UART1TxBuff[UART_TX_BUFF_SIZE];
volatile uint16_t UART1TxBuffTail, UART1TxBuffHead;



void UART1_Init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
  TIM7->CR1 = TIM_CR1_OPM;  //Timer7: one pulse mode
  USART1->BRR = (8 << 4) | 12;   // 500000 @ 140 MHz
//  USART1->BRR = (38 << 4);   // 115200 @ 140 MHz
//  USART1->BRR = (54 << 4) | 4;  // 115200 @ 100 MHz
//  USART1->BRR = (162 << 4) | 12;  //38400 @ 100 MHz
//  USART1->BRR = (130 << 4) | 3;  //38400 @ 80 MHz
  TIM7->PSC = 7000-1;  // 0.1ms prescaler @ 140 MHz
//  TIM7->PSC = 5000-1;  // 0.1ms prescaler  @ 100 MHz
//  TIM7->PSC = 4000-1;  // 0.1ms prescaler  @ 80 MHz
  TIM7->ARR = 200-1;  // 20 ms timeout for RX
  UART1RxBuffTail = UART1RxBuffHead = UART1TxBuffTail = UART1TxBuffHead = 0;
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  USART1->CR1 |= USART_CR1_UE;
  NVIC_EnableIRQ(USART1_IRQn);
}



int8_t UART1_Rx(uint8_t *bt) {
  if(UART1RxBuffHead == UART1RxBuffTail)
    return 0;
  *bt = UART1RxBuff[UART1RxBuffTail++];
  UART1RxBuffTail &= (UART_RX_BUFF_SIZE-1);
  return 1;
}



uint16_t UART1_RxPacket(uint8_t *buff, uint16_t size) {
  int i;
  uint16_t rx_size, ret;
  TIM7->CNT = 0;
  TIM7->EGR = TIM_EGR_UG;
  TIM7->SR = 0;
  TIM7->CR1 |= TIM_CR1_CEN;
  while(1) {
    rx_size = (UART1RxBuffHead - UART1RxBuffTail) & (UART_RX_BUFF_SIZE-1);
    if(rx_size >= size) {
      for(i=0; i<size; i++) {
        buff[i] = UART1RxBuff[UART1RxBuffTail++];
        UART1RxBuffTail &= (UART_RX_BUFF_SIZE-1);
        }
      ret = size;
      break;
      }
    else {
      if(TIM7->SR & 1) {
        ret = 0;
        break;
        }
      }
    }
  TIM7->CR1 &= ~TIM_CR1_CEN;
  return ret;
}



void UART1_Tx(uint8_t bt) {
  UART1TxBuff[UART1TxBuffHead++] = bt;
  UART1TxBuffHead &= (UART_TX_BUFF_SIZE-1);
  USART1->CR1 |= USART_CR1_TXEIE;
}



void UART1_TxPacket(uint8_t *buff, uint16_t leng) {
  int i;
  for(i=0; i<leng; i++) {
    UART1TxBuff[UART1TxBuffHead++] = buff[i];
    UART1TxBuffHead &= (UART_TX_BUFF_SIZE-1);
    }
  if(leng != 0)
    USART1->CR1 |= USART_CR1_TXEIE;
}



void USART1_IRQHandler() {
  uint32_t sr;
  uint8_t bt;
  sr = USART1->SR;
  if(sr & USART_SR_RXNE) {  // RX interrupt
    bt = USART1->DR;
    if((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) == 0) {
      UART1RxBuff[UART1RxBuffHead++] = bt;
      UART1RxBuffHead &= (UART_RX_BUFF_SIZE-1);
      }
    }
  else if(sr & USART_SR_TXE) {  // TX interrupt
    if(UART1TxBuffHead != UART1TxBuffTail) {
      USART1->DR = UART1TxBuff[UART1TxBuffTail++];
      UART1TxBuffTail &= (UART_TX_BUFF_SIZE-1);
      }
    if(UART1TxBuffHead == UART1TxBuffTail) {
      USART1->CR1 &= ~USART_CR1_TXEIE;
      }
    }
  __DSB();__ISB();
}



/*--------------------------------------------------------------------------------------*/

