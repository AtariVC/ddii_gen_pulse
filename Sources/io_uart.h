#ifndef _IO_UART_H_
#define _IO_UART_H_



void UART1_Init(void);
int8_t UART1_Rx(uint8_t *bt);
void UART1_Tx(uint8_t bt);
uint16_t UART1_RxPacket(uint8_t *buff, uint16_t size);
void UART1_TxPacket(uint8_t *buff, uint16_t leng);



#endif



