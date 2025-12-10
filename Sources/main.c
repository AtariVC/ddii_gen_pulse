#include "stm32f405.h"
#include "dwt.h"
#include "io_uart.h"
#include "io_dac.h"
#define _OWNER_
#include "sw_defs.h"


/*Note:
  PA9  -  UART1_TX
  PA10 -  UART1_RX
  PA4  -  DAC_OUT1
  PA5  -  DAC_OUT2
  PB10..PB14 - DIN0..DIN4
*/



#define  CMD_GP_SET_PARAMS    1
#define  CMD_GP_GENERATE      2



void BSP_Init(void);



uint8_t Buff[256];



int main() {
  uint8_t c;
  BSP_Init();
  DWT_Init();
  UART1_Init();
  DAC_Init();

  SysTick_Config(140000000/1000);  // 1 ms interrupt period
  while(1) {
    if(UART1_Rx(&c)) {
      switch(c) {
        case  CMD_GP_SET_PARAMS:
          /*  in:   byte0 - cmd
                    byte1 - DAC1 level hi byte
                    byte2 - DAC1 level lo byte
                    byte3 - DAC2 level hi byte
                    byte4 - DAC2 level lo byte
                    byte5 - DIN[4:0]
                    byte6 - pulse width in us
              out:  byte0 - cmd echo
          */
          if(UART1_RxPacket(Buff, 6)) {
            DAC1_Level = ((uint16_t)Buff[0] << 8) | Buff[1];
            DAC2_Level = ((uint16_t)Buff[2] << 8) | Buff[3];
            DIN_Data = Buff[4];
            PulseWidth = Buff[5];
            UART1_Tx(c);
            }
          break;
        case  CMD_GP_GENERATE:
          /*  in:   byte0 - cmd
              out:  byte0 - cmd echo
          */
          DAC_Generate(DAC1_Level, DAC2_Level, DIN_Data, PulseWidth);
          UART1_Tx(c);
          break;
        }/* switch(c)*/
      }
  
    }/*while(1)*/  
}



void SysTick_Handler(void) {
}


