/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define  CMD_GP_SET_PARAMS    1
#define  CMD_GP_GENERATE      2
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint16_t DAC1_Level;
static uint16_t DAC2_Level;
static uint8_t DIN_Data;
static uint8_t PulseWidth;
static uint32_t dwt_cycles_per_us;

typedef enum
{
  UART_RX_WAIT_CMD = 0,
  UART_RX_WAIT_PARAMS
} UartRxState;

typedef enum
{
  DAC_PULSE_IDLE = 0,
  DAC_PULSE_HIGH,
  DAC_PULSE_LOW
} DacPulseState;

typedef struct
{
  uint16_t dac1;
  uint16_t dac2;
  uint8_t din;
  uint8_t width_us;
} DacPulseParams;

static volatile UartRxState uart_rx_state;
static uint8_t uart_rx_cmd;
static uint8_t uart_rx_params[6];

static volatile uint8_t uart_tx_busy;
static volatile uint8_t uart_tx_pending;
static uint8_t uart_tx_byte;
static uint8_t uart_tx_pending_byte;

static volatile DacPulseState dac_pulse_state;
static uint32_t dac_deadline_cycles;
static volatile uint8_t dac_pulse_pending;
static volatile DacPulseParams dac_pulse_pending_params;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static void UART_RxStart(uint8_t *buf, uint16_t len);
static void UART_SendAck(uint8_t value);
static void DAC_Pulse_Request(uint16_t dac1, uint16_t dac2, uint8_t din, uint8_t width_time_us);
static void DAC_Pulse_Service(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DWT_Init(void)
{
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  dwt_cycles_per_us = SystemCoreClock / 1000000U;
  if (dwt_cycles_per_us == 0U)
  {
    dwt_cycles_per_us = 1U;
  }
}

static uint8_t DWT_DeadlinePassed(uint32_t deadline)
{
  return ((int32_t)(DWT->CYCCNT - deadline)) >= 0;
}

static void UART_RxStart(uint8_t *buf, uint16_t len)
{
  (void)HAL_UART_Receive_IT(&huart1, buf, len);
}

static void UART_SendAck(uint8_t value)
{
  if (uart_tx_busy != 0U)
  {
    uart_tx_pending = 1U;
    uart_tx_pending_byte = value;
    return;
  }

  uart_tx_busy = 1U;
  uart_tx_byte = value;
  if (HAL_UART_Transmit_IT(&huart1, &uart_tx_byte, 1) != HAL_OK)
  {
    uart_tx_busy = 0U;
  }
}

static void DAC_Pulse_Request(uint16_t dac1, uint16_t dac2, uint8_t din, uint8_t width_time_us)
{
  dac_pulse_pending_params.dac1 = dac1;
  dac_pulse_pending_params.dac2 = dac2;
  dac_pulse_pending_params.din = din;
  dac_pulse_pending_params.width_us = width_time_us;
  dac_pulse_pending = 1U;
}

static void DAC_Pulse_Start(const DacPulseParams *params)
{
  const uint16_t dac1 = params->dac1 & 0x0FFFu;
  const uint16_t dac2 = params->dac2 & 0x0FFFu;

  (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac1);
  (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac2);

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  GPIOB->BSRR = ((uint32_t)(params->din & 0x1Fu)) << 10;
  hdac.Instance->SWTRIGR = DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;
  if (primask == 0U)
  {
    __enable_irq();
  }

  dac_deadline_cycles = DWT->CYCCNT + ((uint32_t)params->width_us * dwt_cycles_per_us);
  dac_pulse_state = DAC_PULSE_HIGH;
}

static void DAC_Pulse_Service(void)
{
  switch (dac_pulse_state)
  {
    case DAC_PULSE_IDLE:
      if (dac_pulse_pending != 0U)
      {
        DacPulseParams params;
        params.dac1 = dac_pulse_pending_params.dac1;
        params.dac2 = dac_pulse_pending_params.dac2;
        params.din = dac_pulse_pending_params.din;
        params.width_us = dac_pulse_pending_params.width_us;
        dac_pulse_pending = 0U;
        DAC_Pulse_Start(&params);
      }
      break;
    case DAC_PULSE_HIGH:
      if (DWT_DeadlinePassed(dac_deadline_cycles))
      {
        (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0U);
        (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0U);

        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        hdac.Instance->SWTRIGR = DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;
        if (primask == 0U)
        {
          __enable_irq();
        }

        dac_deadline_cycles = DWT->CYCCNT + dwt_cycles_per_us;
        dac_pulse_state = DAC_PULSE_LOW;
      }
      break;
    case DAC_PULSE_LOW:
      if (DWT_DeadlinePassed(dac_deadline_cycles))
      {
        GPIOB->BSRR = ((uint32_t)0x1Fu) << (10 + 16);
        dac_pulse_state = DAC_PULSE_IDLE;
      }
      break;
    default:
      dac_pulse_state = DAC_PULSE_IDLE;
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  uart_rx_state = UART_RX_WAIT_CMD;
  uart_tx_busy = 0U;
  uart_tx_pending = 0U;
  dac_pulse_state = DAC_PULSE_IDLE;
  dac_pulse_pending = 0U;
  UART_RxStart(&uart_rx_cmd, 1);

  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0U);
  (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0U);
  hdac.Instance->SWTRIGR = DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    DAC_Pulse_Service();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  if (uart_rx_state == UART_RX_WAIT_CMD)
  {
    const uint8_t cmd = uart_rx_cmd;
    switch (cmd)
    {
      case CMD_GP_SET_PARAMS:
        uart_rx_state = UART_RX_WAIT_PARAMS;
        UART_RxStart(uart_rx_params, sizeof(uart_rx_params));
        break;
      case CMD_GP_GENERATE:
        DAC_Pulse_Request(DAC1_Level, DAC2_Level, DIN_Data, PulseWidth);
        UART_SendAck(cmd);
        uart_rx_state = UART_RX_WAIT_CMD;
        UART_RxStart(&uart_rx_cmd, 1);
        break;
      default:
        uart_rx_state = UART_RX_WAIT_CMD;
        UART_RxStart(&uart_rx_cmd, 1);
        break;
    }
  }
  else if (uart_rx_state == UART_RX_WAIT_PARAMS)
  {
    DAC1_Level = ((uint16_t)uart_rx_params[0] << 8) | uart_rx_params[1];
    DAC2_Level = ((uint16_t)uart_rx_params[2] << 8) | uart_rx_params[3];
    DIN_Data = uart_rx_params[4];
    PulseWidth = uart_rx_params[5];
    UART_SendAck(CMD_GP_SET_PARAMS);
    uart_rx_state = UART_RX_WAIT_CMD;
    UART_RxStart(&uart_rx_cmd, 1);
  }
  else
  {
    uart_rx_state = UART_RX_WAIT_CMD;
    UART_RxStart(&uart_rx_cmd, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  if (uart_tx_pending != 0U)
  {
    uart_tx_pending = 0U;
    uart_tx_byte = uart_tx_pending_byte;
    (void)HAL_UART_Transmit_IT(&huart1, &uart_tx_byte, 1);
    return;
  }

  uart_tx_busy = 0U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  uart_rx_state = UART_RX_WAIT_CMD;
  UART_RxStart(&uart_rx_cmd, 1);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
