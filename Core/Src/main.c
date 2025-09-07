/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
#define DELAY_US(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)

/* UART RX handling */
#define RXBUFFERSIZE  64
char RxBuffer[RXBUFFERSIZE];
uint8_t aRxBuffer;
uint8_t Uart1_Rx_Cnt = 0;
uint8_t cmd[3];
uint8_t query_flag = 0;

/* Command preemption: set to 1 by RX ISR when a normal command arrives */
volatile uint8_t new_cmd_ready = 0;

uint8_t state = 1;
uint8_t flag = 0;       /* 1 while executing a command */
uint32_t current_mm = 0; /* position in 0.01 mm units (1mm = 100) */

/* Accumulate pulses not equal to a full 0.01mm (32 pulses = 1 unit) */
static volatile int32_t pulse_accum = 0;

#define TXBUFFERSIZE  7
char TxBuffer[TXBUFFERSIZE] = {0xff, 0x0, 0x0, 0x0, 0x0, 0x0d, 0x0a};

#define WARNING_LED()        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define STATUS_LED()         HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)

#define CLOCKWISE            0
#define ANTI_CLOCKWISE       1

#define MOTOR_EN_ON()        HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET)  /* motor shut down (driver EN high) */
#define MOTOR_EN_OFF()       HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET)
#define MOTOR_PUL_ON()       HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_SET)
#define MOTOR_PUL_OFF()      HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET)
#define MOTOR_DIR_ON()       HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET)
#define MOTOR_DIR_OFF()      HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET)

/* stepper_turn return codes */
#define TURN_DONE            1u
#define TURN_ABORTED         2u
#define TURN_LIMIT_REACHED   3u

/* Perform angle turn at timing 'tim' us per pulse edge; supports preemption.
   Updates current_mm in 0.01 mm units based on actual pulses emitted.
   Returns:
     TURN_DONE          - requested angle completed
     TURN_ABORTED       - aborted due to new_cmd_ready
     TURN_LIMIT_REACHED - limit/home switch condition met (state=0) */
uint8_t stepper_turn(int tim, float angle, uint8_t dir)
{
    float subdivide = 32.0f; /* usteps per 1.8° */
    int n = (int)(angle / (1.8f / subdivide)); /* pulses to emit for requested angle */

    if (dir == CLOCKWISE)
        MOTOR_DIR_ON();
    else /* ANTI_CLOCKWISE */
        MOTOR_DIR_OFF();

    int pulses_sent = 0;
    uint8_t ret = TURN_DONE;

    for (int i = 0; i < n; i++)
    {
        /* Preempt as soon as a new command is ready */
        if (new_cmd_ready) { ret = TURN_ABORTED; break; }

        /* Limit switch handling (home at CLOCKWISE direction) */
        if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_SET && dir == CLOCKWISE)
        {
            MOTOR_EN_ON();  /* disable motor driver */
            state = 0;      /* at home/end */
            ret = TURN_LIMIT_REACHED;
            break;
        }
        else if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET || dir == ANTI_CLOCKWISE)
        {
            MOTOR_EN_OFF();
            state = 1;
        }

        /* One step pulse */
        MOTOR_PUL_OFF();
        DELAY_US(tim/2);
        MOTOR_PUL_ON();
        DELAY_US(tim/2);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

        pulses_sent++;
    }

    /* Update position based on emitted pulses (32 pulses = 0.01 mm = 1 unit) */
    if (state == 0)  /* reached home */
    {
        current_mm = 0;
        pulse_accum = 0;
    }
    else
    {
        if (dir == ANTI_CLOCKWISE)
            pulse_accum += pulses_sent;
        else
            pulse_accum -= pulses_sent;

        while (pulse_accum >= 32) { current_mm += 1; pulse_accum -= 32; }
        while (pulse_accum <= -32) { current_mm -= 1; pulse_accum += 32; }
    }

    return ret;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MOTOR_EN_OFF();

  uint8_t direction = CLOCKWISE;
  uint8_t circ = 0, vel = 0;
  int time_per_step = 0;
  int32_t steps_remaining = 0; /* number of 0.5 mm (90°) chunks to execute */

  /* Reset needle position (home), but allow preemption by new command */
  while (state)
  {
      if (new_cmd_ready) break; /* preempt homing if command arrives */
      (void)stepper_turn(20, 30, CLOCKWISE);
      current_mm = 0;
  }

  while (1)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

    /* If a new command arrived, accept it and preempt any current command */
    if (new_cmd_ready)
    {
        /* cmd[0]=dir, cmd[1]=length, cmd[2]=velocity */
        if (cmd[1] != 0 && cmd[2] != 0)
        {
            flag = 1; /* busy */
            direction = (cmd[0] == 1) ? ANTI_CLOCKWISE : CLOCKWISE;

            circ = cmd[1] / 2;       /* 360deg -> 2mm (keep original mapping) */
            vel = cmd[2];            /* mm/s */
            time_per_step = (int)(0.5f / vel * 1000.0f);  /* original formula kept */

            steps_remaining = 4 * circ; /* number of 0.5mm (90°) segments */

            /* Start immediately; clear the ready flag so we know we've taken it */
            new_cmd_ready = 0;
        }
        else
        {
            /* Invalid command (length/vel zero): ignore */
            new_cmd_ready = 0;
        }
    }

    if (flag && steps_remaining > 0)
    {
        /* Execute one 0.5mm chunk (90°). This call is preemptible. */
        uint8_t r = stepper_turn(time_per_step, 90, direction);

        if (r == TURN_DONE)
        {
            steps_remaining--;
            if (steps_remaining <= 0)
            {
                flag = 0;
                memset(cmd, 0, 3);
            }
        }
        else if (r == TURN_ABORTED)
        {
            /* New command arrived: stop current, main loop will pick it up next iteration */
            flag = 0;
            /* Do not clear cmd/new_cmd_ready here; they hold the new command */
        }
        else if (r == TURN_LIMIT_REACHED)
        {
            /* Hit home/limit; stop current command */
            flag = 0;
            steps_remaining = 0;
            memset(cmd, 0, 3);
        }

        /* After handling one chunk (or abort), continue loop */
        continue;
    }

    /* No active command: allow manual jogging, but still preemptible */
    while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
        if (new_cmd_ready) break; /* preempt manual move */
        uint8_t r = stepper_turn(45, 90, ANTI_CLOCKWISE);
        if (r != TURN_DONE) break; /* limit or abort */
    }
    while (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
        if (new_cmd_ready) break; /* preempt manual move */
        uint8_t r = stepper_turn(45, 90, CLOCKWISE);
        if (r != TURN_DONE) break; /* limit or abort */
    }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 256000;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_Pin|DIR_Pin|PUL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY3_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin DIR_Pin PUL_Pin */
  GPIO_InitStruct.Pin = EN_Pin|DIR_Pin|PUL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */

  if(Uart1_Rx_Cnt >= 255)
  {
    Uart1_Rx_Cnt = 0;
    memset(RxBuffer,0x00,sizeof(RxBuffer));
  }
  else
  {
    RxBuffer[Uart1_Rx_Cnt++] = aRxBuffer;   /* save */

    if((RxBuffer[Uart1_Rx_Cnt-1] == 0x0A)&&(RxBuffer[Uart1_Rx_Cnt-2] == 0x0D))  /* detect the end sign 0x0D0A */
    {
      /*HAL_UART_Transmit(&huart1, (uint8_t *)&RxBuffer, Uart1_Rx_Cnt,0xFFFF);*/

      if (RxBuffer[1] == (char)0xaa && RxBuffer[3] == (char)0xaa && RxBuffer[4] == (char)0xaa && RxBuffer[6] == (char)0xaa && RxBuffer[7] == (char)0xaa)
      {
        query_flag = 1;
      }
      else
      {
        /* Normal command: parse and mark as ready (this will preempt motion) */
        cmd[0] = RxBuffer[1];
        cmd[1] = (uint8_t)(RxBuffer[3]*10 + RxBuffer[4]);
        cmd[2] = (uint8_t)(RxBuffer[6]*10 + RxBuffer[7]);
        new_cmd_ready = 1; /* signal preemption */
      }
      Uart1_Rx_Cnt = 0;
      memset(RxBuffer,0x00,sizeof(RxBuffer));
    }
  }

  HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim2))  /* 5 ms 200 hz */
    {
      if(query_flag)
      {
        TxBuffer[1] = state;
        TxBuffer[2] = flag;
        TxBuffer[3] = (uint8_t) (current_mm / 100);
        TxBuffer[4] = (uint8_t) (current_mm % 100);
        HAL_UART_Transmit(&huart1, (uint8_t *)&TxBuffer, TXBUFFERSIZE, 0xFF);
        query_flag = 0;
      }
    }
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

