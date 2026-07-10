/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include "drive_bus_servo.h"
#include "sensor_ops.h"
#include "sensor_wit.h"
#include "drive_emm.h"
#include "advance_chassis.h"
#include "advance_motion.h"
#include "advance_world.h"
#include "comm_pc.h"
#include "comm_protocol.h"
#include "car_pose.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 原点未建立时的非阻塞重试间隔；OPS 有效前不会建立原点。 */
#define WORLD_ORIGIN_RETRY_MS ((uint32_t)1000U)
/* 发布固件保持 0，避免 printf 的逐字节阻塞影响控制周期。 */
#define DEBUG_UART_ENABLE (0U)

/* TIM6 仅置位这些任务，不在中断上下文执行业务逻辑。 */
#define APP_TASK_PROTOCOL ((uint32_t)0x00000001U)
#define APP_TASK_WORLD ((uint32_t)0x00000002U)
#define APP_TASK_SAFETY ((uint32_t)0x00000004U)
#define APP_TASK_MOTION ((uint32_t)0x00000008U)
#define APP_TASK_MOTOR ((uint32_t)0x00000010U)
#define APP_TASK_ORIGIN ((uint32_t)0x00000020U)
#define APP_TASK_LED ((uint32_t)0x00000040U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
static volatile uint32_t g_app_pending_tasks = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  (void)f;
#if (DEBUG_UART_ENABLE != 0U)
  uint8_t byte = (uint8_t)ch;
  (void)HAL_UART_Transmit(&huart1, &byte, 1, 20U);
#else
  (void)ch;
#endif
  return ch;
}

void testEmmV5Datou(uint8_t id)
{
  const uint16_t vel_rpm = 300U;
  const uint8_t acc = 10U;
  const uint32_t one_turn_pulse = 3200U;

  drive_emm_En_Control(id, true, false);
  HAL_Delay(100);

  drive_emm_Vel_Control(id, 0U, vel_rpm, acc, false);
  HAL_Delay(500);
  drive_emm_Stop_Now(id, false);
  HAL_Delay(500);

  drive_emm_Vel_Control(id, 1U, vel_rpm, acc, false);
  HAL_Delay(500);
  drive_emm_Stop_Now(id, false);
  HAL_Delay(500);

  drive_emm_Pos_Control(id, 0U, vel_rpm, acc, one_turn_pulse, false, false);
  HAL_Delay(500);

  drive_emm_Pos_Control(id, 1U, vel_rpm, acc, one_turn_pulse, false, false);
  HAL_Delay(500);

  drive_emm_Stop_Now(id, false);
}

void situation_led()
{
  // 状态指示灯：每 500ms 翻转一次 GPIO 状态
  // 如果烧录成功并正常运行，可以看到板载 LED 闪烁
  static uint32_t led_tick = 0;
  if (HAL_GetTick() - led_tick >= 500)
  {
    led_tick = HAL_GetTick();
    // 翻转 PF9 红色 LED 和 PF10 绿色 LED
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
  }
}

void test() // 测试的东西全写在里面
{
  // 测试 EMM V5 大头电机
  testEmmV5Datou(CHASSIS_MOTOR_LF_ID);
  testEmmV5Datou(CHASSIS_MOTOR_RF_ID);
  testEmmV5Datou(CHASSIS_MOTOR_LR_ID);
  testEmmV5Datou(CHASSIS_MOTOR_RR_ID);
}

static uint32_t App_TakePendingTasks(void)
{
  uint32_t pending;

  __disable_irq();
  pending = g_app_pending_tasks;
  g_app_pending_tasks = 0U;
  __enable_irq();
  return pending;
}

static void App_TryResetWorldOrigin(void)
{
  if (AdvanceWorld_GetPose()->origin_ready == 0U)
  {
    (void)AdvanceWorld_ResetOrigin();
  }
}

static void App_SafetyCheck(void)
{
  if ((Chassis_IsMotionCommandActive() != 0U) &&
      (drive_emm_IsChassisFeedbackHealthy() == 0U))
  {
    AdvanceMotion_CancelIfActive();
    Chassis_Stop();
  }
}

static void App_RunScheduledTasks(uint32_t pending)
{
  if ((pending & APP_TASK_PROTOCOL) != 0U)
  {
    HostRx_Poll();
  }

  if ((pending & APP_TASK_WORLD) != 0U)
  {
    OPS_Poll();
    WIT_Poll();
    AdvanceWorld_Poll();
  }

  if ((pending & APP_TASK_MOTOR) != 0U)
  {
    drive_emm_Poll();
  }

  if ((pending & APP_TASK_ORIGIN) != 0U)
  {
    App_TryResetWorldOrigin();
  }

  if ((pending & APP_TASK_SAFETY) != 0U)
  {
    App_SafetyCheck();
  }

  if ((pending & APP_TASK_MOTION) != 0U)
  {
    AdvanceMotion_Poll();
  }

  if ((pending & APP_TASK_LED) != 0U)
  {
    situation_led();
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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // 传感器初始化
  OPS_Init(&huart5);
  WIT_Init();

  /* 上层模块先绑定传感器数据视图，再初始化自身状态。 */
  CarPose_Init();
  AdvanceWorld_Init();
  AdvanceMotion_Init();
  (void)drive_emm_Init();
  drive_emm_ConfigureChassisFeedback(
      CHASSIS_MOTOR_LF_ID,
      CHASSIS_MOTOR_RF_ID,
      CHASSIS_MOTOR_LR_ID,
      CHASSIS_MOTOR_RR_ID);

  // 外设初始化
  BusServo_Init(&huart4);

  // 通信初始化
  if (HostRx_InitPc(&huart1) != comm_pc_STATUS_OK)
  {
    printf("HostRx PC init failed\r\n");
  }

  if (HostRx_InitJetson(&huart6) != comm_pc_STATUS_OK)
  {
    printf("HostRx Jetson init failed\r\n");
  }

  /* 原点建立由 1 s 调度任务重试，不阻塞等待 OPS 数据。 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  // 测试函数
  // test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t pending = App_TakePendingTasks();
    if (pending == 0U)
    {
      __WFI();
    }
    else
    {
      App_RunScheduledTasks(pending);
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF9 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    BusServo_OnByteReceived();
  }

  if (huart->Instance == UART5)
  {
    OPS_OnByteReceived();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    WIT_OnUartRxEvent(huart, Size);
  }

  if (huart->Instance == USART3)
  {
    drive_emm_OnUartRxEvent(huart, Size);
  }

  if (huart->Instance == USART1)
  {
    HostRx_OnUartRxEvent(huart, Size);
  }

  if (huart->Instance == USART6)
  {
    HostRx_OnUartRxEvent(huart, Size);
  }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    drive_emm_OnTxComplete(huart);
  }

  if ((huart->Instance == USART1) || (huart->Instance == USART6))
  {
    HostProtocol_OnUartTxComplete(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    drive_emm_OnUartError(huart);
  }

  if ((huart->Instance == USART1) || (huart->Instance == USART6))
  {
    HostProtocol_OnUartError(huart);
  }

  if (huart->Instance == UART4)
  {
    BusServo_OnUartError();
  }

  if (huart->Instance == UART5)
  {
    OPS_OnUartError();
  }

  if (huart->Instance == USART2)
  {
    WIT_OnUartError(huart);
  }

  if (huart->Instance == USART6)
  {
    HostRx_OnUartError(huart);
  }

  if (huart->Instance == USART1)
  {
    HostRx_OnUartError(huart);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t protocol_tick = 0U;
  static uint16_t world_tick = 0U;
  static uint16_t safety_tick = 0U;
  static uint16_t motion_tick = 0U;
  static uint16_t motor_tick = 0U;
  static uint16_t origin_tick = 0U;
  static uint16_t led_tick = 0U;

  if (htim->Instance != TIM6)
  {
    return;
  }

  if (++protocol_tick >= 2U)
  {
    protocol_tick = 0U;
    g_app_pending_tasks |= APP_TASK_PROTOCOL;
  }
  if (++world_tick >= 10U)
  {
    world_tick = 0U;
    g_app_pending_tasks |= APP_TASK_WORLD;
  }
  if (++safety_tick >= 10U)
  {
    safety_tick = 0U;
    g_app_pending_tasks |= APP_TASK_SAFETY;
  }
  if (++motion_tick >= 20U)
  {
    motion_tick = 0U;
    g_app_pending_tasks |= APP_TASK_MOTION;
  }
  if (++motor_tick >= 20U)
  {
    motor_tick = 0U;
    g_app_pending_tasks |= APP_TASK_MOTOR;
  }
  if (++origin_tick >= WORLD_ORIGIN_RETRY_MS)
  {
    origin_tick = 0U;
    g_app_pending_tasks |= APP_TASK_ORIGIN;
  }
  if (++led_tick >= 500U)
  {
    led_tick = 0U;
    g_app_pending_tasks |= APP_TASK_LED;
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
#ifdef USE_FULL_ASSERT
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
