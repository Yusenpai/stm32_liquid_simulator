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
#include "liqsim.h"
#include "flip_fluid.h"
#include "mpu6050.h"
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

DMA_NodeTypeDef Node_GPDMA1_Channel15;
DMA_QListTypeDef List_GPDMA1_Channel15;
DMA_HandleTypeDef handle_GPDMA1_Channel15;
DMA_QListTypeDef pQueueLinkList;
DMA_NodeTypeDef Node_GPDMA1_Channel14;
DMA_QListTypeDef List_GPDMA1_Channel14;
DMA_HandleTypeDef handle_GPDMA1_Channel14;

I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
uint16_t Liq_Display_Buffer[16] = {
    0x5555,
    0xaaaa,
    0x5555,
    0xaaaa,
    0x5555,
    0xaaaa,
    0x5555,
    0xaaaa,
    0x5555,
    0xaaaa,
    0x5555,
    0xaaaa,
    0x5555,
    0xaaaa,
    0x5555,
    0xaaaa,
};
extern uint16_t Liq_DMA_Buffer_ODR[];
extern uint32_t Liq_DMA_Buffer_MODER[];

static FlipFluid fluid;
static MPU6050_Typedef mpu6050;
static uint32_t App_Running_Flag = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
static void MX_ICACHE_Init(void);
/* USER CODE BEGIN PFP */
static void GPIO_Configure();
static void Start_DMA_Request();
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
  MX_GPDMA1_Init();
  MX_I2C4_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  /* Delay time for debug access*/
  HAL_Delay(3000);

  MPU6050_Init(&mpu6050, &hi2c4);
  Liq_Convert_Image_To_DMA(Liq_Display_Buffer);
  GPIO_Configure();
  Start_DMA_Request();
  Fluid_Init(&fluid);
  HAL_TIM_Base_Start_IT(&htim17);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (App_Running_Flag)
    {
      Fluid_Simulate(&fluid);
      Fluid_Draw(&fluid, Liq_Display_Buffer);
      Liq_Convert_Image_To_DMA(Liq_Display_Buffer);
      MPU6050_Read_Accel_IT(&mpu6050);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  DMA_NodeConfTypeDef NodeConfig = {0};

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  NodeConfig.NodeType = DMA_GPDMA_2D_NODE;
  NodeConfig.Init.Request = DMA_REQUEST_SW;
  NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  NodeConfig.Init.Direction = DMA_MEMORY_TO_MEMORY;
  NodeConfig.Init.SrcInc = DMA_SINC_INCREMENTED;
  NodeConfig.Init.DestInc = DMA_DINC_FIXED;
  NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  NodeConfig.Init.SrcBurstLength = 1;
  NodeConfig.Init.DestBurstLength = 1;
  NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  NodeConfig.Init.Mode = DMA_NORMAL;
  NodeConfig.RepeatBlockConfig.RepeatCount = 1;
  NodeConfig.RepeatBlockConfig.SrcAddrOffset = 0;
  NodeConfig.RepeatBlockConfig.DestAddrOffset = 0;
  NodeConfig.RepeatBlockConfig.BlkSrcAddrOffset = -(4 * 216 * 2);
  NodeConfig.RepeatBlockConfig.BlkDestAddrOffset = 0;
  NodeConfig.TriggerConfig.TriggerMode = DMA_TRIGM_SINGLE_BURST_TRANSFER;
  NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_RISING;
  NodeConfig.TriggerConfig.TriggerSelection = GPDMA1_TRIGGER_TIM15_TRGO;
  NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  NodeConfig.SrcAddress = (uint32_t)&Liq_DMA_Buffer_MODER;
  NodeConfig.DstAddress = (uint32_t)&(GPIOA->MODER);
  NodeConfig.DataSize = (4 * 216 * 2);
  if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel15) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel15, NULL, &Node_GPDMA1_Channel15) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel15) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel15.Instance = GPDMA1_Channel15;
  handle_GPDMA1_Channel15.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  handle_GPDMA1_Channel15.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel15.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  handle_GPDMA1_Channel15.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel15.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel15) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel15, &List_GPDMA1_Channel15) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel15, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
  NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
  NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1|DMA_DEST_ALLOCATED_PORT1;
  NodeConfig.RepeatBlockConfig.BlkSrcAddrOffset = - (2 * 216 * 2);
  NodeConfig.SrcAddress = (uint32_t)&Liq_DMA_Buffer_ODR;
  NodeConfig.DstAddress = (uint32_t)&(GPIOA->ODR);
  NodeConfig.DataSize = (2 * 216 * 2);
  if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel14, NULL, &Node_GPDMA1_Channel14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel14) != HAL_OK)
  {
    Error_Handler();
  }
  handle_GPDMA1_Channel14.Instance = GPDMA1_Channel14;
  handle_GPDMA1_Channel14.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  handle_GPDMA1_Channel14.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel14.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  handle_GPDMA1_Channel14.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel14.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel14, &List_GPDMA1_Channel14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel14, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00F07BFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 739;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 9999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 266;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void GPIO_Configure()
{
  /* Configure the GPIOA */
  GPIOA->MODER = 0xFFFF;   // Analog mode
  GPIOA->OTYPER = 0x0000;  // Push-pull
  GPIOA->OSPEEDR = 0xFFFF; // Very high speed
  GPIOA->PUPDR = 0x0000;   // No pull-up, pull-down
  GPIOA->ODR = 0x0000;     // Low level
}

static void Start_DMA_Request()
{
  HAL_DMAEx_List_Start(&handle_GPDMA1_Channel14);
  HAL_DMAEx_List_Start(&handle_GPDMA1_Channel15);
  HAL_TIM_Base_Start(&htim15);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM17)
  {
    App_Running_Flag = 1;
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C4)
  {
    fluid.fluid_setting.gravity_x = -0.75f * (int16_t)(mpu6050.raw_data[2] << 8 | mpu6050.raw_data[3]);
    fluid.fluid_setting.gravity_y = -0.75f * (int16_t)(mpu6050.raw_data[0] << 8 | mpu6050.raw_data[1]);
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
