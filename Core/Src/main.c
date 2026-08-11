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
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "chassis_app.h"
#include "comm_app.h"
#include "gripper_app.h"
#include "kfs_lift_app.h"
#include "kfs_rotate_app.h"
#include "kfs_grip_app.h"
#include "lift_app.h"
#include "weapon_rotate_app.h"
#include "weapon_grip_app.h"
#include "tof200c.h"
#include "rc_control.h"
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

/* USER CODE BEGIN PV */
tof200c_t g_tof200c = {
  .config = {
    .hi2c = &hi2c2,
    .xshut_port = TOF_XSHUT_GPIO_Port,
    .xshut_pin = TOF_XSHUT_Pin,
    .int_port = TOF_INT_GPIO_Port,
    .int_pin = TOF_INT_Pin,
    .i2c_address_7bit = TOF200C_DEFAULT_I2C_ADDRESS_7BIT,
    .profile = TOF200C_PROFILE_HIGH_ACCURACY,
    .stale_timeout_ms = 100U,
  },
};

volatile tof200c_status_t g_tof200c_init_status =
    TOF200C_STATUS_NOT_INITIALIZED;
volatile tof200c_status_t g_tof200c_read_status =
    TOF200C_STATUS_NOT_INITIALIZED;
volatile tof200c_feedback_t g_tof200c_latest;
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_USART1_UART_Init();
  MX_USART10_UART_Init();
  MX_USB_DEVICE_Init();
  MX_UART7_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  ChassisApp_Init();
  LiftApp_Init();
  KfsLiftApp_Init();
  GripperApp_Init();
  KfsRotateApp_Init();
  KfsGripApp_Init();
  WeaponRotateApp_Init();
  bsp_can_init();
  CommApp_Init();
  WeaponGripApp_Init();
  g_tof200c_init_status = tof200c_init(&g_tof200c);
  RcControl_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);  // 翻转 LED2
    HAL_Delay(500);  // 等 500ms
    tof200c_feedback_t latest;
    static uint32_t s_last_tof_ms = 0;

    CommApp_RunPeriodic();
    RcControl_RunPeriodic();
    ChassisApp_RunPeriodic();
    LiftApp_RunPeriodic();
    KfsLiftApp_RunPeriodic();
    GripperApp_RunPeriodic();
    KfsRotateApp_RunPeriodic();
    KfsGripApp_RunPeriodic();
    WeaponRotateApp_RunPeriodic();
    WeaponGripApp_RunPeriodic();

    /* TOF200C: 限速 20ms 处理一次，传感器仅 ~5 Hz 出数。移到 LiftApp 之后
       避免阻塞恢复时影响抬升/底盘 PID 时序。 */
    {
      uint32_t now = HAL_GetTick();
      if ((uint32_t)(now - s_last_tof_ms) >= 20U) {
        tof200c_process(&g_tof200c);
        s_last_tof_ms = now;
      }
    }
    g_tof200c_read_status = tof200c_get_latest(&g_tof200c, &latest);
    if ((g_tof200c_read_status == TOF200C_STATUS_OK) ||
        (g_tof200c_read_status == TOF200C_STATUS_STALE_DATA))
    {
      g_tof200c_latest = latest;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  tof200c_on_exti_callback(&g_tof200c, GPIO_Pin);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  tof200c_on_i2c_mem_rx_complete(&g_tof200c, hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  tof200c_on_i2c_mem_tx_complete(&g_tof200c, hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  tof200c_on_i2c_error(&g_tof200c, hi2c);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
