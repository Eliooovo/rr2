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
#include "lift_app.h"
#include "rs_motor.h"
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
/* KFS 根部旋转测试：RS03 电机 CSP 位置控制（限速 ~45°/s）。 */
static rs_motor_t s_rs03_rotate_motor;
static volatile rs_motor_status_t s_rs03_rotate_init_status = RS_MOTOR_ERROR_NOT_INITIALIZED;
/* 用户在 Ozone 里改这个值测试不同目标角度，单位 rad。
   例：0.0 = 0°, 1.571 = 90°, -1.571 = -90°。 */
static volatile float g_rs03_target_rad = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- KFS 根部旋转测试：RS03 CSP 位置控制 ---- */

static void Rs03RotateTest_Init(void)
{
  s_rs03_rotate_motor.config.hfdcan = &hfdcan3;
  s_rs03_rotate_motor.config.motor_id = 2U;
  s_rs03_rotate_motor.config.master_id = 0xFDU;
  s_rs03_rotate_motor.config.motor_type = RS_MOTOR_TYPE_3;
  s_rs03_rotate_motor.config.offline_timeout_ms = 100U;

  s_rs03_rotate_init_status = rs_motor_init(&s_rs03_rotate_motor);
}

static void Rs03RotateTest_RunPeriodic(void)
{
  static uint32_t last_ctrl_ms = 0U;
  static float last_target_rad = 0.0f;
  uint32_t now_ms = HAL_GetTick();

  if (s_rs03_rotate_init_status != RS_MOTOR_OK) {
    return;
  }

  /* 1. 读反馈、更新可观察变量、检测离线。 */
  {
    static uint32_t last_fb_count = 0U;
    rs_motor_feedback_t fb;

    if (rs_motor_get_feedback(&s_rs03_rotate_motor, &fb) == RS_MOTOR_OK) {
      /*
       * 将角度存到 static 变量，保证 Ozone 中始终可见，不会被优化掉。
       * 即使 rs_motor_get_feedback 尚未被调用，这些变量也持有上一次的有效值。
       */
      static float g_rs03_angle_rad = 0.0f;
      static float g_rs03_angle_deg = 0.0f;
      static uint32_t g_rs03_fb_count = 0U;

      g_rs03_angle_rad = fb.angle_rad;
      g_rs03_angle_deg = fb.angle_rad * 57.29578f;
      g_rs03_fb_count = s_rs03_rotate_motor.state.feedback_count;

      /* 反馈计数不涨 → 掉线，重置状态等下次 CSP 调用时自动重新使能。 */
      if (s_rs03_rotate_motor.state.feedback_count == last_fb_count) {
        s_rs03_rotate_motor.internal.mode_applied = 0U;
        s_rs03_rotate_motor.state.enabled = 0U;
      }
      last_fb_count = s_rs03_rotate_motor.state.feedback_count;
    }
  }

  /* 2. 按周期检查是否需要下发 CSP 位置控制帧。 */
  if ((uint32_t)(now_ms - last_ctrl_ms) < 10U) {
    return;
  }
  last_ctrl_ms = now_ms;

  /*
   * 目标不变且电机正常运行中 → 跳过，减少 CAN 总线负载。
   * 例外：刚上电 (enabled=0) 或离线恢复 (mode_applied=0) 时必须下发。
   */
  if (g_rs03_target_rad == last_target_rad &&
      s_rs03_rotate_motor.state.enabled != 0U &&
      s_rs03_rotate_motor.internal.mode_applied != 0U) {
    return;
  }
  last_target_rad = g_rs03_target_rad;

  /*
   * CSP（Cyclic Synchronous Position）位置控制：
   *   speed_limit_rad_s = 0.785 rad/s ≈ 45°/s
   *   电机内部以不超过限速的速度平滑移动到目标位置，到位后自动保持。
   */
  (void)rs_motor_csp_position_control(
      &s_rs03_rotate_motor,
      0.785f,               /* 速度上限，约 45 deg/s */
      g_rs03_target_rad);   /* 目标角度，单圈范围 [-12.57, +12.57] rad */
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  ChassisApp_Init();
  LiftApp_Init();
  //KfsLiftApp_Init();
  //GripperApp_Init();
  Rs03RotateTest_Init();
  bsp_can_init();
  CommApp_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CommApp_RunPeriodic();
    ChassisApp_RunPeriodic();
    LiftApp_RunPeriodic();
    //KfsLiftApp_RunPeriodic();
    //GripperApp_RunPeriodic();
    Rs03RotateTest_RunPeriodic();
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
