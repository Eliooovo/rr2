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
#include "chassis.h"
#include "comm_protocol.h"
#include "dji_motor.h"
#include "lift.h"
#include "rs_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  RS_MOTOR_TEST_WAIT_START = 0,
  RS_MOTOR_TEST_MOVING_UP,
  RS_MOTOR_TEST_RETURNING_ZERO,
  RS_MOTOR_TEST_ERROR
} RsMotorTestPhase;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RS_MOTOR_TEST_MOTOR_ID                    1U
#define RS_MOTOR_TEST_MASTER_ID                   0xFDU
#define RS_MOTOR_TEST_START_DELAY_MS              1000U
#define RS_MOTOR_TEST_UP_RUN_TIME_MS               30000U
#define RS_MOTOR_TEST_OFFLINE_TIMEOUT_MS          100U
#define RS_MOTOR_TEST_CONTROL_PERIOD_MS            10U
#define RS_MOTOR_TEST_CURRENT_LIMIT_A              2.0f
#define RS_MOTOR_TEST_POSITION_KP_S_1              3.0f
#define RS_MOTOR_TEST_POSITION_TOLERANCE_RAD       0.0f
#define RS_MOTOR_TEST_MAX_SPEED_RAD_S              18.84955592153875943078f
#define RS_MOTOR_TEST_ACCELERATION_RAD_S2          800.0f
#define RS_MOTOR_TEST_UP_TARGET_DEG                7200.0
#define RS_MOTOR_TEST_ZERO_TARGET_DEG              0.0
#define RS_MOTOR_TEST_DEG_TO_RAD_D                 0.01745329251994329577

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static rs_motor_t s_rs_test_motor = {
  .config = {
    .hfdcan = &hfdcan3,
    .motor_id = RS_MOTOR_TEST_MOTOR_ID,
    .master_id = RS_MOTOR_TEST_MASTER_ID,
    .motor_type = RS_MOTOR_TYPE_0,
    .offline_timeout_ms = RS_MOTOR_TEST_OFFLINE_TIMEOUT_MS,
    .multi_turn = {
      .current_limit_a = RS_MOTOR_TEST_CURRENT_LIMIT_A,
      .position_kp_s_1 = RS_MOTOR_TEST_POSITION_KP_S_1,
      .position_tolerance_rad = RS_MOTOR_TEST_POSITION_TOLERANCE_RAD,
      .control_period_ms = RS_MOTOR_TEST_CONTROL_PERIOD_MS,
    },
  },
};

volatile RsMotorTestPhase g_rs_motor_test_phase = RS_MOTOR_TEST_WAIT_START;
volatile rs_motor_status_t g_rs_motor_test_status = RS_MOTOR_STATUS_NOT_INITIALIZED;
static uint32_t s_rs_motor_test_phase_start_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * RobStride 连续多圈位置测试：先向上转到 +7200°，从首条控制命令发送成功
 * 开始计时 30 秒，再发送软件零点 0° 目标并永久运行外环保持控制。
 */
static void RsMotor_TestRun(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (g_rs_motor_test_phase == RS_MOTOR_TEST_ERROR) {
    return;
  }

  g_rs_motor_test_status = rs_motor_update(&s_rs_test_motor, now_ms);
  if (g_rs_motor_test_status != RS_MOTOR_STATUS_OK) {
    (void)rs_motor_disable(&s_rs_test_motor);
    g_rs_motor_test_phase = RS_MOTOR_TEST_ERROR;
    return;
  }

  if (g_rs_motor_test_phase == RS_MOTOR_TEST_WAIT_START) {
    if ((uint32_t)(now_ms - s_rs_motor_test_phase_start_ms) <
        RS_MOTOR_TEST_START_DELAY_MS) {
      return;
    }

    g_rs_motor_test_status = rs_motor_multi_turn_position_control(
        &s_rs_test_motor,
        RS_MOTOR_TEST_MAX_SPEED_RAD_S,
        RS_MOTOR_TEST_ACCELERATION_RAD_S2,
        RS_MOTOR_TEST_UP_TARGET_DEG * RS_MOTOR_TEST_DEG_TO_RAD_D);
    if (g_rs_motor_test_status != RS_MOTOR_STATUS_OK) {
      (void)rs_motor_disable(&s_rs_test_motor);
      g_rs_motor_test_phase = RS_MOTOR_TEST_ERROR;
      return;
    }

    s_rs_motor_test_phase_start_ms = now_ms;
    g_rs_motor_test_phase = RS_MOTOR_TEST_MOVING_UP;
    return;
  }

  if (g_rs_motor_test_phase == RS_MOTOR_TEST_MOVING_UP) {
    if ((uint32_t)(now_ms - s_rs_motor_test_phase_start_ms) <
        RS_MOTOR_TEST_UP_RUN_TIME_MS) {
      return;
    }

    g_rs_motor_test_status = rs_motor_multi_turn_position_control(
        &s_rs_test_motor,
        RS_MOTOR_TEST_MAX_SPEED_RAD_S,
        RS_MOTOR_TEST_ACCELERATION_RAD_S2,
        RS_MOTOR_TEST_ZERO_TARGET_DEG * RS_MOTOR_TEST_DEG_TO_RAD_D);
    if (g_rs_motor_test_status != RS_MOTOR_STATUS_OK) {
      (void)rs_motor_disable(&s_rs_test_motor);
      g_rs_motor_test_phase = RS_MOTOR_TEST_ERROR;
      return;
    }

    g_rs_motor_test_phase = RS_MOTOR_TEST_RETURNING_ZERO;
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
  DjiMotor_Init();
  Chassis_Init();
  Lift_Init();
  g_rs_motor_test_status = rs_motor_init(&s_rs_test_motor);
  if (g_rs_motor_test_status != RS_MOTOR_STATUS_OK) {
    g_rs_motor_test_phase = RS_MOTOR_TEST_ERROR;
  }
  bsp_can_init();
  s_rs_motor_test_phase_start_ms = HAL_GetTick();
  Comm_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Comm_RunPeriodic();
    Chassis_RunPeriodic();
    // Lift_RunPeriodic();
    // RsMotor_TestRun();
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
