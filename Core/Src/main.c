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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "car_control.h"
#include "chassis.h"
#include "communication.h"
#include "led.h"
#include "motor.h"
#include "mpu6050.h"
#include "remote_control.h"
#include "serial.h"
#include "telemetry.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STATUS_LED_EMERGENCY_BLINK_MS 250U
#define STATUS_LED_IMU_BLINK_MS       100U

#define ENCODER_JOYSTICK_TEST_ENABLED 0
#define ENCODER_TEST_PWM              180
#define ENCODER_TEST_REMOTE_MAX_VX    0.20f
#define ENCODER_TEST_REMOTE_MAX_AZ    1.00f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#if ENCODER_JOYSTICK_TEST_ENABLED
static bool encoder_test_running;
static bool encoder_test_armed;
static bool encoder_test_result_held;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if ENCODER_JOYSTICK_TEST_ENABLED
static float encoder_test_abs(float value)
{
  return value >= 0.0f ? value : -value;
}

static int16_t encoder_test_to_pwm(float command)
{
  float scaled = command * (float)ENCODER_TEST_PWM;

  return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void encoder_joystick_test_init(void)
{
  motor_init();
  remote_control_init();
  communication_init();
  encoder_test_running = false;
  encoder_test_armed = false;
  encoder_test_result_held = false;
  led_off();
  serial_print(SERIAL_HOST, "TEST,READY,JOYSTICK,180\r\n");
}

static void encoder_joystick_test_process(void)
{
  float vx_mps = 0.0f;
  float az_radps = 0.0f;
  float drive = 0.0f;
  float turn = 0.0f;
  float left_command;
  float right_command;
  float maximum;
  int16_t left_pwm;
  int16_t right_pwm;
  bool remote_valid;
  bool moving;

  remote_control_process();
  remote_valid = remote_control_get_command(&vx_mps, &az_radps);
  if (remote_valid) {
    drive = vx_mps / ENCODER_TEST_REMOTE_MAX_VX;
    turn = az_radps / ENCODER_TEST_REMOTE_MAX_AZ;
  }

  left_command = drive - turn;
  right_command = drive + turn;
  maximum = encoder_test_abs(left_command);
  if (encoder_test_abs(right_command) > maximum) {
    maximum = encoder_test_abs(right_command);
  }
  if (maximum > 1.0f) {
    left_command /= maximum;
    right_command /= maximum;
  }
  left_pwm = encoder_test_to_pwm(left_command);
  right_pwm = encoder_test_to_pwm(right_command);
  moving = left_pwm != 0 || right_pwm != 0;

  if (!moving) {
    if (encoder_test_running) {
      const encoder_data_t *encoder;

      motor_stop();
      encoder_update();
      encoder_test_running = false;
      encoder_test_result_held = true;
      led_off();
      encoder = encoder_get_data();
      serial_printf(SERIAL_HOST, "TEST,DONE,%ld,%ld\r\n",
                    (long)encoder->left_total,
                    (long)encoder->right_total);
    } else {
      if (remote_valid) {
        encoder_test_armed = true;
      }
      motor_stop();
    }
    return;
  }

  if (!encoder_test_running && !encoder_test_armed) {
    motor_stop();
    return;
  }

  if (!encoder_test_running) {
    encoder_reset();
    encoder_test_running = true;
    encoder_test_armed = false;
    encoder_test_result_held = false;
    led_on();
    serial_print(SERIAL_HOST, "TEST,RUN\r\n");
  }
  motor_set(left_pwm, right_pwm);
}
#endif

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  led_init();
  encoder_init();
  serial_init();
  mpu6050_init(&hi2c1);
#if ENCODER_JOYSTICK_TEST_ENABLED
  encoder_joystick_test_init();
  serial_print(SERIAL_DEBUG, "encoder joystick 5pct test ready\r\n");
#else
  car_control_init();
  telemetry_init();
  serial_print(SERIAL_DEBUG, "cup_car ready\r\n");
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_encoder_ms;
#if !ENCODER_JOYSTICK_TEST_ENABLED
    static uint32_t last_led_ms;
    static bool emergency_was_active;
    static bool heading_was_active;
#endif
    uint32_t now = HAL_GetTick();
#if !ENCODER_JOYSTICK_TEST_ENABLED
    bool emergency_active;
    bool heading_active;
#endif

#if ENCODER_JOYSTICK_TEST_ENABLED
    if (!encoder_test_result_held && now - last_encoder_ms >= 10U) {
#else
    if (now - last_encoder_ms >= 10U) {
#endif
      last_encoder_ms = now;
      encoder_update();
    }

#if ENCODER_JOYSTICK_TEST_ENABLED
    communication_process();
    encoder_joystick_test_process();
#else
    {
      const chassis_feedback_t *chassis = chassis_get_feedback();
      const car_heading_feedback_t *heading =
        car_control_get_heading_feedback();
      mpu6050_set_stationary_hint(
        chassis->left_pwm == 0 && chassis->right_pwm == 0);
      mpu6050_set_straight_motion_hint(
        car_control_heading_active(),
        heading->error_rad,
        heading->correction_radps);
    }
    mpu6050_process();
    car_control_process();
    telemetry_process();

    emergency_active = car_control_emergency_stopped();
    heading_active = car_control_heading_active();
    if (emergency_active) {
      if (!emergency_was_active) {
        last_led_ms = now;
        led_on();
      } else if (now - last_led_ms >= STATUS_LED_EMERGENCY_BLINK_MS) {
        last_led_ms = now;
        led_toggle();
      }
    } else if (car_control_get_mode() == CAR_MODE_NAVIGATION) {
      if (heading_active) {
        if (emergency_was_active || !heading_was_active) {
          last_led_ms = now;
          led_on();
        } else if (now - last_led_ms >= STATUS_LED_IMU_BLINK_MS) {
          last_led_ms = now;
          led_toggle();
        }
      } else {
        led_on();
      }
    } else {
      led_off();
    }
    emergency_was_active = emergency_active;
    heading_was_active = heading_active;
#endif
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

/* USER CODE BEGIN 4 */

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
