/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           :  main.c
  * @brief          :  ST3215 Servo Motor Control
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "sts3215_servo.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define STS3215_ID 5              // Your servo ID
#define SERVO_TEST_DELAY 2000
/* USER CODE END PD */

/* USER CODE BEGIN PV */
char debug_buf[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void Debug_Print(const char *msg);
void TestServoMotorMode(void);
void TestServoPositionMode(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Debug_Print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();  // Servo UART (PA9, 1Mbps)
  MX_USART2_UART_Init();  // Debug UART (PA2, 115200)

  /* USER CODE BEGIN 2 */
  Debug_Print("\r\n========================================\r\n");
  Debug_Print("  ST3215 Servo Motor Control Started!\r\n");
  Debug_Print("========================================\r\n");

  sprintf(debug_buf, "Servo ID: %d\r\n", STS3215_ID);
  Debug_Print(debug_buf);
  Debug_Print("Servo UART:  USART1 (PA9) @ 1Mbps\r\n");
  Debug_Print("Debug UART: USART2 (PA2) @ 115200\r\n\r\n");

  Debug_Print("Setting servo to silent mode...\r\n");
  STS3215SetSilentMode(STS3215_ID);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    Debug_Print("\r\n>> Code is running - ST3215 Controller Active\r\n");

    Debug_Print("\r\n========== TEST 1: MOTOR MODE ==========\r\n");
    TestServoMotorMode();
    HAL_Delay(3000);

    Debug_Print("\r\n========== TEST 2: POSITION MODE ==========\r\n");
    TestServoPositionMode();
    HAL_Delay(3000);
  }
}

/* USER CODE BEGIN 4 */
void TestServoMotorMode(void)
{
  Debug_Print("Switching to MOTOR MODE...\r\n");
  STS3215SetMotorMode(STS3215_ID);
  HAL_Delay(200);

  Debug_Print("Rotating FORWARD at speed 2000.. .\r\n");
  STS3215SetSpeed(STS3215_ID, 2000);
  HAL_Delay(SERVO_TEST_DELAY);

  Debug_Print("Stopping motor...\r\n");
  STS3215SetSpeed(STS3215_ID, 0);
  HAL_Delay(500);

  Debug_Print("Rotating BACKWARD at speed -2000...\r\n");
  STS3215SetSpeed(STS3215_ID, -2000);
  HAL_Delay(SERVO_TEST_DELAY);

  Debug_Print("Stopping motor...\r\n");
  STS3215SetSpeed(STS3215_ID, 0);
  HAL_Delay(500);
}

void TestServoPositionMode(void)
{
  Debug_Print("Switching to SERVO (POSITION) MODE...\r\n");
  STS3215SetServoMode(STS3215_ID);
  HAL_Delay(200);

  Debug_Print("Moving to CENTER position (2048)...\r\n");
  STS3215TurnToPosition(STS3215_ID, 2048, 500, 500, 100);
  HAL_Delay(SERVO_TEST_DELAY);

  Debug_Print("Moving to position 1024...\r\n");
  STS3215TurnToPosition(STS3215_ID, 1024, 500, 500, 100);
  HAL_Delay(SERVO_TEST_DELAY);

  Debug_Print("Moving to position 3072...\r\n");
  STS3215TurnToPosition(STS3215_ID, 3072, 500, 500, 100);
  HAL_Delay(SERVO_TEST_DELAY);
}
/* USER CODE END 4 */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct. OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct. PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL. PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct. ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
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

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
