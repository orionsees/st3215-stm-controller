/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main. c
  * @brief          : Main program body with ST3215 servo control
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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STS3215_ID 5              // Servo ID (default is 1)
#define SERVO_TEST_DELAY 2000     // ms delay between commands
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t sys_tick_counter = 0;
char debug_buf[256];  // Buffer for debug messages
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN FPP */
void TestServoMotorMode(void);
void TestServoPositionMode(void);
void Debug_Print(const char *msg);
/* USER CODE END FPP */

/* USER CODE BEGIN 0 */
// Debug print function using UART2
void Debug_Print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
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

  /* MCU Configuration*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  // Print startup message
  Debug_Print("\r\n\r\n");
  Debug_Print("========================================\r\n");
  Debug_Print("  ST3215 Servo Motor Control Started!   \r\n");
  Debug_Print("========================================\r\n");

  sprintf(debug_buf, "Servo ID:  %d\r\n", STS3215_ID);
  Debug_Print(debug_buf);

  // Set servo to silent mode (no response feedback)
  Debug_Print("Setting servo to silent mode...\r\n");
  STS3215SetSilentMode(STS3215_ID);
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    Debug_Print("\r\n>> Code is running - ST3215 Controller Active\r\n");

    Debug_Print("\r\n========== TEST 1: MOTOR MODE ==========\r\n");
    TestServoMotorMode();

    HAL_Delay(3000);

    Debug_Print("\r\n========== TEST 2: POSITION MODE ==========\r\n");
    TestServoPositionMode();

    HAL_Delay(3000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL. PLLState = RCC_PLL_ON;
  RCC_OscInitStruct. PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL. PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct. ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct. AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief Test servo in motor (continuous rotation) mode
  */
void TestServoMotorMode(void)
{
  Debug_Print("Switching to MOTOR MODE...\r\n");
  STS3215SetMotorMode(STS3215_ID);
  HAL_Delay(200);

  // Rotate forward at speed 2000
  sprintf(debug_buf, "Rotating FORWARD at speed %d.. .\r\n", 2000);
  Debug_Print(debug_buf);
  STS3215SetSpeed(STS3215_ID, 2000);
  HAL_Delay(SERVO_TEST_DELAY);

  // Stop
  Debug_Print("Stopping motor...\r\n");
  STS3215SetSpeed(STS3215_ID, 0);
  HAL_Delay(500);

  // Rotate backward at speed -2000
  sprintf(debug_buf, "Rotating BACKWARD at speed %d...\r\n", -2000);
  Debug_Print(debug_buf);
  STS3215SetSpeed(STS3215_ID, -2000);
  HAL_Delay(SERVO_TEST_DELAY);

  // Stop
  Debug_Print("Stopping motor...\r\n");
  STS3215SetSpeed(STS3215_ID, 0);
  HAL_Delay(500);
}

/**
  * @brief Test servo in position (servo) mode
  */
void TestServoPositionMode(void)
{
  Debug_Print("Switching to SERVO (POSITION) MODE...\r\n");
  STS3215SetServoMode(STS3215_ID);
  HAL_Delay(200);

  // Move to position 256 (center)
  sprintf(debug_buf, "Moving to CENTER position (%d)...\r\n", 256);
  Debug_Print(debug_buf);
  STS3215TurnToPosition(STS3215_ID, 256, 500, 500, 100);
  HAL_Delay(SERVO_TEST_DELAY);

  // Move to position 512 (one direction)
  sprintf(debug_buf, "Moving to position %d...\r\n", 512);
  Debug_Print(debug_buf);
  STS3215TurnToPosition(STS3215_ID, 512, 500, 500, 100);
  HAL_Delay(SERVO_TEST_DELAY);

  // Move to position 0 (other direction)
  sprintf(debug_buf, "Moving to position %d.. .\r\n", 0);
  Debug_Print(debug_buf);
  STS3215TurnToPosition(STS3215_ID, 0, 500, 500, 100);
  HAL_Delay(SERVO_TEST_DELAY);

  // Back to center
  sprintf(debug_buf, "Moving back to CENTER position (%d)...\r\n", 256);
  Debug_Print(debug_buf);
  STS3215TurnToPosition(STS3215_ID, 256, 500, 500, 100);
  HAL_Delay(SERVO_TEST_DELAY);
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
  * @param  line:  assert_param error line source number
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
