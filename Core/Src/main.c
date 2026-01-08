/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ST3215 Servo Motor Control - Full Library Demo
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "sts3215.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define SERVO_ID 5
/* USER CODE END PD */

/* USER CODE BEGIN PV */
STS3215_HandleTypeDef hservo;
STS3215_FeedbackTypeDef feedback;
char debug_buf[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void Debug_Print(const char *msg);
void Demo_PositionMode(void);
void Demo_WheelMode(void);
void Demo_ReadFeedback(void);
void Demo_MultiServo(void);
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
    Debug_Print("\r\n============================================\r\n");
    Debug_Print("  ST3215 Full Library Demo\r\n");
    Debug_Print("============================================\r\n\r\n");

    // Initialize servo library
    STS3215_Init(&hservo, &huart1);
    STS3215_SetLevel(&hservo, 1);  // Enable responses

    // Ping servo
    sprintf(debug_buf, "Pinging servo ID %d...  ", SERVO_ID);
    Debug_Print(debug_buf);

    int pingResult = STS3215_Ping(&hservo, SERVO_ID);
    if(pingResult >= 0) {
        sprintf(debug_buf, "Found!  (ID: %d)\r\n\r\n", pingResult);
        Debug_Print(debug_buf);
    } else {
        Debug_Print("NOT FOUND!\r\n\r\n");
    }

    // Enable torque
    STS3215_EnableTorque(&hservo, SERVO_ID, 1);
    HAL_Delay(100);

    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        Debug_Print("\r\n>> Starting Demo Cycle\r\n");

        // Demo 1: Position Mode
        Demo_PositionMode();
        HAL_Delay(2000);

        // Demo 2: Read Feedback
        Demo_ReadFeedback();
        HAL_Delay(1000);

        // Demo 3:  Wheel Mode
        Demo_WheelMode();
        HAL_Delay(2000);

        Debug_Print("\r\n>> Demo Cycle Complete\r\n");
        HAL_Delay(3000);
    }
}

/* USER CODE BEGIN 4 */
void Demo_PositionMode(void)
{
    Debug_Print("\r\n=== POSITION MODE DEMO ===\r\n");

    // Set to servo mode
    STS3215_ServoMode(&hservo, SERVO_ID);
    HAL_Delay(100);

    // Move to center (2048)
    Debug_Print("Moving to position 2048 (center)...\r\n");
    STS3215_WritePosEx(&hservo, SERVO_ID, 2048, 1000, 50);
    HAL_Delay(1500);

    // Move to position 1024
    Debug_Print("Moving to position 1024...\r\n");
    STS3215_WritePosEx(&hservo, SERVO_ID, 1024, 1000, 50);
    HAL_Delay(1500);

    // Move to position 3072
    Debug_Print("Moving to position 3072...\r\n");
    STS3215_WritePosEx(&hservo, SERVO_ID, 3072, 1000, 50);
    HAL_Delay(1500);

    // Back to center
    Debug_Print("Moving back to center...\r\n");
    STS3215_WritePosEx(&hservo, SERVO_ID, 2048, 1000, 50);
    HAL_Delay(1500);
}

void Demo_WheelMode(void)
{
    Debug_Print("\r\n=== WHEEL MODE DEMO ===\r\n");

    // Set to wheel mode
    STS3215_WheelMode(&hservo, SERVO_ID);
    HAL_Delay(100);

    // Rotate forward
    Debug_Print("Rotating forward (speed 1000)...\r\n");
    STS3215_WriteSpe(&hservo, SERVO_ID, 1000, 50);
    HAL_Delay(2000);

    // Stop
    Debug_Print("Stopping...\r\n");
    STS3215_WriteSpe(&hservo, SERVO_ID, 0, 50);
    HAL_Delay(500);

    // Rotate backward
    Debug_Print("Rotating backward (speed -1000)...\r\n");
    STS3215_WriteSpe(&hservo, SERVO_ID, -1000, 50);
    HAL_Delay(2000);

    // Stop
    Debug_Print("Stopping...\r\n");
    STS3215_WriteSpe(&hservo, SERVO_ID, 0, 50);
    HAL_Delay(500);

    // Return to servo mode
    STS3215_ServoMode(&hservo, SERVO_ID);
}

void Demo_ReadFeedback(void)
{
    Debug_Print("\r\n=== FEEDBACK READING ===\r\n");

    // Read individual values
    int pos = STS3215_ReadPos(&hservo, SERVO_ID);
    int temp = STS3215_ReadTemper(&hservo, SERVO_ID);
    int volt = STS3215_ReadVoltage(&hservo, SERVO_ID);
    int load = STS3215_ReadLoad(&hservo, SERVO_ID);
    int moving = STS3215_ReadMove(&hservo, SERVO_ID);

    sprintf(debug_buf, "Position:     %d\r\n", pos);
    Debug_Print(debug_buf);

    sprintf(debug_buf, "Temperature:  %d C\r\n", temp);
    Debug_Print(debug_buf);

    sprintf(debug_buf, "Voltage:     %d. %d V\r\n", volt/10, volt%10);
    Debug_Print(debug_buf);

    sprintf(debug_buf, "Load:        %d\r\n", load);
    Debug_Print(debug_buf);

    sprintf(debug_buf, "Moving:      %s\r\n", moving ? "Yes" : "No");
    Debug_Print(debug_buf);

    // Or read all at once
    Debug_Print("\r\nFull feedback read:\r\n");
    if(STS3215_FeedBack(&hservo, SERVO_ID, &feedback) > 0) {
        sprintf(debug_buf, "  Pos: %d, Spd: %d, Load: %d\r\n",
                feedback.Position, feedback.Speed, feedback.Load);
        Debug_Print(debug_buf);
        sprintf(debug_buf, "  Temp: %dC, Volt:  %d.%dV, Curr: %dmA\r\n",
                feedback.Temperature, feedback.Voltage/10, feedback. Voltage%10, feedback.Current);
        Debug_Print(debug_buf);
    }
}

void Demo_MultiServo(void)
{
    // Example for controlling multiple servos synchronously
    // Uncomment and modify if you have multiple servos

    /*
    u8 ids[] = {1, 2, 3};
    s16 positions[] = {1024, 2048, 3072};
    u16 speeds[] = {500, 500, 500};
    u8 accs[] = {50, 50, 50};

    STS3215_SyncWritePosEx(&hservo, ids, 3, positions, speeds, accs);
    */
}
/* USER CODE END 4 */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct. HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL. PLLState = RCC_PLL_ON;
    RCC_OscInitStruct. PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
