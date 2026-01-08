/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ST3215 Servo Controller - Fixed Version
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
#include <stdlib.h>
/* USER CODE END Includes */

/*===========================================================================*/
/*              CONFIGURATION - MODIFY TO MATCH YOUR SETUP                   */
/*===========================================================================*/
/* USER CODE BEGIN PD */
#define NUM_SERVOS 5
static u8 servo_ids[NUM_SERVOS] = {1, 2, 3, 4, 5};

#define RX_BUFFER_SIZE 128
#define FEEDBACK_INTERVAL_MS 200   // Slower:  200ms (5Hz) instead of 50ms
/* USER CODE END PD */

/* USER CODE BEGIN PV */
STS3215_HandleTypeDef hservo;
STS3215_FeedbackTypeDef feedback[NUM_SERVOS];
char tx_buf[128];

// Command reception - polling based (more reliable than interrupt for this use case)
char rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_index = 0;

// Timing
uint32_t last_feedback_time = 0;
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void PC_Send(const char *msg);
void SendFeedbackToPC(void);
void CheckForCommand(void);
void ProcessCommand(char *cmd);
void ExecutePosAll(s16 position, u16 speed, u8 acc);
void ExecuteStop(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void PC_Send(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void SendFeedbackToPC(void)
{
    for(int i = 0; i < NUM_SERVOS; i++) {
        u8 id = servo_ids[i];
        int result = STS3215_FeedBack(&hservo, id, &feedback[i]);

        if(result > 0) {
            sprintf(tx_buf, "$FB,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    id,
                    feedback[i].Position,
                    feedback[i].Speed,
                    feedback[i].Load,
                    feedback[i].Temperature,
                    feedback[i].Voltage,
                    feedback[i].Current,
                    feedback[i].Moving);
        } else {
            sprintf(tx_buf, "$FB,%d,OFFLINE\n", id);
        }
        PC_Send(tx_buf);
        HAL_Delay(5);  // Small delay between each servo
    }
    PC_Send("$END\n");
}

void CheckForCommand(void)
{
    uint8_t byte;

    // Check if data available (non-blocking)
    while(HAL_UART_Receive(&huart2, &byte, 1, 1) == HAL_OK) {
        if(byte == '\n' || byte == '\r') {
            if(rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                ProcessCommand(rx_buffer);
                rx_index = 0;
            }
        } else {
            if(rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = byte;
            }
        }
    }
}

void ProcessCommand(char *cmd)
{
    // Debug: echo received command
    sprintf(tx_buf, "$DBG,Received:%s\n", cmd);
    PC_Send(tx_buf);

    // Check for valid command start
    if(cmd[0] != '$') return;

    // Parse command
    if(strstr(cmd, "$CMD,CENTER") != NULL) {
        PC_Send("$ACK,CENTER,OK\n");
        ExecutePosAll(2048, 1000, 50);
    }
    else if(strstr(cmd, "$CMD,PING") != NULL) {
        PC_Send("$ACK,PING,PONG\n");
    }
    else if(strstr(cmd, "$CMD,STOP") != NULL) {
        PC_Send("$ACK,STOP,OK\n");
        ExecuteStop();
    }
    else if(strstr(cmd, "$CMD,POSALL") != NULL) {
        // Parse:  $CMD,POSALL,<pos>,<speed>,<acc>
        char *token = strtok(cmd, ",");  // $CMD
        token = strtok(NULL, ",");        // POSALL
        token = strtok(NULL, ",");        // position

        if(token != NULL) {
            s16 pos = atoi(token);
            token = strtok(NULL, ",");
            u16 spd = token ? atoi(token) : 1000;
            token = strtok(NULL, ",");
            u8 acc = token ? atoi(token) : 50;

            PC_Send("$ACK,POSALL,OK\n");
            ExecutePosAll(pos, spd, acc);
        }
    }
}

void ExecutePosAll(s16 position, u16 speed, u8 acc)
{
    if(position < 0) position = 0;
    if(position > 4095) position = 4095;

    sprintf(tx_buf, "$DBG,Moving all to %d\n", position);
    PC_Send(tx_buf);

    for(int i = 0; i < NUM_SERVOS; i++) {
        STS3215_WritePosEx(&hservo, servo_ids[i], position, speed, acc);
        HAL_Delay(10);
    }
}

void ExecuteStop(void)
{
    for(int i = 0; i < NUM_SERVOS; i++) {
        STS3215_WriteSpe(&hservo, servo_ids[i], 0, 0);
    }
}

/* USER CODE END 0 */

/*===========================================================================*/
/*                              MAIN FUNCTION                                */
/*===========================================================================*/

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();  // Servo (PA9, 1Mbps)
    MX_USART2_UART_Init();  // PC Communication (PA2, 115200)

    /* USER CODE BEGIN 2 */

    // Startup message
    PC_Send("\n\n$READY,5\n");
    PC_Send("$DBG,ST3215 Controller Started\n");

    // Initialize servo library
    STS3215_Init(&hservo, &huart1);
    STS3215_SetLevel(&hservo, 0);  // Silent mode - no responses from servos

    PC_Send("$DBG,Servo library initialized\n");

    // Enable torque on all servos
    for(int i = 0; i < NUM_SERVOS; i++) {
        STS3215_EnableTorque(&hservo, servo_ids[i], 1);
        HAL_Delay(20);
    }

    PC_Send("$DBG,Torque enabled on all servos\n");
    PC_Send("$DBG,Ready for commands\n");

    last_feedback_time = HAL_GetTick();
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // Check for incoming commands (polling)
        CheckForCommand();

        // Send feedback periodically
        if((HAL_GetTick() - last_feedback_time) >= FEEDBACK_INTERVAL_MS) {
            last_feedback_time = HAL_GetTick();
            SendFeedbackToPC();
        }

        HAL_Delay(1);  // Small delay to prevent tight loop
    }
}

/*===========================================================================*/
/*                         SYSTEM CONFIGURATION                              */
/*===========================================================================*/

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL. PLLState = RCC_PLL_ON;
    RCC_OscInitStruct. PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct. ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
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
