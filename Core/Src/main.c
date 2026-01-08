/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ST3215 Servo Controller - Complete Fixed Version
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
#define FEEDBACK_INTERVAL_MS 200
/* USER CODE END PD */

/* USER CODE BEGIN PV */
STS3215_HandleTypeDef hservo;
STS3215_FeedbackTypeDef feedback[NUM_SERVOS];
char tx_buf[128];

// Command reception
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

// Command execution functions
void Cmd_PosAll(s16 position, u16 speed, u8 acc);
void Cmd_Pos(u8 id, s16 position, u16 speed, u8 acc);
void Cmd_SpeedAll(s16 speed, u8 acc);
void Cmd_Speed(u8 id, s16 speed, u8 acc);
void Cmd_Wheel(u8 id);
void Cmd_Servo(u8 id);
void Cmd_Torque(u8 enable);
void Cmd_Stop(void);

// Helper function to parse integers from string
int ParseInt(const char *str, int defaultVal);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void PC_Send(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

int ParseInt(const char *str, int defaultVal)
{
    if(str == NULL || *str == '\0') return defaultVal;
    return atoi(str);
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
        HAL_Delay(5);
    }
    PC_Send("$END\n");
}

void CheckForCommand(void)
{
    uint8_t byte;

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
    // Make a copy for parsing (strtok modifies the string)
    char cmd_copy[RX_BUFFER_SIZE];
    strncpy(cmd_copy, cmd, RX_BUFFER_SIZE - 1);
    cmd_copy[RX_BUFFER_SIZE - 1] = '\0';

    // Debug output
    sprintf(tx_buf, "$DBG,RX:%s\n", cmd);
    PC_Send(tx_buf);

    // Check for valid command start
    if(cmd_copy[0] != '$') return;
    if(strstr(cmd_copy, "$CMD,") == NULL) return;

    // Parse:  $CMD,<COMMAND>,<param1>,<param2>,...
    char *params[10];
    int param_count = 0;

    char *token = strtok(cmd_copy, ",");
    while(token != NULL && param_count < 10) {
        params[param_count++] = token;
        token = strtok(NULL, ",");
    }

    // params[0] = "$CMD"
    // params[1] = command name
    // params[2... ] = parameters

    if(param_count < 2) return;

    char *command = params[1];

    // PING
    if(strcmp(command, "PING") == 0) {
        PC_Send("$ACK,PING,PONG\n");
    }
    // CENTER - Move all to 2048
    else if(strcmp(command, "CENTER") == 0) {
        PC_Send("$ACK,CENTER,OK\n");
        Cmd_PosAll(2048, 1000, 50);
    }
    // STOP - Emergency stop
    else if(strcmp(command, "STOP") == 0) {
        PC_Send("$ACK,STOP,OK\n");
        Cmd_Stop();
    }
    // TORQUE - Enable/disable torque
    // Format: $CMD,TORQUE,<0/1>
    else if(strcmp(command, "TORQUE") == 0) {
        u8 enable = (param_count >= 3) ? ParseInt(params[2], 1) : 1;
        Cmd_Torque(enable);
        PC_Send("$ACK,TORQUE,OK\n");
    }
    // POSALL - Move all servos to position
    // Format: $CMD,POSALL,<pos>,<speed>,<acc>
    else if(strcmp(command, "POSALL") == 0) {
        s16 pos = (param_count >= 3) ? ParseInt(params[2], 2048) : 2048;
        u16 spd = (param_count >= 4) ? ParseInt(params[3], 1000) : 1000;
        u8 acc  = (param_count >= 5) ? ParseInt(params[4], 50) : 50;

        sprintf(tx_buf, "$DBG,POSALL pos=%d spd=%d acc=%d\n", pos, spd, acc);
        PC_Send(tx_buf);

        Cmd_PosAll(pos, spd, acc);
        PC_Send("$ACK,POSALL,OK\n");
    }
    // POS - Move single servo to position
    // Format: $CMD,POS,<id>,<pos>,<speed>,<acc>
    else if(strcmp(command, "POS") == 0) {
        if(param_count >= 4) {
            u8 id   = ParseInt(params[2], 1);
            s16 pos = ParseInt(params[3], 2048);
            u16 spd = (param_count >= 5) ? ParseInt(params[4], 1000) : 1000;
            u8 acc  = (param_count >= 6) ? ParseInt(params[5], 50) : 50;

            sprintf(tx_buf, "$DBG,POS id=%d pos=%d spd=%d acc=%d\n", id, pos, spd, acc);
            PC_Send(tx_buf);

            Cmd_Pos(id, pos, spd, acc);
            PC_Send("$ACK,POS,OK\n");
        } else {
            PC_Send("$ACK,POS,ERROR: PARAMS\n");
        }
    }
    // WHEEL - Set servo to wheel mode
    // Format: $CMD,WHEEL,<id>
    else if(strcmp(command, "WHEEL") == 0) {
        if(param_count >= 3) {
            u8 id = ParseInt(params[2], 1);
            Cmd_Wheel(id);
            PC_Send("$ACK,WHEEL,OK\n");
        } else {
            PC_Send("$ACK,WHEEL,ERROR:PARAMS\n");
        }
    }
    // SERVO - Set servo to position mode
    // Format: $CMD,SERVO,<id>
    else if(strcmp(command, "SERVO") == 0) {
        if(param_count >= 3) {
            u8 id = ParseInt(params[2], 1);
            Cmd_Servo(id);
            PC_Send("$ACK,SERVO,OK\n");
        } else {
            PC_Send("$ACK,SERVO,ERROR: PARAMS\n");
        }
    }
    // SPEED - Set single servo wheel speed
    // Format: $CMD,SPEED,<id>,<speed>,<acc>
    else if(strcmp(command, "SPEED") == 0) {
        if(param_count >= 4) {
            u8 id   = ParseInt(params[2], 1);
            s16 spd = ParseInt(params[3], 0);
            u8 acc  = (param_count >= 5) ? ParseInt(params[4], 50) : 50;

            sprintf(tx_buf, "$DBG,SPEED id=%d spd=%d acc=%d\n", id, spd, acc);
            PC_Send(tx_buf);

            Cmd_Speed(id, spd, acc);
            PC_Send("$ACK,SPEED,OK\n");
        } else {
            PC_Send("$ACK,SPEED,ERROR:PARAMS\n");
        }
    }
    // SPEEDALL - Set all servos wheel speed
    // Format: $CMD,SPEEDALL,<speed>,<acc>
    else if(strcmp(command, "SPEEDALL") == 0) {
        if(param_count >= 3) {
            s16 spd = ParseInt(params[2], 0);
            u8 acc  = (param_count >= 4) ? ParseInt(params[3], 50) : 50;

            Cmd_SpeedAll(spd, acc);
            PC_Send("$ACK,SPEEDALL,OK\n");
        } else {
            PC_Send("$ACK,SPEEDALL,ERROR: PARAMS\n");
        }
    }
    // Unknown command
    else {
        sprintf(tx_buf, "$ACK,%s,UNKNOWN\n", command);
        PC_Send(tx_buf);
    }
}

/*===========================================================================*/
/*                      COMMAND EXECUTION FUNCTIONS                          */
/*===========================================================================*/

void Cmd_PosAll(s16 position, u16 speed, u8 acc)
{
    // Clamp position
    if(position < 0) position = 0;
    if(position > 4095) position = 4095;

    sprintf(tx_buf, "$DBG,Executing POSALL:  %d @ spd=%d acc=%d\n", position, speed, acc);
    PC_Send(tx_buf);

    for(int i = 0; i < NUM_SERVOS; i++) {
        STS3215_WritePosEx(&hservo, servo_ids[i], position, speed, acc);
        HAL_Delay(10);
    }
}

void Cmd_Pos(u8 id, s16 position, u16 speed, u8 acc)
{
    // Clamp position
    if(position < 0) position = 0;
    if(position > 4095) position = 4095;

    sprintf(tx_buf, "$DBG,Executing POS: ID=%d pos=%d spd=%d acc=%d\n", id, position, speed, acc);
    PC_Send(tx_buf);

    STS3215_WritePosEx(&hservo, id, position, speed, acc);
}

void Cmd_SpeedAll(s16 speed, u8 acc)
{
    sprintf(tx_buf, "$DBG,Executing SPEEDALL:  %d acc=%d\n", speed, acc);
    PC_Send(tx_buf);

    for(int i = 0; i < NUM_SERVOS; i++) {
        STS3215_WriteSpe(&hservo, servo_ids[i], speed, acc);
        HAL_Delay(10);
    }
}

void Cmd_Speed(u8 id, s16 speed, u8 acc)
{
    sprintf(tx_buf, "$DBG,Executing SPEED: ID=%d spd=%d acc=%d\n", id, speed, acc);
    PC_Send(tx_buf);

    STS3215_WriteSpe(&hservo, id, speed, acc);
}

void Cmd_Wheel(u8 id)
{
    sprintf(tx_buf, "$DBG,Setting ID=%d to WHEEL mode\n", id);
    PC_Send(tx_buf);

    STS3215_WheelMode(&hservo, id);
}

void Cmd_Servo(u8 id)
{
    sprintf(tx_buf, "$DBG,Setting ID=%d to SERVO mode\n", id);
    PC_Send(tx_buf);

    STS3215_ServoMode(&hservo, id);
}

void Cmd_Torque(u8 enable)
{
    sprintf(tx_buf, "$DBG,Setting torque:  %s\n", enable ? "ON" : "OFF");
    PC_Send(tx_buf);

    for(int i = 0; i < NUM_SERVOS; i++) {
        STS3215_EnableTorque(&hservo, servo_ids[i], enable);
        HAL_Delay(10);
    }
}

void Cmd_Stop(void)
{
    PC_Send("$DBG,EMERGENCY STOP\n");

    for(int i = 0; i < NUM_SERVOS; i++) {
        STS3215_WriteSpe(&hservo, servo_ids[i], 0, 0);
        HAL_Delay(5);
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

    // Startup delay
    HAL_Delay(500);

    // Startup message
    PC_Send("\n\n");
    PC_Send("$READY,5\n");
    PC_Send("$DBG,ST3215 Controller v2.0\n");

    // Initialize servo library
    STS3215_Init(&hservo, &huart1);
    STS3215_SetLevel(&hservo, 0);  // Silent mode

    PC_Send("$DBG,Servo library initialized\n");

    // Enable torque on all servos
    Cmd_Torque(1);

    PC_Send("$DBG,Ready for commands!\n");

    last_feedback_time = HAL_GetTick();
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        // Check for incoming commands
        CheckForCommand();

        // Send feedback periodically
        if((HAL_GetTick() - last_feedback_time) >= FEEDBACK_INTERVAL_MS) {
            last_feedback_time = HAL_GetTick();
            SendFeedbackToPC();
        }

        HAL_Delay(1);
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
    RCC_OscInitStruct. HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL. PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL. PLLMUL = RCC_PLL_MUL16;
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
