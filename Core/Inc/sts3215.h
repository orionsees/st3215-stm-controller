/*
 * sts3215.h
 * ST3215 Servo Motor Library for STM32
 * Converted from Feetech/Waveshare Arduino Library
 * Date: 2026. 01.08
 */

#ifndef _STS3215_H
#define _STS3215_H

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/*                         TYPE DEFINITIONS                                  */
/*===========================================================================*/
typedef int8_t   s8;
typedef uint8_t  u8;
typedef uint16_t u16;
typedef int16_t  s16;
typedef uint32_t u32;
typedef int32_t  s32;

/*===========================================================================*/
/*                      PROTOCOL INSTRUCTIONS                                */
/*===========================================================================*/
#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03
#define INST_REG_WRITE      0x04
#define INST_REG_ACTION     0x05
#define INST_SYNC_READ      0x82
#define INST_SYNC_WRITE     0x83

/*===========================================================================*/
/*                         BAUD RATE DEFINITIONS                             */
/*===========================================================================*/
#define STS_BAUD_1M         0
#define STS_BAUD_500K       1
#define STS_BAUD_250K       2
#define STS_BAUD_128K       3
#define STS_BAUD_115200     4
#define STS_BAUD_76800      5
#define STS_BAUD_57600      6
#define STS_BAUD_38400      7
#define STS_BAUD_19200      8
#define STS_BAUD_14400      9
#define STS_BAUD_9600       10
#define STS_BAUD_4800       11

/*===========================================================================*/
/*                    MEMORY TABLE - EPROM (Read Only)                       */
/*===========================================================================*/
#define STS_MODEL_L                 3
#define STS_MODEL_H                 4

/*===========================================================================*/
/*                    MEMORY TABLE - EPROM (Read/Write)                      */
/*===========================================================================*/
#define STS_ID                      5
#define STS_BAUD_RATE               6
#define STS_MIN_ANGLE_LIMIT_L       9
#define STS_MIN_ANGLE_LIMIT_H       10
#define STS_MAX_ANGLE_LIMIT_L       11
#define STS_MAX_ANGLE_LIMIT_H       12
#define STS_CW_DEAD                 26
#define STS_CCW_DEAD                27
#define STS_OFS_L                   31
#define STS_OFS_H                   32
#define STS_MODE                    33

/*===========================================================================*/
/*                    MEMORY TABLE - SRAM (Read/Write)                       */
/*===========================================================================*/
#define STS_TORQUE_ENABLE           40
#define STS_ACC                     41
#define STS_GOAL_POSITION_L         42
#define STS_GOAL_POSITION_H         43
#define STS_GOAL_TIME_L             44
#define STS_GOAL_TIME_H             45
#define STS_GOAL_SPEED_L            46
#define STS_GOAL_SPEED_H            47
#define STS_TORQUE_LIMIT_L          48
#define STS_TORQUE_LIMIT_H          49
#define STS_LOCK                    55

/*===========================================================================*/
/*                    MEMORY TABLE - SRAM (Read Only)                        */
/*===========================================================================*/
#define STS_PRESENT_POSITION_L      56
#define STS_PRESENT_POSITION_H      57
#define STS_PRESENT_SPEED_L         58
#define STS_PRESENT_SPEED_H         59
#define STS_PRESENT_LOAD_L          60
#define STS_PRESENT_LOAD_H          61
#define STS_PRESENT_VOLTAGE         62
#define STS_PRESENT_TEMPERATURE     63
#define STS_MOVING                  66
#define STS_PRESENT_CURRENT_L       69
#define STS_PRESENT_CURRENT_H       70

/*===========================================================================*/
/*                         SERVO MODES                                       */
/*===========================================================================*/
#define STS_MODE_POSITION           0
#define STS_MODE_WHEEL              1
#define STS_MODE_PWM                3

/*===========================================================================*/
/*                       BROADCAST ID                                        */
/*===========================================================================*/
#define STS_BROADCAST_ID            0xFE

/*===========================================================================*/
/*                    CONFIGURATION STRUCTURE                                */
/*===========================================================================*/
typedef struct {
    UART_HandleTypeDef *huart;      // UART handle for servo communication
    u8 Level;                        // Response level (0=no response, 1=all respond)
    u8 End;                          // Endianness (0=little endian for STS series)
    u8 Error;                        // Last error code
    u32 IOTimeOut;                   // Read timeout in ms
} STS3215_HandleTypeDef;

/*===========================================================================*/
/*                    FEEDBACK DATA STRUCTURE                                */
/*===========================================================================*/
typedef struct {
    s16 Position;
    s16 Speed;
    s16 Load;
    u8  Voltage;
    u8  Temperature;
    u8  Moving;
    s16 Current;
    u8  Mode;
} STS3215_FeedbackTypeDef;

/*===========================================================================*/
/*                      INITIALIZATION FUNCTIONS                             */
/*===========================================================================*/

/**
 * @brief Initialize the STS3215 servo library
 * @param hservo Pointer to servo handle
 * @param huart Pointer to UART handle for servo communication
 */
void STS3215_Init(STS3215_HandleTypeDef *hservo, UART_HandleTypeDef *huart);

/**
 * @brief Set response level
 * @param hservo Pointer to servo handle
 * @param level 0=no response (silent), 1=all commands respond
 */
void STS3215_SetLevel(STS3215_HandleTypeDef *hservo, u8 level);

/*===========================================================================*/
/*                      POSITION CONTROL FUNCTIONS                           */
/*===========================================================================*/

/**
 * @brief Write position with speed and acceleration
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param Position Target position (0-4095, or signed for continuous)
 * @param Speed Speed (0-4095)
 * @param ACC Acceleration (0-254)
 * @return 1 on success, 0 on failure
 */
int STS3215_WritePosEx(STS3215_HandleTypeDef *hservo, u8 ID, s16 Position, u16 Speed, u8 ACC);

/**
 * @brief Register write position (async, needs RegWriteAction to execute)
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param Position Target position
 * @param Speed Speed
 * @param ACC Acceleration
 * @return 1 on success, 0 on failure
 */
int STS3215_RegWritePosEx(STS3215_HandleTypeDef *hservo, u8 ID, s16 Position, u16 Speed, u8 ACC);

/**
 * @brief Execute all registered write commands
 * @param hservo Pointer to servo handle
 * @param ID Servo ID (use STS_BROADCAST_ID for all)
 * @return 1 on success, 0 on failure
 */
int STS3215_RegWriteAction(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Sync write positions to multiple servos
 * @param hservo Pointer to servo handle
 * @param ID Array of servo IDs
 * @param IDN Number of servos
 * @param Position Array of positions
 * @param Speed Array of speeds (can be NULL)
 * @param ACC Array of accelerations (can be NULL)
 */
void STS3215_SyncWritePosEx(STS3215_HandleTypeDef *hservo, u8 ID[], u8 IDN,
                            s16 Position[], u16 Speed[], u8 ACC[]);

/*===========================================================================*/
/*                        WHEEL MODE FUNCTIONS                               */
/*===========================================================================*/

/**
 * @brief Set servo to wheel (continuous rotation) mode
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215_WheelMode(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Set servo to position (servo) mode
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215_ServoMode(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Write speed in wheel mode
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param Speed Speed (-4095 to 4095, negative for reverse)
 * @param ACC Acceleration
 * @return 1 on success, 0 on failure
 */
int STS3215_WriteSpe(STS3215_HandleTypeDef *hservo, u8 ID, s16 Speed, u8 ACC);

/*===========================================================================*/
/*                        TORQUE CONTROL FUNCTIONS                           */
/*===========================================================================*/

/**
 * @brief Enable or disable torque
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param Enable 1=enable, 0=disable
 * @return 1 on success, 0 on failure
 */
int STS3215_EnableTorque(STS3215_HandleTypeDef *hservo, u8 ID, u8 Enable);

/*===========================================================================*/
/*                        EPROM LOCK FUNCTIONS                               */
/*===========================================================================*/

/**
 * @brief Unlock EPROM for writing
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215_UnLockEprom(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Lock EPROM (protect from writing)
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215_LockEprom(STS3215_HandleTypeDef *hservo, u8 ID);

/*===========================================================================*/
/*                      CALIBRATION FUNCTIONS                                */
/*===========================================================================*/

/**
 * @brief Calibrate servo offset (set current position as center)
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215_CalibrationOfs(STS3215_HandleTypeDef *hservo, u8 ID);

/*===========================================================================*/
/*                        READ FUNCTIONS                                     */
/*===========================================================================*/

/**
 * @brief Ping a servo to check if it's online
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return Servo ID if found, -1 if not found
 */
int STS3215_Ping(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Read all feedback data from servo
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param feedback Pointer to feedback structure
 * @return Number of bytes read, -1 on failure
 */
int STS3215_FeedBack(STS3215_HandleTypeDef *hservo, u8 ID, STS3215_FeedbackTypeDef *feedback);

/**
 * @brief Read current position
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return Position value, -1 on failure
 */
int STS3215_ReadPos(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Read current speed
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return Speed value (signed), -32768 on failure
 */
int STS3215_ReadSpeed(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Read current load
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return Load value (signed), -32768 on failure
 */
int STS3215_ReadLoad(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Read supply voltage
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return Voltage in 0.1V units, -1 on failure
 */
int STS3215_ReadVoltage(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Read temperature
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return Temperature in degrees Celsius, -1 on failure
 */
int STS3215_ReadTemper(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Read moving status
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return 1 if moving, 0 if stopped, -1 on failure
 */
int STS3215_ReadMove(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Read current draw
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return Current in mA (signed), -32768 on failure
 */
int STS3215_ReadCurrent(STS3215_HandleTypeDef *hservo, u8 ID);

/**
 * @brief Read operating mode
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @return Mode (0=position, 1=wheel), -1 on failure
 */
int STS3215_ReadMode(STS3215_HandleTypeDef *hservo, u8 ID);

/*===========================================================================*/
/*                    LOW-LEVEL READ/WRITE FUNCTIONS                         */
/*===========================================================================*/

/**
 * @brief Write a single byte to servo memory
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param MemAddr Memory address
 * @param bDat Byte to write
 * @return 1 on success, 0 on failure
 */
int STS3215_WriteByte(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 bDat);

/**
 * @brief Write a word (2 bytes) to servo memory
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param MemAddr Memory address
 * @param wDat Word to write
 * @return 1 on success, 0 on failure
 */
int STS3215_WriteWord(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u16 wDat);

/**
 * @brief Read a single byte from servo memory
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param MemAddr Memory address
 * @return Byte value, -1 on failure
 */
int STS3215_ReadByte(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr);

/**
 * @brief Read a word (2 bytes) from servo memory
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param MemAddr Memory address
 * @return Word value, -1 on failure
 */
int STS3215_ReadWord(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr);

/**
 * @brief General write to servo memory
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param MemAddr Memory address
 * @param nDat Data buffer
 * @param nLen Data length
 * @return 1 on success, 0 on failure
 */
int STS3215_GenWrite(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);

/**
 * @brief General read from servo memory
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param MemAddr Memory address
 * @param nDat Data buffer
 * @param nLen Data length to read
 * @return Number of bytes read, 0 on failure
 */
int STS3215_Read(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);

/*===========================================================================*/
/*                    ID AND BAUD RATE CONFIGURATION                         */
/*===========================================================================*/

/**
 * @brief Change servo ID
 * @param hservo Pointer to servo handle
 * @param oldID Current servo ID
 * @param newID New servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215_SetID(STS3215_HandleTypeDef *hservo, u8 oldID, u8 newID);

/**
 * @brief Change servo baud rate
 * @param hservo Pointer to servo handle
 * @param ID Servo ID
 * @param baudRateIndex Baud rate index (STS_BAUD_xxx)
 * @return 1 on success, 0 on failure
 */
int STS3215_SetBaudRate(STS3215_HandleTypeDef *hservo, u8 ID, u8 baudRateIndex);

#ifdef __cplusplus
}
#endif

#endif /* _STS3215_H */
