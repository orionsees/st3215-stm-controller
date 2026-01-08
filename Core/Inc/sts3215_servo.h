/*
 * scs15_servo.h
 *
 *  Created on: Aug 13, 2024
 *      Author: user
 */

#ifndef INC_STS3215_SERVO_H_
#define INC_STS3215_SERVO_H_

#include <stdlib.h>
#include "usart.h"

#define STS3215_UART &huart2
#define BIG_ENDIAN_PERIPHERAL 0

// Commands
#define STS3215_READ_DATA_COMMAND 0x02
#define STS3215_WRITE_DATA_COMMAND 0x03

// Registers
#define STS3215_ACCELERATION_REG 0x29
#define STS3215_BAUD_RATE_REG 0x06
#define STS3215_ACK_MODE_REG 0x08
#define STS3215_TEMPERATURE_REG 0x3F
#define STS3215_VOLTAGE_REG 0x3E
#define STS3215_OPERATION_MODE_REG 0x21
#define STS3215_REFERENCE_SPEED_H_REG 0x2E

void Host2Servo(uint8_t *dataSentFirst, uint8_t *dataSendSecond, uint16_t data);
uint8_t STS3215Checksum(uint8_t *data);
uint8_t STS3215SetBaudrate(uint8_t scs15_id, uint32_t baudrate);

void STS3215SetSilentMode(uint8_t scs15_id);
void STS3215SetNonSilentMode(uint8_t scs15_id);

uint8_t STS3215ReadTemperature(uint8_t scs15_id, uint8_t *drive_temperature);
uint8_t STS3215ReadVoltage(uint8_t scs15_id, uint8_t *drive_voltage);

void STS3215SetMotorMode(uint8_t scs15_id);
void STS3215SetServoMode(uint8_t scs15_id);
void STS3215SetSpeed(uint8_t scs15_id, int16_t scs15_speed);

void STS3215TurnToPosition(uint8_t scs15_id, uint16_t scs15_position,
		uint16_t scs15_delay, uint16_t scs15_speed, uint8_t sc15_acceleration);

#endif /* INC_STS3215_SERVO_H_ */
