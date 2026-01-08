/*
 * scs15_servo.c
 *
 *  Created on: Aug 13, 2024
 *      Author: user
 */

#include "sts3215_servo.h"

void Host2Servo(uint8_t *dataSentFirst, uint8_t *dataSendSecond, uint16_t data)
{
	if (BIG_ENDIAN_PERIPHERAL)
	{
		*dataSentFirst = (data >> 8);
		*dataSendSecond = (data & 0xFF);
	}
	else
	{
		*dataSendSecond = (data >> 8);
		*dataSentFirst = (data & 0xFF);
	}
}

uint8_t STS3215Checksum(uint8_t *sts3215_data)
{
	uint8_t check_sum = 0;

	for (uint8_t i = 0; i <= sts3215_data[3]; i++)
	{
		check_sum += sts3215_data[i + 2];
	}

	return ~check_sum;
}

uint8_t STS3215SetBaudrate(uint8_t sts3215_id, uint32_t baudrate)
{
	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = STS3215_WRITE_DATA_COMMAND;
	txBuffer[5] = STS3215_BAUD_RATE_REG;

	switch (baudrate)
	{
	case 1000000:
		txBuffer[6] = 0x00;
		break;
	case 500000:
		txBuffer[6] = 0x01;
		break;
	case 250000:
		txBuffer[6] = 0x02;
		break;
	case 128000:
		txBuffer[6] = 0x03;
		break;
	case 115200:
		txBuffer[6] = 0x04;
		break;
	case 76800:
		txBuffer[6] = 0x05;
		break;
	case 57600:
		txBuffer[6] = 0x06;
		break;
	case 38400:
		txBuffer[6] = 0x07;
		break;
	default:
		return 1;

	}

	txBuffer[7] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);

	return 0;

}

void STS3215SetSilentMode(uint8_t sts3215_id)
{
	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = STS3215_WRITE_DATA_COMMAND;
	txBuffer[5] = STS3215_ACK_MODE_REG;
	txBuffer[6] = 0x00;
	txBuffer[7] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}

void STS3215SetNonSilentMode(uint8_t sts3215_id)
{
	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = STS3215_WRITE_DATA_COMMAND;
	txBuffer[5] = STS3215_ACK_MODE_REG;
	txBuffer[6] = 0x01;
	txBuffer[7] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}

uint8_t STS3215ReadTemperature(uint8_t sts3215_id, uint8_t *drive_temperature)
{
	uint8_t rxBuffer[7] =
	{ 0 };
	uint8_t rx_data_length = 0;
	uint8_t rx_data_error = 0;

	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = STS3215_READ_DATA_COMMAND;
	txBuffer[5] = STS3215_TEMPERATURE_REG;
	txBuffer[6] = 0x01;
	txBuffer[7] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_HalfDuplex_EnableReceiver(STS3215_UART);
	HAL_UART_Receive(STS3215_UART, rxBuffer, sizeof(rxBuffer), 100);

	if (rxBuffer[0] == 0xFF && rxBuffer[1] == 0xFF)
	{
		rx_data_length = rxBuffer[3] - 2;	//scs15 magic
		rx_data_error = rxBuffer[4];
	}
	else
	{
		//Error_Handler();
		return 1;
	}

	if (rx_data_error != 0)
	{
		//Error_Handler();
		return 2;
	}
	else if (rx_data_length == 0)
	{
		//Error_Handler();
		return 3;
	}
	else
	{
		*drive_temperature = rxBuffer[5];
		return 0;
	}
}

uint8_t STS3215ReadVoltage(uint8_t sts3215_id, uint8_t *drive_voltage)
{
	uint8_t rxBuffer[7] =
	{ 0 };
	uint8_t rx_data_length = 0;
	uint8_t rx_data_error = 0;

	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = STS3215_READ_DATA_COMMAND;
	txBuffer[5] = STS3215_VOLTAGE_REG;
	txBuffer[6] = 0x01;
	txBuffer[7] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_HalfDuplex_EnableReceiver(STS3215_UART);
	HAL_UART_Receive(STS3215_UART, rxBuffer, sizeof(rxBuffer), 100);

	if (rxBuffer[0] == 0xFF && rxBuffer[1] == 0xFF)
	{
		rx_data_length = rxBuffer[3] - 2;	//scs15 magic
		rx_data_error = rxBuffer[4];
	}
	else
	{
		//Error_Handler();
		return 1;
	}

	if (rx_data_error != 0)
	{
		//Error_Handler();
		return 2;
	}
	else if (rx_data_length == 0)
	{
		//Error_Handler();
		return 3;
	}
	else
	{
		*drive_voltage = rxBuffer[5];
		return 0;
	}
}

void STS3215TurnToPosition(uint8_t sts3215_id, uint16_t sts3215_position,
		uint16_t sts3215_delay, uint16_t sts3215_speed,
		uint8_t sts3215_acceleration)
{
	uint8_t txBuffer[14];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x0A; // 10
	txBuffer[4] = STS3215_WRITE_DATA_COMMAND;
	txBuffer[5] = STS3215_ACCELERATION_REG;
	txBuffer[6] = sts3215_acceleration;
	Host2Servo(txBuffer + 7, txBuffer + 8, sts3215_position);
	Host2Servo(txBuffer + 9, txBuffer + 10, sts3215_delay);
	Host2Servo(txBuffer + 11, txBuffer + 12, sts3215_speed);

	txBuffer[13] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);
}

void STS3215SetMotorMode(uint8_t sts3215_id)
{
	uint8_t txBuffer[8];
	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)

	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = STS3215_WRITE_DATA_COMMAND;
	txBuffer[5] = STS3215_OPERATION_MODE_REG;
	txBuffer[6] = 0x01;

	txBuffer[7] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}

void STS3215SetSpeed(uint8_t sts3215_id, int16_t sts3215_speed)
{
	if (sts3215_speed < -3000)
	{
		sts3215_speed = -3000;
	}
	else if (sts3215_speed > 3000)
	{
		sts3215_speed = 3000;
	}
	else
	{
		;
	}

	uint8_t txBuffer[9];
	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)

	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x05;
	txBuffer[4] = STS3215_WRITE_DATA_COMMAND;
	txBuffer[5] = STS3215_REFERENCE_SPEED_H_REG;

	Host2Servo(txBuffer + 6, txBuffer + 7,
			((uint16_t) (abs(sts3215_speed)))
					| (((uint16_t) (sts3215_speed < 0)) << 15));

	txBuffer[8] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}

void STS3215SetServoMode(uint8_t sts3215_id)
{
	uint8_t txBuffer[8];
	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)

	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = sts3215_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = STS3215_WRITE_DATA_COMMAND;
	txBuffer[5] = STS3215_OPERATION_MODE_REG;
	txBuffer[6] = 0x00;

	txBuffer[7] = STS3215Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(STS3215_UART);
	HAL_UART_Transmit(STS3215_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}
