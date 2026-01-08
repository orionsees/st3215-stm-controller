/*
 * sts3215.c
 * ST3215 Servo Motor Library for STM32
 * Converted from Feetech/Waveshare Arduino Library
 * Date: 2026.01.08
 */

#include "sts3215.h"
#include <string.h>

/*===========================================================================*/
/*                      PRIVATE FUNCTION PROTOTYPES                          */
/*===========================================================================*/
static void Host2SCS(STS3215_HandleTypeDef *hservo, u8 *DataL, u8 *DataH, u16 Data);
static u16 SCS2Host(STS3215_HandleTypeDef *hservo, u8 DataL, u8 DataH);
static void WriteBuf(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
static int Ack(STS3215_HandleTypeDef *hservo, u8 ID);
static int CheckHead(STS3215_HandleTypeDef *hservo);
static void FlushRx(STS3215_HandleTypeDef *hservo);

/*===========================================================================*/
/*                      INITIALIZATION FUNCTIONS                             */
/*===========================================================================*/

void STS3215_Init(STS3215_HandleTypeDef *hservo, UART_HandleTypeDef *huart)
{
    hservo->huart = huart;
    hservo->Level = 1;          // All commands respond by default
    hservo->End = 0;            // Little endian for STS series
    hservo->Error = 0;
    hservo->IOTimeOut = 100;    // 100ms timeout
}

void STS3215_SetLevel(STS3215_HandleTypeDef *hservo, u8 level)
{
    hservo->Level = level;
}

/*===========================================================================*/
/*                      PRIVATE HELPER FUNCTIONS                             */
/*===========================================================================*/

// Convert 16-bit value to two 8-bit values (endianness aware)
static void Host2SCS(STS3215_HandleTypeDef *hservo, u8 *DataL, u8 *DataH, u16 Data)
{
    if(hservo->End) {
        *DataL = (Data >> 8);
        *DataH = (Data & 0xFF);
    } else {
        *DataH = (Data >> 8);
        *DataL = (Data & 0xFF);
    }
}

// Convert two 8-bit values to 16-bit value (endianness aware)
static u16 SCS2Host(STS3215_HandleTypeDef *hservo, u8 DataL, u8 DataH)
{
    u16 Data;
    if(hservo->End) {
        Data = DataL;
        Data <<= 8;
        Data |= DataH;
    } else {
        Data = DataH;
        Data <<= 8;
        Data |= DataL;
    }
    return Data;
}

// Flush RX buffer
static void FlushRx(STS3215_HandleTypeDef *hservo)
{
    u8 dummy;
    HAL_HalfDuplex_EnableReceiver(hservo->huart);
    while(HAL_UART_Receive(hservo->huart, &dummy, 1, 1) == HAL_OK);
}

// Write command buffer to servo
static void WriteBuf(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun)
{
    u8 msgLen = 2;
    u8 bBuf[7];
    u8 CheckSum = 0;

    bBuf[0] = 0xFF;
    bBuf[1] = 0xFF;
    bBuf[2] = ID;
    bBuf[4] = Fun;

    if(nDat) {
        msgLen += nLen + 1;
        bBuf[3] = msgLen;
        bBuf[5] = MemAddr;

        HAL_HalfDuplex_EnableTransmitter(hservo->huart);
        HAL_UART_Transmit(hservo->huart, bBuf, 6, 100);
        HAL_UART_Transmit(hservo->huart, nDat, nLen, 100);
    } else {
        bBuf[3] = msgLen;
        HAL_HalfDuplex_EnableTransmitter(hservo->huart);
        HAL_UART_Transmit(hservo->huart, bBuf, 5, 100);
    }

    // Calculate checksum
    CheckSum = ID + msgLen + Fun + MemAddr;
    if(nDat) {
        for(u8 i = 0; i < nLen; i++) {
            CheckSum += nDat[i];
        }
    }
    CheckSum = ~CheckSum;
    HAL_UART_Transmit(hservo->huart, &CheckSum, 1, 100);
}

// Check for response header (0xFF 0xFF)
static int CheckHead(STS3215_HandleTypeDef *hservo)
{
    u8 bDat;
    u8 bBuf[2] = {0, 0};
    u8 Cnt = 0;

    HAL_HalfDuplex_EnableReceiver(hservo->huart);

    while(1) {
        if(HAL_UART_Receive(hservo->huart, &bDat, 1, hservo->IOTimeOut) != HAL_OK) {
            return 0;
        }
        bBuf[1] = bBuf[0];
        bBuf[0] = bDat;
        if(bBuf[0] == 0xFF && bBuf[1] == 0xFF) {
            break;
        }
        Cnt++;
        if(Cnt > 10) {
            return 0;
        }
    }
    return 1;
}

// Wait for and validate acknowledgment
static int Ack(STS3215_HandleTypeDef *hservo, u8 ID)
{
    hservo->Error = 0;

    if(ID != STS_BROADCAST_ID && hservo->Level) {
        if(! CheckHead(hservo)) {
            return 0;
        }

        u8 bBuf[4];
        if(HAL_UART_Receive(hservo->huart, bBuf, 4, hservo->IOTimeOut) != HAL_OK) {
            return 0;
        }

        if(bBuf[0] != ID) {
            return 0;
        }
        if(bBuf[1] != 2) {
            return 0;
        }

        u8 calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
        if(calSum != bBuf[3]) {
            return 0;
        }
        hservo->Error = bBuf[2];
    }
    return 1;
}

/*===========================================================================*/
/*                    LOW-LEVEL READ/WRITE FUNCTIONS                         */
/*===========================================================================*/

int STS3215_GenWrite(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
    FlushRx(hservo);
    WriteBuf(hservo, ID, MemAddr, nDat, nLen, INST_WRITE);
    return Ack(hservo, ID);
}

int STS3215_RegWrite(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
    FlushRx(hservo);
    WriteBuf(hservo, ID, MemAddr, nDat, nLen, INST_REG_WRITE);
    return Ack(hservo, ID);
}

int STS3215_RegWriteAction(STS3215_HandleTypeDef *hservo, u8 ID)
{
    FlushRx(hservo);
    WriteBuf(hservo, ID, 0, NULL, 0, INST_REG_ACTION);
    return Ack(hservo, ID);
}

int STS3215_WriteByte(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 bDat)
{
    FlushRx(hservo);
    WriteBuf(hservo, ID, MemAddr, &bDat, 1, INST_WRITE);
    return Ack(hservo, ID);
}

int STS3215_WriteWord(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u16 wDat)
{
    u8 bBuf[2];
    Host2SCS(hservo, bBuf+0, bBuf+1, wDat);
    FlushRx(hservo);
    WriteBuf(hservo, ID, MemAddr, bBuf, 2, INST_WRITE);
    return Ack(hservo, ID);
}

int STS3215_Read(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr, u8 *nData, u8 nLen)
{
    FlushRx(hservo);
    WriteBuf(hservo, ID, MemAddr, &nLen, 1, INST_READ);

    if(! CheckHead(hservo)) {
        return 0;
    }

    u8 bBuf[4];
    hservo->Error = 0;

    if(HAL_UART_Receive(hservo->huart, bBuf, 3, hservo->IOTimeOut) != HAL_OK) {
        return 0;
    }

    if(HAL_UART_Receive(hservo->huart, nData, nLen, hservo->IOTimeOut) != HAL_OK) {
        return 0;
    }

    if(HAL_UART_Receive(hservo->huart, bBuf+3, 1, hservo->IOTimeOut) != HAL_OK) {
        return 0;
    }

    // Verify checksum
    u8 calSum = bBuf[0] + bBuf[1] + bBuf[2];
    for(u8 i = 0; i < nLen; i++) {
        calSum += nData[i];
    }
    calSum = ~calSum;

    if(calSum != bBuf[3]) {
        return 0;
    }

    hservo->Error = bBuf[2];
    return nLen;
}

int STS3215_ReadByte(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr)
{
    u8 bDat;
    int Size = STS3215_Read(hservo, ID, MemAddr, &bDat, 1);
    if(Size != 1) {
        return -1;
    }
    return bDat;
}

int STS3215_ReadWord(STS3215_HandleTypeDef *hservo, u8 ID, u8 MemAddr)
{
    u8 nDat[2];
    int Size = STS3215_Read(hservo, ID, MemAddr, nDat, 2);
    if(Size != 2) {
        return -1;
    }
    return SCS2Host(hservo, nDat[0], nDat[1]);
}

/*===========================================================================*/
/*                      POSITION CONTROL FUNCTIONS                           */
/*===========================================================================*/

int STS3215_WritePosEx(STS3215_HandleTypeDef *hservo, u8 ID, s16 Position, u16 Speed, u8 ACC)
{
    if(Position < 0) {
        Position = -Position;
        Position |= (1 << 15);
    }

    u8 bBuf[7];
    bBuf[0] = ACC;
    Host2SCS(hservo, bBuf+1, bBuf+2, Position);
    Host2SCS(hservo, bBuf+3, bBuf+4, 0);  // Time = 0
    Host2SCS(hservo, bBuf+5, bBuf+6, Speed);

    return STS3215_GenWrite(hservo, ID, STS_ACC, bBuf, 7);
}

int STS3215_RegWritePosEx(STS3215_HandleTypeDef *hservo, u8 ID, s16 Position, u16 Speed, u8 ACC)
{
    if(Position < 0) {
        Position = -Position;
        Position |= (1 << 15);
    }

    u8 bBuf[7];
    bBuf[0] = ACC;
    Host2SCS(hservo, bBuf+1, bBuf+2, Position);
    Host2SCS(hservo, bBuf+3, bBuf+4, 0);
    Host2SCS(hservo, bBuf+5, bBuf+6, Speed);

    return STS3215_RegWrite(hservo, ID, STS_ACC, bBuf, 7);
}

void STS3215_SyncWritePosEx(STS3215_HandleTypeDef *hservo, u8 ID[], u8 IDN,
                            s16 Position[], u16 Speed[], u8 ACC[])
{
    u8 offbuf[7 * IDN];

    for(u8 i = 0; i < IDN; i++) {
        s16 pos = Position[i];
        if(pos < 0) {
            pos = -pos;
            pos |= (1 << 15);
        }

        u16 V = Speed ?  Speed[i] :  0;
        offbuf[i*7] = ACC ?  ACC[i] :  0;

        Host2SCS(hservo, offbuf+i*7+1, offbuf+i*7+2, pos);
        Host2SCS(hservo, offbuf+i*7+3, offbuf+i*7+4, 0);
        Host2SCS(hservo, offbuf+i*7+5, offbuf+i*7+6, V);
    }

    // Build sync write packet
    FlushRx(hservo);

    u8 mesLen = ((7+1) * IDN + 4);
    u8 Sum = 0;
    u8 bBuf[7];

    bBuf[0] = 0xFF;
    bBuf[1] = 0xFF;
    bBuf[2] = STS_BROADCAST_ID;
    bBuf[3] = mesLen;
    bBuf[4] = INST_SYNC_WRITE;
    bBuf[5] = STS_ACC;
    bBuf[6] = 7;  // Data length per servo

    HAL_HalfDuplex_EnableTransmitter(hservo->huart);
    HAL_UART_Transmit(hservo->huart, bBuf, 7, 100);

    Sum = STS_BROADCAST_ID + mesLen + INST_SYNC_WRITE + STS_ACC + 7;

    for(u8 i = 0; i < IDN; i++) {
        HAL_UART_Transmit(hservo->huart, &ID[i], 1, 100);
        HAL_UART_Transmit(hservo->huart, offbuf + i*7, 7, 100);
        Sum += ID[i];
        for(u8 j = 0; j < 7; j++) {
            Sum += offbuf[i*7 + j];
        }
    }

    Sum = ~Sum;
    HAL_UART_Transmit(hservo->huart, &Sum, 1, 100);
}

/*===========================================================================*/
/*                        WHEEL MODE FUNCTIONS                               */
/*===========================================================================*/

int STS3215_WheelMode(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_WriteByte(hservo, ID, STS_MODE, 1);
}

int STS3215_ServoMode(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_WriteByte(hservo, ID, STS_MODE, 0);
}

int STS3215_WriteSpe(STS3215_HandleTypeDef *hservo, u8 ID, s16 Speed, u8 ACC)
{
    if(Speed < 0) {
        Speed = -Speed;
        Speed |= (1 << 15);
    }

    u8 bBuf[1] = {ACC};
    STS3215_GenWrite(hservo, ID, STS_ACC, bBuf, 1);

    u8 sBuf[2];
    Host2SCS(hservo, sBuf+0, sBuf+1, Speed);

    return STS3215_GenWrite(hservo, ID, STS_GOAL_SPEED_L, sBuf, 2);
}

/*===========================================================================*/
/*                        TORQUE CONTROL FUNCTIONS                           */
/*===========================================================================*/

int STS3215_EnableTorque(STS3215_HandleTypeDef *hservo, u8 ID, u8 Enable)
{
    return STS3215_WriteByte(hservo, ID, STS_TORQUE_ENABLE, Enable);
}

/*===========================================================================*/
/*                        EPROM LOCK FUNCTIONS                               */
/*===========================================================================*/

int STS3215_UnLockEprom(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_WriteByte(hservo, ID, STS_LOCK, 0);
}

int STS3215_LockEprom(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_WriteByte(hservo, ID, STS_LOCK, 1);
}

/*===========================================================================*/
/*                      CALIBRATION FUNCTIONS                                */
/*===========================================================================*/

int STS3215_CalibrationOfs(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_WriteByte(hservo, ID, STS_TORQUE_ENABLE, 128);
}

/*===========================================================================*/
/*                        READ FUNCTIONS                                     */
/*===========================================================================*/

int STS3215_Ping(STS3215_HandleTypeDef *hservo, u8 ID)
{
    FlushRx(hservo);
    WriteBuf(hservo, ID, 0, NULL, 0, INST_PING);

    hservo->Error = 0;
    if(! CheckHead(hservo)) {
        return -1;
    }

    u8 bBuf[4];
    if(HAL_UART_Receive(hservo->huart, bBuf, 4, hservo->IOTimeOut) != HAL_OK) {
        return -1;
    }

    if(bBuf[0] != ID && ID != STS_BROADCAST_ID) {
        return -1;
    }
    if(bBuf[1] != 2) {
        return -1;
    }

    u8 calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
    if(calSum != bBuf[3]) {
        return -1;
    }

    hservo->Error = bBuf[2];
    return bBuf[0];
}

int STS3215_FeedBack(STS3215_HandleTypeDef *hservo, u8 ID, STS3215_FeedbackTypeDef *feedback)
{
    u8 Mem[STS_PRESENT_CURRENT_H - STS_PRESENT_POSITION_L + 1];

    int nLen = STS3215_Read(hservo, ID, STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
    if(nLen != sizeof(Mem)) {
        return -1;
    }

    // Parse position
    s16 pos = SCS2Host(hservo, Mem[0], Mem[1]);
    if(pos & (1 << 15)) {
        pos = -(pos & ~(1 << 15));
    }
    feedback->Position = pos;

    // Parse speed
    s16 spd = SCS2Host(hservo, Mem[2], Mem[3]);
    if(spd & (1 << 15)) {
        spd = -(spd & ~(1 << 15));
    }
    feedback->Speed = spd;

    // Parse load
    s16 load = SCS2Host(hservo, Mem[4], Mem[5]);
    if(load & (1 << 10)) {
        load = -(load & ~(1 << 10));
    }
    feedback->Load = load;

    // Voltage and temperature
    feedback->Voltage = Mem[STS_PRESENT_VOLTAGE - STS_PRESENT_POSITION_L];
    feedback->Temperature = Mem[STS_PRESENT_TEMPERATURE - STS_PRESENT_POSITION_L];

    // Moving status
    feedback->Moving = Mem[STS_MOVING - STS_PRESENT_POSITION_L];

    // Current
    s16 curr = SCS2Host(hservo, Mem[STS_PRESENT_CURRENT_L - STS_PRESENT_POSITION_L],
                                Mem[STS_PRESENT_CURRENT_H - STS_PRESENT_POSITION_L]);
    if(curr & (1 << 15)) {
        curr = -(curr & ~(1 << 15));
    }
    feedback->Current = curr;

    return nLen;
}

int STS3215_ReadPos(STS3215_HandleTypeDef *hservo, u8 ID)
{
    int Pos = STS3215_ReadWord(hservo, ID, STS_PRESENT_POSITION_L);
    if(Pos == -1) {
        return -1;
    }
    if(Pos & (1 << 15)) {
        Pos = -(Pos & ~(1 << 15));
    }
    return Pos;
}

int STS3215_ReadSpeed(STS3215_HandleTypeDef *hservo, u8 ID)
{
    int Speed = STS3215_ReadWord(hservo, ID, STS_PRESENT_SPEED_L);
    if(Speed == -1) {
        return -32768;
    }
    if(Speed & (1 << 15)) {
        Speed = -(Speed & ~(1 << 15));
    }
    return Speed;
}

int STS3215_ReadLoad(STS3215_HandleTypeDef *hservo, u8 ID)
{
    int Load = STS3215_ReadWord(hservo, ID, STS_PRESENT_LOAD_L);
    if(Load == -1) {
        return -32768;
    }
    if(Load & (1 << 10)) {
        Load = -(Load & ~(1 << 10));
    }
    return Load;
}

int STS3215_ReadVoltage(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_ReadByte(hservo, ID, STS_PRESENT_VOLTAGE);
}

int STS3215_ReadTemper(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_ReadByte(hservo, ID, STS_PRESENT_TEMPERATURE);
}

int STS3215_ReadMove(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_ReadByte(hservo, ID, STS_MOVING);
}

int STS3215_ReadCurrent(STS3215_HandleTypeDef *hservo, u8 ID)
{
    int Current = STS3215_ReadWord(hservo, ID, STS_PRESENT_CURRENT_L);
    if(Current == -1) {
        return -32768;
    }
    if(Current & (1 << 15)) {
        Current = -(Current & ~(1 << 15));
    }
    return Current;
}

int STS3215_ReadMode(STS3215_HandleTypeDef *hservo, u8 ID)
{
    return STS3215_ReadByte(hservo, ID, STS_MODE);
}

/*===========================================================================*/
/*                    ID AND BAUD RATE CONFIGURATION                         */
/*===========================================================================*/

int STS3215_SetID(STS3215_HandleTypeDef *hservo, u8 oldID, u8 newID)
{
    STS3215_UnLockEprom(hservo, oldID);
    int result = STS3215_WriteByte(hservo, oldID, STS_ID, newID);
    STS3215_LockEprom(hservo, newID);
    return result;
}

int STS3215_SetBaudRate(STS3215_HandleTypeDef *hservo, u8 ID, u8 baudRateIndex)
{
    STS3215_UnLockEprom(hservo, ID);
    int result = STS3215_WriteByte(hservo, ID, STS_BAUD_RATE, baudRateIndex);
    STS3215_LockEprom(hservo, ID);
    return result;
}
