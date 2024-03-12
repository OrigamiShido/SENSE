#include "I2C_communication.h"

#define R 0 //Read
#define W 1 //Write
#define W2 2 //write data with two bytes
#define X 3
#define Y 4
#define Z 5

//memory register
#define ACCEL_XOUT_H 0X3B
#define ACCEL_XOUT_L 0X3C
#define ACCEL_YOUT_H 0X3D
#define ACCEL_YOUT_L 0X3E
#define ACCEL_ZOUT_H 0X3F
#define ACCEL_ZOUT_L 0X40
#define TEMP_OUT_H 0X41
#define TEMP_OUT_L 0X42
#define GYRO_XOUT_H 0X43
#define GYRO_XOUT_L 0X44
#define GYRO_YOUT_H 0X45
#define GYRO_YOUT_L 0X46
#define GYRO_ZOUT_H 0X47

void msp6050_readacc(uint8_t command);
void DirectCommands(uint8_t command, uint16_t data, uint8_t type);
void delayUS(uint16_t us);