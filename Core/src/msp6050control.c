#include "msp6050control.h"
int accx=0,accy=0,accz=0;
uint8_t* halfint=NULL;
unsigned char RX_data[2] = {0x00};

void msp6050_readacc(uint8_t command)
{
    switch(command)
    {
        case X:
            halfint=&accx;
            DirectCommands(ACCEL_XOUT_H,0,R);
            *halfint=RX_data[1];
            halfint++;
            *halfint=RX_data[0];
            halfint++;
            DirectCommands(ACCEL_XOUT_L,0,R);
            *halfint=RX_data[1];
            halfint++;
            *halfint=RX_data[0];
            break;
        case Y:
            halfint=&accy;
            DirectCommands(ACCEL_YOUT_H,0,R);
            *halfint=RX_data[1];
            halfint++;
            *halfint=RX_data[0];
            halfint++;
            DirectCommands(ACCEL_YOUT_L,0,R);
            *halfint=RX_data[1];
            halfint++;
            *halfint=RX_data[0];
            break;
        case Z:
            halfint=&accz;
            DirectCommands(ACCEL_ZOUT_H,0,R);
            *halfint=RX_data[1];
            halfint++;
            *halfint=RX_data[0];
            halfint++;
            DirectCommands(ACCEL_ZOUT_L,0,R);
            *halfint=RX_data[1];
            halfint++;
            *halfint=RX_data[0];
            break;
    }
    halfint=NULL;
    return;
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{   //type: R = read, W = write
    uint8_t TX_data[2] = {0x00, 0x00};

    //little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R) {//Read
        I2C_ReadReg(command, RX_data, 2); //RX_data is a global variable
//        delay_cycles(64000);delay_cycles(64000);
//        delay_cycles(40000); // for 400k Test
        delayUS(2000);delayUS(2000);//success in 100k
    }
    if (type == W) {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
        I2C_WriteReg(command,TX_data,2);
        delayUS(2000);delayUS(2000);
    }
}

void delayUS(uint16_t us) {   // Sets the delay in microseconds.
    uint16_t ms;
    char i;
    ms = us / 1000;
    for(i=0; i< ms; i++)
        delay_cycles(32000);
}