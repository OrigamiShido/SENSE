#include "ti_msp_dl_config.h"
#include "oled.h"
#include <eeprom_emulation_type_a.h>
#include "I2C_communication.h"

//DEFINE
#define R 0 //Read
#define W 1 //Write
#define W2 2 //write data with two bytes
#define X 3
#define Y 4
#define Z 5
#define ADDRESS 0x9000
#define FULLRANGE 2

//memory register
#define SMPLRT_DIV 0X19
#define CONFIG 0X1A
#define GYRO_CONFIG 0X1B
#define ACCEL_CONFIG 0X1C

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

#define PWR_MGMT_1 0X6B
#define PWR_MGMT_2 0X6C
#define WHO_AM_I 0X75

//OLED PARAMETER
uint8_t TxPacket[4] = {0x90, 0x00, 0x00, 0x00};  //��������
uint8_t RxPacket[4]={0x00, 0x00, 0x00, 0x00};   //��������
uint8_t RxTemp; //��ʱ���ݣ���ս���FIFO��

//FLASH PARAMETER
uint32_t EEPROMEmulationBuffer[EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t)]={0};

//BLUETOOTH PARAMETER
uint8_t data=0;
uint8_t dataflow[50]={0};
uint8_t i=0;

//I2C PARAMETER
int accx=0,accy=0,accz=0;
int accshowx=0,accshowy=0,accshowz=0;
uint8_t* halfint=NULL;
unsigned char RX_data[2] = {0x00};

//OTHERS
bool ischanged=true;

//FUNCTIONS
void msp6050_readacc(uint8_t command);
void DirectCommands(uint8_t command, uint8_t data, uint8_t type);
void delayUS(uint16_t us);
void readacc(void);
void save();
int read();
void Displayacc(void);
void Computeacc(void);
void msp6050Init(void);
void msp6050Shut(void);

int main(void)
{
	//VARIABLES
	unsigned int status=114514;
	unsigned int lastnumber=0;

	uint32_t EEPROMEmulationState;
	uint8_t week=9;
	SYSCFG_DL_init();

	NVIC_ClearPendingIRQ(UART0_INT_IRQn);
	NVIC_EnableIRQ(UART0_INT_IRQn);

    //6050 INIT
    msp6050Init();

    //timer enabler
	DL_TimerG_startCounter(TIMER_0_INST);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	
    //OLED self test
	OLED_Init();
	OLED_Clear();
	/*OLED_ShowCHinese(0,0,7);//
	OLED_ShowCHinese(18,0,8);//
	OLED_ShowCHinese(36,0,9);
  	OLED_ShowNum(0,2,100,3,18);
	OLED_ShowString(1,4,"2022302121246");
  	OLED_ShowString(2,6,"Notepad sys");
	//delay_ms(100000);this expression may be wrong! the sysclk doesn't recognize this function in right way!
	delay_cycles(120000000);
	OLED_Clear();*/

    //FLASH INIT TEMP
	EEPROM_TypeA_eraseAllSectors();
    EEPROMEmulationState = EEPROM_TypeA_init(&EEPROMEmulationBuffer[0]);

	//DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX,8);
	while (1) 
	{
        if(ischanged)
        {
            Computeacc();
            Displayacc();
            //wait to be inserted
            ischanged=false;
        }
        
    }
}

void  UART0_IRQHandler()//UART中断
{
   switch (DL_UART_Main_getPendingInterrupt(UART0)) //����Ƿ񴮿��ж�
		{
        case DL_UART_MAIN_IIDX_RX:
            data = DL_UART_Main_receiveData(UART0);  //���ͽ��յ�������
			i=data;
            DL_UART_Main_transmitDataBlocking(UART0, data);
            break;
        default:
            break;
		}
}

void TIMER_0_INST_IRQHandler (void){//定时器中断
	DL_UART_Main_transmitDataBlocking(UART0,i);
	i++;
    readacc();
    ischanged=true;
}

void Displayacc()
{
    OLED_Clear();
    OLED_ShowString(0,0,"accx:");
    OLED_ShowNum(48,0,accx,5,16);//need to be revised
    OLED_ShowString(0,2,"accy:");
    OLED_ShowNum(48,2,accy,5,16);
    OLED_ShowString(0,4,"accz:");
    OLED_ShowNum(48,4,accz,5,16);
}

void Computeacc(void)
{
    accshowx=accx;
    accshowy=accy;
    accshowz=accz;
    accshowx=accshowx*FULLRANGE*10000/32768;//2g=20000
    accshowy=accshowy*FULLRANGE*10000/32768;
    accshowz=accshowz*FULLRANGE*10000/32768;
    return;
}

void msp6050Init(void)
{
    DirectCommands(PWR_MGMT_1,0x00,W);
    DirectCommands(SMPLRT_DIV,0x07,W);//(?)
    DirectCommands(CONFIG,0x06,W);
    DirectCommands(GYRO_CONFIG,0x18,W);
    DirectCommands(ACCEL_CONFIG,0x01,W);
    return;
}

void msp6050Shut(void)
{
    DirectCommands(PWR_MGMT_1,0x40,W);
    return;
}

void readacc(void)//读加速度
{
    msp6050_readacc(X);
    msp6050_readacc(Y);
    msp6050_readacc(Z);
    return;
}

void msp6050_readacc(uint8_t command)//分函数
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

void DirectCommands(uint8_t command, uint8_t data, uint8_t type)//基础函数
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{   //type: R = read, W = write
    uint8_t TX_data = 0x00;

    //little endian format
    TX_data = data; 

    if (type == R) {//Read
        I2C_ReadReg(command, RX_data, 2); //RX_data is a global variable
//        delay_cycles(64000);delay_cycles(64000);
//        delay_cycles(40000); // for 400k Test
        delayUS(2000);delayUS(2000);//success in 100k
    }
    if (type == W) {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
        I2C_WriteReg(command,TX_data,1);
        delayUS(2000);delayUS(2000);//(?)
    }
}

void delayUS(uint16_t us) {   // Sets the delay in microseconds.
    uint16_t ms;
    char i;
    ms = us / 1000;
    for(i=0; i< ms; i++)
        delay_cycles(32000);
}

void save()
{
	uint32_t dataarray[EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t)]={0};
	EEPROM_TypeA_eraseAllSectors();
	for(unsigned int i=0;i<EEPROM_EMULATION_DATA_SIZE/sizeof(uint32_t);i++)
		dataarray[i]=0;
	dataarray[0]=accx;
    dataarray[1]=accy;
    dataarray[2]=accz;
	DL_FlashCTL_unprotectSector( FLASHCTL, ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);
	DL_FlashCTL_programMemoryFromRAM( FLASHCTL, ADDRESS, dataarray, 3, DL_FLASHCTL_REGION_SELECT_MAIN);
	// DL_FlashCTL_programMemoryFromRAM( FLASHCTL, ADDRESS, dataarray, 6, DL_FLASHCTL_REGION_SELECT_MAIN);
}

int read()//unfinished
{
	uint32_t address=ADDRESS;
	for(unsigned int i=0;i<EEPROM_EMULATION_DATA_SIZE/sizeof(uint32_t);i++)
		EEPROMEmulationBuffer[i]=0;
	for(int i=0;(*(uint32_t *)(address+i))!=0xFFFFFFFF;i+=4)
		EEPROMEmulationBuffer[i/4]=*(uint32_t *)(address+i);
	int result=EEPROMEmulationBuffer[0];
	return result;	
}


/*
target
1.两个以上的外部中断
2.读取加速度显示（量程那个？）
3.显示
4.uart调试内容发送
5.i2c读取

6.蓝牙发送应答(功能测试成功)
7.硬件i2c//注意所有的寄存器一开始都是0，芯片睡眠
8.flash存储
*/