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
#define GYROFULLRANGE 250

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
#define GYRO_ZOUT_L 0X48

#define PWR_MGMT_1 0X6B
#define PWR_MGMT_2 0X6C
#define WHO_AM_I 0X75

//OLED PARAMETER
uint8_t TxPacket[4] = {0x90, 0x00, 0x00, 0x00};  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
uint8_t RxPacket[4]={0x00, 0x00, 0x00, 0x00};   //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
uint8_t RxTemp; //ï¿½ï¿½Ê±ï¿½ï¿½ï¿½Ý£ï¿½ï¿½ï¿½Õ½ï¿½ï¿½ï¿½FIFOï¿½ï¿½

//FLASH PARAMETER
uint32_t EEPROMEmulationBuffer[EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t)]={0};

uint8_t i=0;

//BLUETOOTH PARAMETER
uint8_t data=0;

//I2C PARAMETER
unsigned char RX_data[1] = {0x00};
struct accdata{
    short accx;
    short accy;
    short accz;
    short accshowx;
    short accshowy;
    short accshowz;
}acc={0,0,0,0,0,0};
struct gyrodata{
    short gyrox;
    short gyroy;
    short gyroz;
    short gyroshowx;
    short gyroshowy;
    short gyroshowz;
}gyro={0,0,0,0,0,0};

struct judgepack{
    bool isacc;
    bool isgyro;
    bool isacccalced;
    bool isgyrocalced;
    bool isstarted;
    bool isclear;
    bool screenplay;
}judge={true,false,true,true,true,true,true};

//OTHERS
bool ischanged=true;

//FUNCTIONS
void msp6050_readacc(uint8_t command);
void msp6050_readgyro(uint8_t command);
void DirectCommands(uint8_t command, uint8_t data, uint8_t type);
void delayUS(uint16_t us);
void readacc(void);
void save();
int read();
void Displayacc(void);
void Computeacc(void);
void msp6050Init(void);
void msp6050Shut(void);
void TransformtoFloat(uint8_t x,uint8_t y,int num,uint8_t intergercount);
void transmituartdata(char addchar, short data);
void Computegyro(void);
void Displaygyro(void);
void readgyro(void);

int main(void)
{
	//VARIABLES
	//uint32_t EEPROMEmulationState;
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

    //FLASH INIT TEMP
	EEPROM_TypeA_eraseAllSectors();
    //EEPROMEmulationState = EEPROM_TypeA_init(&EEPROMEmulationBuffer[0]);

	//DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX,8);
	while (1) 
	{
        if(ischanged)
        {
            if(judge.isclear)
            {
                OLED_Clear();
                judge.isclear=false;
            }
            Computeacc();
            if(judge.isacc)
            {
                if(judge.isacccalced)
                {
                    transmituartdata('x',acc.accshowx);
                    transmituartdata('y',acc.accshowy);
                    transmituartdata('z',acc.accshowz);
                }
                else
                {
                    transmituartdata('x',acc.accx);
                    transmituartdata('y',acc.accy);
                    transmituartdata('z',acc.accz);
                }
            }
            if(judge.screenplay)
            {
                Displayacc();
            }
            Computegyro();
            if(judge.isgyro)
            {
                if(judge.isgyrocalced)
                {
                    transmituartdata('a',gyro.gyroshowx);
                    transmituartdata('b',gyro.gyroshowy);
                    transmituartdata('c',gyro.gyroshowz);
                }
                else
                {
                    transmituartdata('a',gyro.gyrox);
                    transmituartdata('b',gyro.gyroy);
                    transmituartdata('c',gyro.gyroz);
                }
            }
            if(!judge.screenplay)
            {
                Displaygyro();
            }
            ischanged=false;
        }
    }
}

void  UART0_IRQHandler()//UARTä¸­æ–­
{
   switch (DL_UART_Main_getPendingInterrupt(UART0)) //ï¿½ï¿½ï¿½ï¿½Ç·ñ´®¿ï¿½ï¿½Ð¶ï¿½
		{
        case DL_UART_MAIN_IIDX_RX:
            data = DL_UART_Main_receiveData(UART0);  //ï¿½ï¿½ï¿½Í½ï¿½ï¿½Õµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
			switch(data)
            {
                case 'A':judge.isacc=!judge.isacc;break;
                case 'B':judge.isgyro=!judge.isgyro;break;
                case 'C':break;
                case 'D':judge.isacccalced=!judge.isacccalced;break;
                case 'E':judge.isgyrocalced=!judge.isgyrocalced;break;
                case 'F':break;
                case 'G':break;
                case 'H':judge.screenplay=true;judge.isclear=true;break;
                case 'I':if(judge.isstarted)
                {
                    msp6050Shut();
                    judge.isstarted=false;
                }
                else
                {
                    msp6050Init();
                    judge.isstarted=true;
                }break;
                case 'J':judge.screenplay=false;judge.isclear=true;break;
                case 'K':break;
            }
            DL_UART_Main_transmitDataBlocking(UART0, data);
            break;
        default:
            break;
		}
}

void TIMER_0_INST_IRQHandler(void){//å®šæ—¶å™¨ä¸­æ–­
    readacc();
    readgyro();
    i++;
    if(i==10)
    {
        save();
        i=0;
    }
    ischanged=true;
}

void transmituartdata(char addchar, short data)
{
    uint8_t transmitdata[2]={data>>8,(uint8_t)data};
    DL_UART_Main_transmitDataBlocking(UART1,addchar);
    DL_UART_Main_transmitDataBlocking(UART1,transmitdata[0]);
    DL_UART_Main_transmitDataBlocking(UART1,transmitdata[1]);
    DL_UART_Main_transmitDataBlocking(UART1,'\n');
}

void Displayacc(void)
{
    //OLED_Clear();
    OLED_ShowString(0,0,"accx:");
    TransformtoFloat(48,0,acc.accshowx,1);
    OLED_ShowString(0,2,"accy:");
    TransformtoFloat(48,2,acc.accshowy,1);
    OLED_ShowString(0,4,"accz:");
    TransformtoFloat(48,4,acc.accshowz,1);
}

void Displaygyro(void)
{
    OLED_ShowString(0,0,"gyrox:");
    TransformtoFloat(56,0,gyro.gyroshowx,3);
    OLED_ShowString(0,2,"gyroy:");
    TransformtoFloat(56,2,gyro.gyroshowy,3);
    OLED_ShowString(0,4,"gyroz:");
    TransformtoFloat(56,4,gyro.gyroshowz,3);
}

void TransformtoFloat(uint8_t x,uint8_t y,int num,uint8_t intergercount)
{
    char nums[10]={'0','1','2','3','4','5','6','7','8','9'};
    switch(intergercount)
    {
        case 1:
			if(num<0)
            {
                OLED_ShowChar(x,y,'-');
                num=-num;
            }
            else
            {
                OLED_ShowChar(x,y,' ');
            }
            x+=8;
            OLED_ShowChar(x,y,nums[num/10000]);
            OLED_ShowChar(x+8,y,'.');
            OLED_ShowChar(x+16,y,nums[(num%10000)/1000]);
            OLED_ShowChar(x+24,y,nums[(num%1000)/100]);
            OLED_ShowChar(x+32,y,nums[(num%100)/10]);
            OLED_ShowChar(x+40,y,nums[num%10]);		
        break;
        case 2:break;//åº”è¯¥ä¸ä¼šç”¨åˆ°10gä»¥ä¸Šçš„æ»¡é‡ç¨‹ï¼Œä¸è¿‡é¢„ç•™äº†æ”¹è£…å£
        case 3:
        	if(num<0)
            {
                OLED_ShowChar(x,y,'-');
                num=-num;
            }
            else
            {
                OLED_ShowChar(x,y,' ');
            }
            x+=8;
            OLED_ShowChar(x,y,nums[num/10000]);
            OLED_ShowChar(x+8,y,nums[(num%10000)/1000]);
            OLED_ShowChar(x+16,y,nums[(num%1000)/100]);
            //OLED_ShowChar(x+24,y,'.');
            OLED_ShowChar(x+32,y,nums[(num%100)/10]);
            OLED_ShowChar(x+40,y,nums[num%10]);	
        break;
        default:break;
    }
}

void Computegyro(void)
{
    gyro.gyroshowx=gyro.gyrox;
    gyro.gyroshowy=gyro.gyroy;
    gyro.gyroshowz=gyro.gyroz;
    gyro.gyroshowx=gyro.gyroshowx*GYROFULLRANGE*100/32768;//25000 is 250
    gyro.gyroshowy=gyro.gyroshowy*GYROFULLRANGE*100/32768;
    gyro.gyroshowz=gyro.gyroshowz*GYROFULLRANGE*100/32768;
    return;

}

void readgyro(void)
{
    msp6050_readgyro(X);
    msp6050_readgyro(Y);
    msp6050_readgyro(Z);
    return;
}

void Computeacc(void)
{
    acc.accshowx=acc.accx;
    acc.accshowy=acc.accy;
    acc.accshowz=acc.accz;
    acc.accshowx=acc.accshowx*FULLRANGE*10000/32768;//2g=20000
    acc.accshowy=acc.accshowy*FULLRANGE*10000/32768;
    acc.accshowz=acc.accshowz*FULLRANGE*10000/32768;
    return;
}

void msp6050Init(void)
{
    DirectCommands(PWR_MGMT_1,0x00,W);
    DirectCommands(SMPLRT_DIV,0x07,W);//(?)
    DirectCommands(CONFIG,0x06,W);
    DirectCommands(GYRO_CONFIG,0x00,W);
    DirectCommands(ACCEL_CONFIG,0x01,W);
    return;
}

void msp6050Shut(void)
{
    DirectCommands(PWR_MGMT_1,0x40,W);
    return;
}

void readacc(void)//è¯»åŠ é€Ÿåº¦
{
    msp6050_readacc(X);
    msp6050_readacc(Y);
    msp6050_readacc(Z);
    return;
}

void msp6050_readacc(uint8_t command)//åˆ†å‡½æ•°
{
    switch(command)
    {
        case X:
            
            DirectCommands(ACCEL_XOUT_H,0,R);
            acc.accx=RX_data[0]<<8;
            DirectCommands(ACCEL_XOUT_L,0,R);
            acc.accx+=RX_data[0];
            break;
        case Y:
            DirectCommands(ACCEL_YOUT_H,0,R);
            acc.accy=RX_data[0]<<8;
            DirectCommands(ACCEL_YOUT_L,0,R);
            acc.accy+=RX_data[0];
            break;
        case Z:
            DirectCommands(ACCEL_ZOUT_H,0,R);
            acc.accz=RX_data[0]<<8;
            DirectCommands(ACCEL_ZOUT_L,0,R);
            acc.accz+=RX_data[0];
            break;
    }
    return;
}

void msp6050_readgyro(uint8_t command)
{
    switch(command)
    {
        case X:
            DirectCommands(GYRO_XOUT_H,0,R);
            gyro.gyrox=RX_data[0]<<8;
            DirectCommands(GYRO_XOUT_L,0,R);
            gyro.gyrox+=RX_data[0];
            break;
        case Y:
            DirectCommands(GYRO_YOUT_H,0,R);
            gyro.gyroy=RX_data[0]<<8;
            DirectCommands(GYRO_YOUT_L,0,R);
            gyro.gyroy+=RX_data[0];
            break;
        case Z:
            DirectCommands(GYRO_ZOUT_H,0,R);
            gyro.gyroz=RX_data[0]<<8;
            DirectCommands(GYRO_ZOUT_L,0,R);
            gyro.gyroz+=RX_data[0];
            break;
    }
    return;
}

void DirectCommands(uint8_t command, uint8_t data, uint8_t type)//åŸºç¡€å‡½æ•°
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{   //type: R = read, W = write
    uint8_t TX_data[1] = {0x00};

    //little endian format
    TX_data[0] = data; 

    if (type == R) {//Read
        I2C_ReadReg(command, RX_data, 1); //RX_data is a global variable
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
	dataarray[0]=acc.accx;
    dataarray[1]=acc.accy;
    dataarray[2]=acc.accz;
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
目标：
外部中断done
加速度读取done
显示done
uart发送done
i2c读取done
蓝牙控制done
硬件i2cdone
flash读取

*/