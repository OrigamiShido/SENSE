#include "ti_msp_dl_config.h"
#include "oled.h"
#include <eeprom_emulation_type_a.h>
#include "I2C_communication.h"
#include "msp6050control.h"

//DEFINE

//OLED PARAMETER
uint8_t TxPacket[4] = {0x90, 0x00, 0x00, 0x00};  //��������
uint8_t RxPacket[4]={0x00, 0x00, 0x00, 0x00};   //��������
uint8_t RxTemp; //��ʱ���ݣ���ս���FIFO��

uint32_t EEPROMEmulationBuffer[EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t)]={0};

uint8_t data=0;
uint8_t dataflow[50]={0};
uint8_t i=0;
int main(void)
{
	//VARIABLES
	unsigned int status=114514;
	unsigned int lastnumber=0;

	uint32_t EEPROMEmulationState;
	uint8_t week=9;
	SYSCFG_DL_init();
	//timer enabler

	NVIC_ClearPendingIRQ(UART0_INT_IRQn);
	NVIC_EnableIRQ(UART0_INT_IRQn);

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
	//EEPROM_TypeA_eraseAllSectors();
    //EEPROMEmulationState = EEPROM_TypeA_init(&EEPROMEmulationBuffer[0]);
	//DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX,8);
	while (1) 
	{

    }
}

void  UART0_IRQHandler()
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

void TIMER_0_INST_IRQHandler (void){
	DL_UART_Main_transmitDataBlocking(UART0,i);
	i++;
}