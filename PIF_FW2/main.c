#include <stdint.h>
#include <stdbool.h>

#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/eeprom.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "global.h"

void Basic_Init(void);
void Interrupt_Init(void);
void Eeprom_Init(void);
void PWM_Init(void);
void Timer_Init(void);
void UART_Init(void);
void Delay_ms(uint16_t t);

void Check_Flag_Error(void);
void Blinky(uint8_t led_num, uint8_t is_fast);

void Button_Init_Handler(void)
{
	uint8_t  flag_int = GPIOIntStatus(Button_Base,true);
	GPIOIntClear(Button_Base,Button_1|Button_2);
	Delay_ms(10);
	if (((!GPIOPinRead(Button_Base,Button_1))&&(flag_int == Button_1))||((GPIOPinRead(Button_Base,Button_2))&&(flag_int == Button_2)))
	{
	switch(flag_state)
	{
	case idle:
		{
			if (flag_int == Button_1)
			{
				timer_cnt = 0;
				flag_timer_check = check_1s;
				TimerEnable(TIMER0_BASE,TIMER_A);
			}
			else // neu button2 nhan tuc la co loi
			{
				flag_state = error;
				EEPROMProgramNonBlocking(ERROR,0); // ghi co flag_err vao trong eeprom
			}
		}
	break;
	case run:
		{
			if (flag_int == Button_1)
			{
				flag_timer_check = check_1s;
				TimerEnable(TIMER0_BASE,TIMER_A);
			}
			else
			{
				flag_state = error;
				EEPROMProgramNonBlocking(ERROR,0); // ghi co flag_err vao trong eeprom
			}
		}
	break;
	case error:
		{
			if (flag_int == Button_1)	// nhan lan 1 bat che do kiem tra
			{
				if(check_double_cnt == 0)
				{
					TimerEnable(TIMER0_BASE,TIMER_A);
					flag_timer_check = check_double;
				}
				check_double_cnt ++;
			}
		}
	break;
	}
	}
}
void Timer0_Int_Handler(void)
{
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	switch(flag_timer_check)
	{
	case check_1s:
	{
		if (!GPIOPinRead(Button_Base,Button_1)) // neu nut 1 van con o muc 0
		{
			timer_cnt ++ ;
			if (timer_cnt > 800)
			{
				TimerDisable(TIMER0_BASE,TIMER_A);
				timer_cnt = 0;
				if(flag_state == idle) 	flag_state = run;
				else					flag_state = idle;
			}
		}
		else // neu nut 1 da duoc tha
		{

			TimerDisable(TIMER0_BASE,TIMER_A); // tat timer
			timer_cnt = 0;
		}
	}
	break;
	default:
	{
		timer_cnt ++ ;
		if((check_double_cnt == 2)&&(timer_cnt < 250)&&(timer_cnt>100)) // neu thoi gian nhan lien tiep be hon khoang 200ms
		{
			TimerDisable(TIMER0_BASE,TIMER_A);
			timer_cnt = 0;
			flag_state = idle;
			check_double_cnt = 0;
			EEPROMProgramNonBlocking(NON_ERROR,0); // xoa flag_err trong eeprom
		}
		else if((timer_cnt>250)&&(check_double_cnt == 1)) // neu kiem tra thay nut 2 chua duoc nhan ma thoi gian da qua 200 ms
		{
			check_double_cnt = 0;
			TimerDisable(TIMER0_BASE,TIMER_A);
			timer_cnt = 0;
		}
	}
	break;
	}
}



void main(void)
{
	Basic_Init();
	Interrupt_Init();
	UART_Init();
	UARTprintf("BAI TAP FW2\n");
	UARTDisable(UART0_BASE);
	Eeprom_Init();
	Timer_Init();
	uint8_t i_led ;
	for(i_led=0;i_led<4;i_led++)
		Blinky(Led_1,fast);
	Check_Flag_Error();
	while(1)
	{
		switch(flag_state)
		{
		case idle:
			{
				GPIOPinTypeGPIOOutput(Led_Base,Led_1|Led_2);
				GPIOPinWrite(Led_Base,Led_1|Led_2,0); // tat 2 led

				UARTEnable(UART0_BASE);
				UARTprintf("IDLE!!!\n");
				UARTprintf("Nhan giu bt1 trong hon 1s hoac goi '1' de chuyen sang run\n");

				while(flag_state == idle)
				{
					unsigned char flag_run = 0;
					if(UARTCharsAvail(UART0_BASE))
						flag_run = UARTgetc();
					if(flag_run == '1')
					{
						flag_state = run;
						UARTDisable(UART0_BASE);
						break;
					}
					//		flag_state = run;
					Blinky(Led_1,slow);
				}
			}
		break;
		case run:
			{
				uint8_t pulse_width = 1;
				uint8_t pulse_flag = 1;// bang 1 dem len, bang 0 dem xuong
				PWM_Init();
				GPIOPinWrite(Led_Base,Led_1,0xFF);

				UARTEnable(UART0_BASE);
				UARTprintf("RUN !!!\n");
				UARTDisable(UART0_BASE);

				while(flag_state == run)
				{
				PWMPulseWidthSet(PWM1_BASE,PWM_OUT_6,pulse_width);
				Delay_ms(50);
				if (pulse_flag)
					pulse_width ++;
				else
					pulse_width --;
				if (pulse_width == 100) pulse_flag = 0;
				if (pulse_width == 1) pulse_flag = 1;
				}
			}
		break;
		case error:
			{
				GPIOPinTypeGPIOOutput(Led_Base,Led_1|Led_2);
				GPIOPinWrite(Led_Base,Led_1|Led_2,0); // tat 2 led

				UARTEnable(UART0_BASE);
				UARTprintf("ERORR!!!\n");
				UARTDisable(UART0_BASE);

				while(flag_state == error)
				{
				Blinky(Led_1|Led_2,fast);
				}
			}
		break;
		}
	}
}
void Basic_Init(void)
{
	SysCtlClockSet(SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_SYSDIV_5); // dat tan so 40MHz
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	HWREG(Button_Base + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(Button_Base + GPIO_O_CR) = 0x01;
	GPIOPinTypeGPIOOutput(Led_Base,Led_1|Led_2);
	GPIOPinWrite(Led_Base,Led_1|Led_2,0); // tat 2 led
}
void Interrupt_Init(void)
{
	GPIODirModeSet(Button_Base,Button_1|Button_2,GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(Button_Base,Button_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(Button_Base,Button_2,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);

	GPIOIntEnable(Button_Base,Button_1|Button_2);

	GPIOIntTypeSet(Button_Base,Button_1,GPIO_FALLING_EDGE);
	GPIOIntTypeSet(Button_Base,Button_2,GPIO_RISING_EDGE);

	GPIOIntRegister(Button_Base,&Button_Init_Handler);
	IntMasterEnable();
}
void Timer_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC_UP);
	TimerLoadSet(TIMER0_BASE,TIMER_A,50000);
	// set bo dem 50000*200 xung tuc 0.25s
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	TimerIntRegister(TIMER0_BASE,TIMER_A,&Timer0_Int_Handler);

}
void Eeprom_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	EEPROMInit();
	
}
void Check_Flag_Error(void) // true co loi, false khong loi
{
	EEPROMRead(&flag_err,0,4);
	if(flag_err == ERROR) flag_state = error;
	else 			      flag_state = idle;
}
void Blinky(uint8_t led_num,uint8_t is_fast)
{
	uint16_t delay_time;
	if(is_fast)
		delay_time = 200;
	else
		delay_time = 1000;
	GPIOPinWrite(Led_Base,led_num,0);
	Delay_ms(delay_time);
	GPIOPinWrite(Led_Base,led_num,0xFF);
	Delay_ms(delay_time);
}
void PWM_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	GPIOPinTypePWM(Led_Base,Led_2);

	PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3,100);
	PWMPulseWidthSet(PWM1_BASE,PWM_OUT_6,1);
	PWMOutputState(PWM1_BASE,PWM_OUT_6_BIT,true);

	PWMGenEnable(PWM1_BASE,PWM_GEN_3);
}
void Delay_ms(uint16_t t)
{
	unsigned int i;
	for(i = 0;i<t;i++)
	{
		SysCtlDelay(SysCtlClockGet()/3000);
	}
}
void UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0|GPIO_PIN_1);

	UARTStdioConfig(0,115200,SysCtlClockGet());
}

