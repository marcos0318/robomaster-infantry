#include "main.h"
#include "function_list.h"

volatile u32 ticks_msimg = (u32)-1;

void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	tft_init(2,BLACK,GREEN,GREEN);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
	ENCODER_Init();
	GUN_Init();
}

int main(void)
{	
	init();

	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			if (ticks_msimg % 20 == 0)
				GUN_PokeControl();

			//TODO: nothing:-)
			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				tft_prints(0,2,"Infantry V1.3");
				tft_prints(0,3,"%d",ticks_msimg);
				
				tft_update();
				LED_blink(LED1);
			}			
			
		}
	}	
}	

	



