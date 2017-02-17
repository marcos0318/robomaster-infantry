#include "main.h"
#include "function_list.h"

static u32 ticks_msimg = (u32)-1;

typedef struct{
	signed int setpoint;
	int sum_error;
	float proportion;
	float integral;
	float derivative;
	int last_error;        //e[-1];
	int prev_error;        //e[-2]
}PID_t;

PID_t PIDG,PIDS,PIDS0;
const int MAX=3000;


void incPIDinit(void)
{
//front LEFT
	
 PIDG.sum_error=0;
 PIDG.last_error=0;
 PIDG.prev_error=0;
 PIDG.proportion=0;//constants
 PIDG.integral=0;	 //constants
 PIDG.derivative=0;//constants
 PIDG.setpoint=0;
	
	PIDS.sum_error=0;
 PIDS.last_error=0;
 PIDS.prev_error=0;
 PIDS.proportion=0;//constants
 PIDS.integral=0;	 //constants
 PIDS.derivative=0;//constants
 PIDS.setpoint=0;
	
	PIDS0.sum_error=0;
 PIDS0.last_error=0;
 PIDS0.prev_error=0;
 PIDS0.proportion=0;//constants
 PIDS0.integral=0;	 //constants
 PIDS0.derivative=0;//constants
 PIDS0.setpoint=0;
	
}


void speedincPIDinit(void){
	PIDS.sum_error=0;
 PIDS.last_error=0;
 PIDS.prev_error=0;
	}

void speed0incPIDinit(void){
	PIDS0.sum_error=0;
 PIDS0.last_error=0;
 PIDS0.prev_error=0;
 
	}
void PID_set()        //pp,ii,dd maybe those constants I've set
{
  PIDG.proportion=0.51;
	PIDG.integral=0.009;
	PIDG.derivative=0.4;
	
	PIDS.proportion=8.47;
	PIDS.integral=0.57;
	PIDS.derivative=8.47; 
	
	PIDS0.proportion=80.0;
	PIDS0.integral=7;
	PIDS0.derivative=100; 
}
void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
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
	Quad_Encoder_Configuration();
	Encoder_Start1();
	Encoder_Start2();
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
	incPIDinit();
}
float incPIDcalc(PID_t* PIDx,signed int nextpoint)
{
	int iError;
	float iincpid;
	iError=PIDx->setpoint-nextpoint;    //current error
	/*                                        //fisrt way to calculate increment
	iincpid = PIDx->proportion * iError       //e[k]
	- PIDx->integral * PIDx->last_error       //e[k-1]
	+ PIDx->derivative * PIDx->prev_error;
	
	*/
	
	iincpid=																	//second way to calculate increment
	PIDx->proportion * (iError-PIDx->last_error)
	+PIDx->integral*iError
	+PIDx->derivative * (iError-2 * PIDx->last_error + PIDx->prev_error);
	
	PIDx->prev_error = PIDx->last_error;			//save the error to be used in the next calculation
	PIDx->last_error = iError;
	return iincpid;
	
	
}
 void PID_setpoint(PID_t * PIDx,signed int setvalue)
 {
  PIDx->setpoint=setvalue; 
 }

int32_t ABS(float s){
	if(s<0) return (int32_t)(-s);
	else return (int32_t)s;
}


int main(void)
{	
//	u8 dum[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '\n'};
	init();
	int32_t setpoint=0;
	int32_t setpoint_prev=0;
	bool inc=true;
	int32_t setpoint_buffer=0;
	int32_t speed_tar=0;
	int32_t speed=0;
	int32_t speed0=0;
	PID_set();
	//int32_t total=0;
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			//buzzer_check();			
			
			//if (ticks_msimg%500==0)
				//DataMonitor_Send(dum, sizeof(dum));
			tft_clear();
			//TODO: nothing:-)
			if(ticks_msimg%20==0)
			{
				
				setpoint=DBUS_ReceiveData.mouse.xtotal*0.8;
				if(setpoint_buffer<setpoint) inc=true; else inc=false;
				if(inc) {setpoint_buffer+=300;if(setpoint_buffer>setpoint) setpoint_buffer=setpoint;}
				else {setpoint_buffer-=300; if(setpoint_buffer<setpoint) setpoint_buffer=setpoint;}
				if(setpoint_buffer>=setpoint) setpoint_buffer=setpoint;
				PID_setpoint(&PIDG,setpoint_buffer);
				PID_setpoint(&PIDS0,0);
				//PID_setpoint(&PID,50);
				speed_tar+=incPIDcalc(&PIDG,(int32_t)(GMYawEncoder.ecd_angle));
				
				if(speed_tar>500) speed_tar=500;
				else if(speed_tar<-500) speed_tar=-500;
				PID_setpoint(&PIDS,speed_tar);
				//PID_setpoint(&PIDS,400);
				
				
				
				if(ABS(GMYawEncoder.ecd_angle-setpoint)<30&&ABS(GMYawEncoder.ecd_angle-setpoint_prev)<40){
					while((setpoint-setpoint_prev)==0){
						setpoint=DBUS_ReceiveData.mouse.xtotal*0.8;
						if(ABS(GMYawEncoder.ecd_angle-setpoint)>85) break;
						if (ticks_msimg != get_ms_ticks()){
							ticks_msimg=get_ms_ticks();
							speed0+=incPIDcalc(&PIDS0,GMYawEncoder.filter_rate);
							Set_CM_Speed(CAN1,speed0,0,0,0);
							if(ticks_msimg%20==0){
								for (int j=2;j<12;j++) tft_clear_line(j);
									tft_prints(1,2,"setpoint=%d",setpoint);
									tft_prints(1,3,"filter rate=%d",GMYawEncoder.filter_rate);
									tft_prints(1,4,"temp sp=%d",setpoint_buffer);
		
									tft_prints(1,5,"speed=%d",speed);
									tft_prints(1,6,"angle=%f",GMYawEncoder.ecd_angle);
									tft_prints(1,7,"speed_tar=%d",speed_tar);
									tft_prints(1,8,"mouse.xt=%d",DBUS_ReceiveData.mouse.xtotal);
									tft_prints(1,9,"mouse.yt=%d",DBUS_ReceiveData.mouse.ytotal);
				//tft_prints(1,10,"mouse.z=%d",DBUS_ReceiveData.mouse.z);
									tft_prints(1,10,"inside");
									tft_prints(1,11,"l=%d,r=%d",DBUS_ReceiveData.mouse.press_left,DBUS_ReceiveData.mouse.press_right);
									tft_update();		
						}
						setpoint_prev=setpoint;
					}
					}
					speedincPIDinit();
					speed=0;
				}
				else {
					speed+=incPIDcalc(&PIDS,GMYawEncoder.filter_rate);
					Set_CM_Speed(CAN1,speed,0,speed,speed);
					speed0=0;
					speed0incPIDinit();
				}
				
				
				for (int j=2;j<12;j++) tft_clear_line(j);
				tft_prints(1,2,"setpoint=%d",setpoint);
				tft_prints(1,3,"filter rate=%d",GMYawEncoder.filter_rate);
				tft_prints(1,4,"temp sp=%d",setpoint_buffer);
				
				tft_prints(1,5,"speed=%d",speed);
				tft_prints(1,6,"angle=%f",GMYawEncoder.ecd_angle);
  			tft_prints(1,7,"speed_tar=%d",speed_tar);
				tft_prints(1,8,"mouse.xt=%d",DBUS_ReceiveData.mouse.xtotal);
				tft_prints(1,9,"mouse.yt=%d",DBUS_ReceiveData.mouse.ytotal);
				//tft_prints(1,10,"mouse.z=%d",DBUS_ReceiveData.mouse.z);
				tft_prints(1,10,"outside");
				tft_prints(1,11,"l=%d,r=%d",DBUS_ReceiveData.mouse.press_left,DBUS_ReceiveData.mouse.press_right);
			tft_update();		
				LED_blink(LED1);
				setpoint_prev=setpoint;
			}			
			
		}
		//total+=CM1Encoder.filter_rate;
	}	
}	

	



