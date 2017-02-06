#include "main.h"
#include "function_list.h"


#define BUFFER_LENGTH 700
#define POWER_BUFFER_LENGTH 20
#define ANGLE_PID_LIMIT 2000
#define MOVING_BOUND_1 200
#define MOVING_BOUND_2 450

static u32 ticks_msimg = (u32)-1;

void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,WHITE,BLACK,BLACK);
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
}



//NORMAL MODE
//PID controls

//The first loop of pid 
	// Structure to strore PID data 
	struct pid_control_states states[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
	// Control loop input,output and setpoint variables
	int32_t wheel_setpoints[4] = {0,0,0,0};
	int32_t wheel_feedbacks[4] = {0,0,0,0};
	int32_t wheel_outputs[4] = {0,0,0,0};
	int32_t kp = 80, 
				ki = 4, 	
				kd = 1;

//The second loop of the PID
	struct pid_control_states state_angle = {0,0,0};
	int32_t setpoint_angle = 0;
	int32_t feedback_angle = 0;
	int32_t output_angle_speed = 0;
	
	int32_t kp_angle = 1;
	int32_t ki_angle = 0;
	int32_t kd_angle = 1;


/*the sudden change in setpoint of the motors
	will cause the sharp increase of the power
	so we need to buffer the wheel_setpoints
*/
	int32_t buffer[4][BUFFER_LENGTH] ;
	
	int32_t buffer_out(int32_t* b, int32_t length, int32_t counter){
		int32_t b_output = 0;
		for (int i = 0 ; i< length ; i++){
			b_output+= b[i];
		}
		b_output = b_output/ length;
		return b_output;
	}
	
	float buffer_out_f(float* b, int32_t length, int32_t counter){
		float b_output = 0;
		for (int i = 0 ; i< length ; i++){
			b_output+= b[i];
		}
		b_output = b_output/ length;
		return b_output;
	}
	
	
	void buffer_in(int32_t* b , int32_t length, int32_t counter, int32_t input ){
		int16_t index = counter%length ;
		b[index] = input;
	}
	
	void buffer_in_f(float* b , int32_t length, int32_t counter, int32_t input ){
		int16_t index = counter%length ;
		b[index] = input;
	}


	
	
// Motor setpoint adjusters, adjust the max setpoint at the same time, but keep the direction
int32_t abs(int32_t x){
		if (x < 0){
				return -x;
		}
		return x;
}
void wheel_setpoints_adjust(int32_t * sp1, int32_t* sp2, int32_t* sp3, int32_t* sp4, int32_t limit){
		int32_t max = abs(*sp1);
		if (abs(*sp2) > max) max = abs(*sp2);
		if (abs(*sp3) > max) max = abs(*sp3);
		if (abs(*sp4) > max) max = abs(*sp4);

		if (max > limit){ 
				*sp1 = *sp1 * limit  / max;
				*sp2 = *sp2 * limit  / max;
				*sp3 = *sp3 * limit  / max;
				*sp4 = *sp4 * limit  / max;			
		}
}	




//The power control, NOT intend to use pid control, but still consider using close-loop control 

int32_t FILTER_RATE_LIMIT = 1000;
//int32_t power_buffer[POWER_BUFFER_LENGTH];
float feedback_current = 0;
float feedback_voltage = 0;
int32_t work_target = 60;
float Pr = 0;
float PL = 80;
float W = 60;
int32_t W_int = 60;
int32_t work_pid_output = 0;
int32_t  wheel_setpoint_coefficient = 1000;
struct pid_control_states work_state = {0,0,0};

int32_t kp_power = 10;
int32_t ki_power = 3;
int32_t kd_power = 0;




int main(void)
{	
	init();
	
//init the buffers with zero 
	for (int i =0 ; i<4 ; i++){
		for (int j = 0; j< BUFFER_LENGTH; j++){
			buffer[i][j]=0;
		}
	}
	
	
	
	
//The control of gimbal
// The yaw pid
	int16_t current_yaw = 1000;
	
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			
			
			//TODDO: motor related control stuff,
			//			you can add any seperate file to do that. Most function that you need is in the function_list.h
			// 			the function_list.h can be found through the top of this main.c
				
      
			
			if (DBUS_ReceiveData.rc.switch_right == 3){

//power control 
      //imitate the process in judging system
			if (ticks_msimg%50==20){
					//getPr
		      feedback_current = InfantryJudge.RealCurrent; 
				  feedback_voltage = InfantryJudge.RealVoltage;
				  Pr = feedback_current*feedback_voltage;
				
				  W = W - (Pr -PL) * 0.02;
          if (W <= 0){
							W = 0;
						//妈的我血流满地了
					}				
				  else if (W > 60) {
						  W = 60;
					}
					W_int = W;
					work_pid_output = pid_process_gai1(&work_state, &work_target, &W_int ,kp_power,ki_power,kd_power ); 
					wheel_setpoint_coefficient = 1000 - work_pid_output;
					if (wheel_setpoint_coefficient < 200) {wheel_setpoint_coefficient = 200;}
				  if (wheel_setpoint_coefficient > 1000) {wheel_setpoint_coefficient = 1000;}
			}
				
				
			


			
			
//Second loop of pid (absolute angle control)
			int32_t speed_limitor  = 660;
      int32_t speed_multiplier = FILTER_RATE_LIMIT;
			int32_t angular_speed_limitor = 200;
			int32_t forward_speed =  (DBUS_ReceiveData.rc.ch1 + DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660)
			* speed_multiplier/speed_limitor ;
			int32_t right_speed =    (DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660)
			* speed_multiplier/speed_limitor ;
			//setpoint_angle += DBUS_ReceiveData.rc.ch2/angular_speed_limitor;	
			int32_t increment_of_angle = (DBUS_ReceiveData.rc.ch2 + DBUS_CheckPush(KEY_G)*660 - DBUS_CheckPush(KEY_F)*660)
			/angular_speed_limitor;
			//if the car is moving, slower the angle_setpoint change 
			if ( (abs(DBUS_ReceiveData.rc.ch1+ DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660)+abs(DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660))> MOVING_BOUND_1){
				 if(increment_of_angle > 2) increment_of_angle = 2;
				 else if (increment_of_angle < -2) increment_of_angle = -2;
				
			}
			
			if ( (abs(DBUS_ReceiveData.rc.ch1+ DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660 )+abs(DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660))> MOVING_BOUND_2){
				 if(increment_of_angle > 1) increment_of_angle = 1;
				 else if (increment_of_angle < -1) increment_of_angle = -1;
				
			}
			
			//Limit the angle setpoint by the FILTER_RATE_LIMIT we can only test the static time, but not the dynamic 
				if (FILTER_RATE_LIMIT < 501){
						if(increment_of_angle > 2) increment_of_angle = 2;
				    else if (increment_of_angle < -2) increment_of_angle = -2;
				}
				
				if (FILTER_RATE_LIMIT < 301){
						if(increment_of_angle > 1) increment_of_angle = 1;
				    else if (increment_of_angle < -1) increment_of_angle = -1;
				}
			
			setpoint_angle += increment_of_angle;
			
			feedback_angle = output_angle;
			
			
			output_angle_speed  = pid_process(&state_angle,&setpoint_angle, &feedback_angle, kp_angle, ki_angle,kd_angle);
			if (output_angle_speed > ANGLE_PID_LIMIT ){output_angle_speed = ANGLE_PID_LIMIT;}
			else if (output_angle_speed < -ANGLE_PID_LIMIT){output_angle_speed = -ANGLE_PID_LIMIT;}
			
//output as the input of the first pid
			//simply add together is not a efficient way to control
				//haha, this is the coefficient of turning
			int32_t max_wheel_setpoint = abs(forward_speed)+abs(right_speed );	
			int32_t larger_abs_speed = abs(forward_speed);
			if(larger_abs_speed<abs(right_speed )) larger_abs_speed = abs(right_speed );
			
			
			wheel_setpoints[0] =(  forward_speed+right_speed )*larger_abs_speed / max_wheel_setpoint ;
			wheel_setpoints[1] =( -forward_speed+right_speed )*larger_abs_speed / max_wheel_setpoint ;
			wheel_setpoints[2] =( -forward_speed-right_speed )*larger_abs_speed / max_wheel_setpoint ;
			wheel_setpoints[3] =(  forward_speed-right_speed )*larger_abs_speed / max_wheel_setpoint ;
			//but the code above is only for the moving back and forth, left and right
			
			
			for (int i = 0; i <4 ; i ++){
				buffer_in(buffer[i], BUFFER_LENGTH, ticks_msimg , wheel_setpoints[i]);
				wheel_setpoints[i] = buffer_out(buffer[i], BUFFER_LENGTH, ticks_msimg);
				wheel_setpoints[i] += output_angle_speed;
			}	
			
			//This is where the power control happen
			for(int i = 0 ; i < 4 ; i++){
					wheel_setpoints[i] = wheel_setpoints[i]*wheel_setpoint_coefficient/1000;
			}
			
      wheel_setpoints_adjust(&wheel_setpoints[0], &wheel_setpoints[1],&wheel_setpoints[2],&wheel_setpoints[3] ,FILTER_RATE_LIMIT );
			
//these are the feed back as the current state 
			wheel_feedbacks[0] = CM1Encoder.filter_rate;
			wheel_feedbacks[1] = CM2Encoder.filter_rate;
			wheel_feedbacks[2] = CM3Encoder.filter_rate;
 			wheel_feedbacks[3] = CM4Encoder.filter_rate;
			
		
//pid process to get the output as the torque
			
			
			for (int i = 0 ; i < 4 ; i++){
					  wheel_outputs[i] = pid_process(&states[i],
						&wheel_setpoints[i],
						&wheel_feedbacks[i],
						kp,ki,kd);
			}
			
			
			
			
//change the actuator value
				
				
			//first argument: left-front, forward
			//second argument: right-front, backward
			//third arguemnt: right-back, backward
			//fourth argument: left-back, forward

			 
				Set_CM_Speed(CAN2, 
					wheel_outputs[0],
					wheel_outputs[1],
					wheel_outputs[2],
					wheel_outputs[3]);
				
		}	
	
			
		  
		  else if (DBUS_ReceiveData.rc.switch_right == 2){
				Set_Gimbal_Current( CAN2, current_yaw ,0 ,0);
			
			}
				
			
			
			
			if(ticks_msimg%50==0)
			{
				tft_clear();
			/*
				tft_prints(1,2,"Pr: %.1fl", Pr);
				tft_prints(1,3,"PL: %.1fl", PL);
				tft_prints(1,4, "W: %d", W_int);
				tft_prints(1,5, "Co: %d",wheel_setpoint_coefficient );
			*/
				tft_prints (1,2, "yI: %d", current_yaw);
				
				for(int i = 0; i <4; i++){
					//tft_prints(1, i+2, "sp%d: %d",i+1,wheel_setpoints[i] );
					//tft_prints(1,i+7,"fr%d: %d",i+1,wheel_feedbacks[i]);
				}
				tft_update();
				LED_blink(LED1);
			}		




		
			
		}//main loop with ticks	
	}
	
	
}	//main

	



