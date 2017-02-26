#include "main.h"
#include "function_list.h"


#define BUFFER_LENGTH 300
#define POWER_BUFFER_LENGTH 20
#define ANGLE_PID_LIMIT 2000
#define MOVING_BOUND_1 200
#define MOVING_BOUND_2 450
enum State{StaticState,MovingState};
bool SetpointStatic=false;
enum State GimbalState;
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


int32_t buffer[4][BUFFER_LENGTH];

//PID controls

//The control of filter rate of wheels 
// Structure to strore PID data 
struct pid_control_states states[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
int32_t wheel_setpoints[4] = {0,0,0,0};
int32_t wheel_feedbacks[4] = {0,0,0,0};
int32_t wheel_outputs[4] = {0,0,0,0};
int32_t kp = 80, ki = 4, kd = 1;

//The control of angle of chasis
struct pid_control_states state_angle = {0,0,0};
int32_t setpoint_angle 	= 0;
int32_t feedback_angle = 0;
int32_t output_angle_speed = 0;
int32_t kp_angle = 1;
int32_t ki_angle = 0;
int32_t kd_angle = 1;
	

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


//Gimbal yaw control (Position loop and velocity loop) 
float prevGMYawEncoderEcdAngle=0;
struct inc_pid_states gimbalPositionState, gimbalSpeedMoveState;// gimbalSpeedStaticState;
int32_t gimbalPositionSetpoint = 0;// prevGimbalPositionSetpoint = 0;
int32_t bufferedGimbalPositionSetpoint = 0;
bool isGimbalPositionSetpointIncrease = true;

int32_t gimbalSpeedSetpoint = 0;
int32_t gimbalSpeedMoveOutput = 0;// gimbalSpeedStaticOutput = 0;





int main(void)
{	
	init();
	
	//init the buffers with zero 
	for (int i =0 ; i<4 ; i++){
		for (int j = 0; j< BUFFER_LENGTH; j++){
			buffer[i][j]=0;
		}
	}
	
	incPIDinit(&gimbalPositionState);
	incPIDinit(&gimbalSpeedMoveState);
	//incPIDinit(&gimbalSpeedStaticState);

	incPIDset(&gimbalPositionState, 0.6,0.1,0);
	incPIDset(&gimbalSpeedMoveState, 60,3,40);
	//incPIDset(&gimbalSpeedStaticState, 80.0, 7, 100);
	//incPIDsetpoint(&gimbalSpeedStaticState, 0);
	while (1)  {	


		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			
			if (DBUS_ReceiveData.rc.switch_right == 1){
			
				if (ticks_msimg%20==0){
					//power control 
	      			//imitate the process in judging system
					feedback_current = InfantryJudge.RealCurrent; 
					feedback_voltage = InfantryJudge.RealVoltage;
					Pr = feedback_current*feedback_voltage;
					
					W = W - (Pr -PL) * 0.02;
					if (W <= 0){
						W = 0;
						//ÂèµÄÎÒÑªÁ÷ÂúµØÁË
					}				
					else if (W > 60) {
						  W = 60;
					}
					W_int = W;
					work_pid_output = pid_process_gai1(&work_state, &work_target, &W_int ,kp_power,ki_power,kd_power ); 
					//The coefficient that reflect how much more energy can be used
					wheel_setpoint_coefficient = 1000 - work_pid_output;
					if (wheel_setpoint_coefficient < 200) {wheel_setpoint_coefficient = 200;}
					if (wheel_setpoint_coefficient > 1000) {wheel_setpoint_coefficient = 1000;}
				}
					
					

				//Analyse the data received from DBUS and transfer moving command					
				
				int32_t speed_limitor  = 660;
				int32_t speed_multiplier = FILTER_RATE_LIMIT;
				int32_t angular_speed_limitor = 200;
				int32_t forward_speed = (DBUS_ReceiveData.rc.ch1 + DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660) * speed_multiplier/speed_limitor ;
				int32_t right_speed =   (DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_E)*660 - DBUS_CheckPush(KEY_Q)*660) * speed_multiplier/speed_limitor ;
				int32_t increment_of_angle = (DBUS_ReceiveData.rc.ch2 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660) /angular_speed_limitor;

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
					wheel_outputs[i] = pid_process(&states[i], &wheel_setpoints[i], &wheel_feedbacks[i], kp,ki,kd);
				}
						 
				Set_CM_Speed(CAN2, wheel_outputs[0], wheel_outputs[1], wheel_outputs[2], wheel_outputs[3]);
				Set_CM_Speed(CAN1, 0, 0, 0, 0);
			}	
			else if (DBUS_ReceiveData.rc.switch_right == 3) {
				gimbalPositionSetpoint = -DBUS_ReceiveData.mouse.xtotal;
						
				if(bufferedGimbalPositionSetpoint < gimbalPositionSetpoint) isGimbalPositionSetpointIncrease = true;
				else isGimbalPositionSetpointIncrease = false;
				if(isGimbalPositionSetpointIncrease){
					bufferedGimbalPositionSetpoint+=50;
					if (bufferedGimbalPositionSetpoint > gimbalPositionSetpoint)
					bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
				}
				else {
					bufferedGimbalPositionSetpoint-=50;
					if(bufferedGimbalPositionSetpoint < gimbalPositionSetpoint) 
						bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
				}

				incPIDsetpoint(&gimbalPositionState, bufferedGimbalPositionSetpoint);
				gimbalSpeedSetpoint+=incPIDcalc(&gimbalPositionState, (int32_t)(GMYawEncoder.ecd_angle));
				//Limit the output
				if (gimbalSpeedSetpoint > 1000) gimbalSpeedSetpoint = 1000;
				else if (gimbalSpeedSetpoint < -1000) gimbalSpeedSetpoint = -1000;
				//incPIDClearError(&gimbalSpeedStaticState);
				incPIDsetpoint(&gimbalSpeedMoveState, gimbalSpeedSetpoint);
				gimbalSpeedMoveOutput+=incPIDcalc(&gimbalSpeedMoveState, GMYawEncoder.filter_rate);
		  	Set_CM_Speed(CAN1, gimbalSpeedMoveOutput,0,0,0);
				
				
			  if(ticks_msimg%20==0){	
					for (int j=2;j<12;j++) tft_clear_line(j);
					tft_prints(1,2,"setpoint=%d",gimbalPositionSetpoint);
					tft_prints(1,3,"filter rate=%d",GMYawEncoder.filter_rate);
					tft_prints(1,4,"temp sp=%d",bufferedGimbalPositionSetpoint);

					tft_prints(1,6,"angle=%f",GMYawEncoder.ecd_angle);
					if(GimbalState==MovingState) tft_prints(1,7,"speed_tar=%d",gimbalSpeedSetpoint);
					tft_prints(1,8,"mouse.xt=%d",DBUS_ReceiveData.mouse.xtotal);
					tft_prints(1,9,"mouse.yt=%d",DBUS_ReceiveData.mouse.ytotal);
					//tft_prints(1,10,"mouse.z=%d",DBUS_ReceiveData.mouse.z);
					if(GimbalState==StaticState) tft_prints(1,10,"inside"); else tft_prints(1,10,"outside");
					tft_prints(1,11,"l=%d,r=%d",DBUS_ReceiveData.mouse.press_left,DBUS_ReceiveData.mouse.press_right);
					tft_update();		
				}
					
		
				prevGMYawEncoderEcdAngle=GMYawEncoder.ecd_angle;
			} 
			else {
				Set_CM_Speed(CAN1, 0, 0, 0, 0);
				Set_CM_Speed(CAN2, 0, 0, 0, 0);
			}		
			
				
			
			/*
			//all the tft_prints things				
			if(ticks_msimg%50==0)
			{
				tft_clear();
				if (DBUS_ReceiveData.rc.switch_right == 1) {
					tft_prints (1,2, "W: %.2f", W);
			    tft_prints (1,3, "Pr: %.2f", Pr);
					tft_prints (1,4, "Cof: %d", wheel_setpoint_coefficient);
				}
				if (DBUS_ReceiveData.rc.switch_right == 3) {
					tft_prints (1,2, "Angle: %.3f", CM1Encoder.ecd_angle);
					tft_prints (1,3, "");
				}
				
				tft_update();
				LED_blink(LED1);
			}		
			*/			
			
		}//main loop with ticks	
	}
	
	
}	//main

	



