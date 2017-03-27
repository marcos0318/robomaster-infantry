#include "main.h"
#include "function_list.h"

volatile u32 ticks_msimg = (u32)-1;
#define BUFFER_LENGTH 400
#define POWER_BUFFER_LENGTH 20
#define ANGLE_PID_LIMIT 300
#define MOVING_BOUND_1 200
#define MOVING_BOUND_2 450
#define LiftingMotorSetpointLimit 32768
int16_t LiftingMotorSetpoint[4]={0};
enum State{StaticState,MovingState};
bool SetpointStatic=false;
enum State GimbalState; 	
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
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
	ENCODER_Init();
	GUN_Init();
}


int16_t checkSetpoint(int16_t a, bool dir){
	if(dir){
		if(a>LiftingMotorSetpointLimit-10)
			a=LiftingMotorSetpointLimit-1;
		else{a+=10;}
	}		else{
		if(a<(-LiftingMotorSetpointLimit+10)){
			a=-LiftingMotorSetpointLimit;
		} else {a-=10;}
	}
	return a;
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
int32_t FILTER_RATE_LIMIT = 700;
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

//********************************************************************************************************************
//Gimbal yaw control (Position loop and velocity loop) 
//setpoint control
float gimbalPositionSetpoint = 0;// prevGimbalPositionSetpoint = 0;
float bufferedGimbalPositionSetpoint = 0;
float gimbalPositionFeedback= 0;
bool isGimbalPositionSetpointIncrease = true;

struct fpid_control_states gimbalPositionState = {0,0,0};
float gpos_kp = 0.5;
float gpos_ki = 0.00032;
float gpos_kd = 22;

int32_t yawPosMultiplier = 3;		//DBUS mouse yaw control

//speed control
struct inc_pid_states gimbalSpeedMoveState;// gimbalSpeedStaticState;
int32_t gimbalSpeedSetpoint = 0;
int32_t gimbalSpeedMoveOutput = 0;
int32_t outsideLimit=1500;
//********************************************************************************************************************
//Pitch control
//setpoint control
float pitchPositionSetpoint = 0;// prevGimbalPositionSetpoint = 0;
float bufferedPitchPositionSetpoint = 0;
float pitchPositionFeedback= 0;
bool isPitchPositionSetpointIncrease = true;
int32_t storedPitch = 0;

struct fpid_control_states pitchPositionState = {0,0,0};
float ppos_kp = 0.4;
float ppos_ki = 0.0003;
float ppos_kd = 12;

//speed control
struct inc_pid_states pitchSpeedMoveState;// gimbalSpeedStaticState;
int32_t pitchSpeedSetpoint = 0;
int32_t pitchSpeedMoveOutput = 0;

int32_t pitchPosMultiplier = 3;       //DBUS mouse pitch control

//********************************************************************************************************************

//DBUS control data
int32_t speed_limitor  = 660;
int32_t speed_multiplier;
int32_t angular_speed_limitor = 200;
int32_t forward_speed ;
int32_t right_speed;
int32_t increment_of_angle;
int32_t mouse_prev=0;
float gimbalNotOutGyroOutput=0;
bool locked=false;   //for key F
bool Fprev=false;
//********************************************************************************************************************




//The coorperation of gimbal and the chasis
//The direction is from 0 to 8192
//The gyro of chasis is ranged from 0 to 3600, so we need converstion
int32_t direction = 0;
int32_t upperTotal = 4500;

//The rune mode
int32_t isRuneMode = 0;
int32_t lastIsRuneMode = 0;

int32_t currentLeft = 0;
int32_t lastLeft = 0;





//debug session
int32_t error = 0 ;
int32_t spSp = 0; 
int32_t Cerror = 0;




int main(void)
{	
	init();
	DBUS_ReceiveData.mouse.ytotal=0;
	
	//init the buffers with zero 
	for (int i =0 ; i<4 ; i++){
		for (int j = 0; j<BUFFER_LENGTH; j++){
			buffer[i][j]=0;
		}
	}
	
	incPIDinit(&gimbalSpeedMoveState);
	incPIDinit(&pitchSpeedMoveState);

	incPIDset(&gimbalSpeedMoveState, 70,3.7,0);
	incPIDset(&pitchSpeedMoveState, 70,3.7,0);

	mouse_prev=DBUS_ReceiveData.mouse.xtotal;
	while (1)  
	{	


		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	

			if (DBUS_ReceiveData.rc.switch_left == 1||DBUS_ReceiveData.rc.switch_left == 3) { 

					
//********************************************************************************************************************
//DBUS data analyze begins
				//Analyse the data received from DBUS and transfer moving command					
				
				speed_limitor  = 660;
				speed_multiplier = FILTER_RATE_LIMIT;
				angular_speed_limitor = 200;
				forward_speed = (DBUS_ReceiveData.rc.ch1 + DBUS_CheckPush(KEY_W)*660 - DBUS_CheckPush(KEY_S)*660) * speed_multiplier/speed_limitor ;
				right_speed =   (DBUS_ReceiveData.rc.ch0 + DBUS_CheckPush(KEY_D)*660 - DBUS_CheckPush(KEY_A)*660) * speed_multiplier/speed_limitor ;


				//direction = -DBUS_ReceiveData.rc.ch2*2 + -DBUS_ReceiveData.mouse.xtotal*4 ;
				if(DBUS_ReceiveData.rc.switch_left == 1) {		//auto follow mode
					direction += (-DBUS_ReceiveData.rc.ch2/300 + -DBUS_ReceiveData.mouse.x);
					setpoint_angle = -direction * 3600/upperTotal;
					gimbalPositionSetpoint = direction + output_angle*upperTotal/3600;
					if (gimbalPositionSetpoint > 1500) gimbalPositionSetpoint = 1500;
					if (gimbalPositionSetpoint < -1500) gimbalPositionSetpoint = -1500;
				}
				if(DBUS_ReceiveData.rc.switch_left == 3)		//keyboard-mouse mode, chasis will turn if mouse go beyong the boundary
				{

					//direction not move when the difference is large
					if (abs(direction + output_angle*upperTotal/3600) <= outsideLimit) 
						direction += (-DBUS_ReceiveData.rc.ch2/300 + -DBUS_ReceiveData.mouse.x);
					else if ((direction + output_angle*upperTotal/3600)>outsideLimit)
						direction = outsideLimit  -output_angle*upperTotal/3600;			
					else if ((direction + output_angle*upperTotal/3600)<-outsideLimit)
						direction = -outsideLimit -output_angle*upperTotal/3600;

					gimbalPositionSetpoint = direction +  output_angle*upperTotal/3600;

					if(DBUS_ReceiveData.mouse.press_right) {
						setpoint_angle = -direction * 3600/upperTotal;
					}

					/*
					if(DBUS_ReceiveData.mouse.xtotal*yawPosMultiplier>=outsideLimit){
						gimbalPositionSetpoint=-outsideLimit;
						
						setpoint_angle+=(DBUS_ReceiveData.mouse.x)/14;
						DBUS_ReceiveData.mouse.xtotal=outsideLimit/yawPosMultiplier;
					}else if(DBUS_ReceiveData.mouse.xtotal*yawPosMultiplier<=-outsideLimit){
						gimbalPositionSetpoint=outsideLimit;
						
						setpoint_angle+=(DBUS_ReceiveData.mouse.x)/14;
						DBUS_ReceiveData.mouse.xtotal=-outsideLimit/yawPosMultiplier	;
					}
					*/
					//Used for protection				
					if (gimbalPositionSetpoint > 2000) gimbalPositionSetpoint = 2000;
					if (gimbalPositionSetpoint < -2000) gimbalPositionSetpoint = -2000;
					// else gimbalPositionSetpoint=-DBUS_ReceiveData.mouse.xtotal*yawPosMultiplier;
					
					/*
					if(DBUS_ReceiveData.mouse.press_right && Fprev){
						setpoint_angle+=(DBUS_ReceiveData.mouse.x)/5;
						gimbalPositionSetpoint=0;
					}
					
					if( DBUS_ReceiveData.mouse.press_right && !Fprev)		//calibrating
					{
						//start to press F
						gimbalPositionSetpoint=0;
						setpoint_angle=output_angle-GMYawEncoder.ecd_angle/2.7;
						DBUS_ReceiveData.mouse.xtotal=0;
						
					}
					Fprev=DBUS_ReceiveData.mouse.press_right;
					//mouse_prev=DBUS_ReceiveData.mouse.xtotal;
					*/
				}
//********************************************************************************************************************


//chasis turing speed limit control begins				
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
//chasis turning speed limit control ends
//********************************************************************************************************************
//auto follow mode gimbal control begins
				if(DBUS_ReceiveData.rc.switch_left == 1){

					//The separate gimbal control 
					//Gimbal input
				}
//auto follow mode gimbal control ends
//********************************************************************************************************************

				if(DBUS_ReceiveData.rc.switch_left == 3){
					
				//position setpoint is done above
									
					if(bufferedGimbalPositionSetpoint < gimbalPositionSetpoint) isGimbalPositionSetpointIncrease = true;
					else isGimbalPositionSetpointIncrease = false;
					if(isGimbalPositionSetpointIncrease){
						bufferedGimbalPositionSetpoint+=5;
						if (bufferedGimbalPositionSetpoint > gimbalPositionSetpoint)
						bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
					}
					else {
						bufferedGimbalPositionSetpoint-=5;
						if(bufferedGimbalPositionSetpoint < gimbalPositionSetpoint) 
							bufferedGimbalPositionSetpoint = gimbalPositionSetpoint;
					}
					
									
				}
//********************************************************************************************************************
//pitch setpoint control
				//limit pitch position
				
					if(DBUS_ReceiveData.mouse.ytotal*pitchPosMultiplier>-120) DBUS_ReceiveData.mouse.ytotal=-120/pitchPosMultiplier;
					else if(DBUS_ReceiveData.mouse.ytotal*pitchPosMultiplier<-950) DBUS_ReceiveData.mouse.ytotal=-950/pitchPosMultiplier;
					
					//pitch setpoint
					pitchPositionSetpoint=-DBUS_ReceiveData.mouse.ytotal*pitchPosMultiplier;
					
					if(bufferedPitchPositionSetpoint < pitchPositionSetpoint) isPitchPositionSetpointIncrease = true;
					else isPitchPositionSetpointIncrease = false;
					if(isPitchPositionSetpointIncrease) {
						bufferedPitchPositionSetpoint+=70;
						if (bufferedPitchPositionSetpoint > pitchPositionSetpoint)
						bufferedPitchPositionSetpoint = pitchPositionSetpoint;
					}
					else {
						bufferedPitchPositionSetpoint-=70;
						if(bufferedPitchPositionSetpoint < pitchPositionSetpoint) 
							bufferedPitchPositionSetpoint = pitchPositionSetpoint;
					}
				
//********************************************************************************************************************				
//Yaw and Pitch speed control 				
				
				gimbalPositionFeedback = GMYawEncoder.ecd_angle;
				gimbalSpeedSetpoint = (int32_t)fpid_process(&gimbalPositionState, &gimbalPositionSetpoint, &gimbalPositionFeedback,gpos_kp,gpos_ki,gpos_kd );

				pitchPositionFeedback = GMPitchEncoder.ecd_angle;
				pitchSpeedSetpoint = (int32_t)fpid_process(&pitchPositionState, &pitchPositionSetpoint, &pitchPositionFeedback,ppos_kp,ppos_ki,ppos_kd );


				//Limit the output
				if (gimbalSpeedSetpoint > 200) gimbalSpeedSetpoint = 200;
				else if (gimbalSpeedSetpoint < -200) gimbalSpeedSetpoint = -200;
				
				if (pitchSpeedSetpoint > 80) pitchSpeedSetpoint = 80;
				else if (pitchSpeedSetpoint < -80) pitchSpeedSetpoint = -80;
					
				//mock speed here
				//gimbalSpeedSetpoint = DBUS_ReceiveData.rc.ch2 * 0.5;
				//Get the speed here

				incPIDsetpoint(&gimbalSpeedMoveState, gimbalSpeedSetpoint);
				gimbalSpeedMoveOutput+=incPIDcalc(&gimbalSpeedMoveState, GMYawEncoder.filter_rate);
				
				incPIDsetpoint(&pitchSpeedMoveState, pitchSpeedSetpoint);
				pitchSpeedMoveOutput+=incPIDcalc(&pitchSpeedMoveState, GMPitchEncoder.filter_rate);

//********************************************************************************************************************	
				//Print gimbal Yaw and Pitch
			  if(ticks_msimg%20==0){	
					for (int j=2;j<12;j++) tft_clear_line(j);
					tft_prints(1,2,"pos_set= %.1f",gimbalPositionSetpoint);
					tft_prints(1,3,"pos_set= %.1f",pitchPositionSetpoint);
					//tft_prints(1,4,"D= %d", direction);
					tft_prints(1,5,"error= %d", error);
					tft_prints(1,4,"tarbu:%d", Cerror);
					//tft_prints(1,5,"error= %d", states[0].cummulated_error);
					tft_prints(1,6,"speed=%d",GMballfeedEncoder.filter_rate);
					tft_prints(1,7,"ecd:%d", spSp);
					tft_prints(1,8,"Gun=%d",gunSpeed);
					//tft_prints(1,7,"%d",CM1Encoder.filter_rate);
					//tft_prints(1,8,"mouse.xt=%d",DBUS_ReceiveData.mouse.xtotal);
					//tft_prints(1,9,"mouse.yt=%d",DBUS_ReceiveData.mouse.ytotal);
					//tft_prints(1,10,"mouse.z=%d",DBUS_ReceiveData.mouse.z);
					tft_prints(1,11,"l=%d,r=%d",DBUS_ReceiveData.mouse.press_right,DBUS_ReceiveData.mouse.press_right);
					tft_update();		
				}

				//call the acturater function
				//if (ticks_msimg % 20 == 0)
				GUN_PokeControl();

		  	Set_CM_Speed(CAN1, gimbalSpeedMoveOutput,pitchSpeedMoveOutput,gunSpeed,0);						 
				Set_CM_Speed(CAN2, wheel_outputs[0], wheel_outputs[1], wheel_outputs[2], wheel_outputs[3]);
			}	
			
			/*
			else if (DBUS_ReceiveData.rc.switch_left == 3) { //Also the stop mode now
				Set_CM_Speed(CAN1,0,0,0,0);
				Set_CM_Speed(CAN2,0,0,0,0);
					
		
			} 
			*/
			else if (DBUS_ReceiveData.rc.switch_left ==2) { //The stop mode
				if(ticks_msimg%20==0){
					if(DBUS_CheckPush(KEY_Z)){
						LiftingMotorSetpoint[0]=LiftingMotorSetpoint[1]=checkSetpoint(LiftingMotorSetpoint[0],true);
					}
					if(DBUS_CheckPush(KEY_X)){
						LiftingMotorSetpoint[0]=LiftingMotorSetpoint[1]=checkSetpoint(LiftingMotorSetpoint[0],false);
					}
					if(DBUS_CheckPush(KEY_Z)){
						LiftingMotorSetpoint[2]=LiftingMotorSetpoint[3]=checkSetpoint(LiftingMotorSetpoint[2],true);
					}
					if(DBUS_CheckPush(KEY_Z)){
						LiftingMotorSetpoint[2]=LiftingMotorSetpoint[3]=checkSetpoint(LiftingMotorSetpoint[3],false);
					}
					for(uint8_t i=0;i<4;i++){
						tft_prints(1,i+2,"LMP %d %d",i,LiftingMotorSetpoint[i]);
					}
					tft_update();
					
				}
				for(uint8_t i;i<4;i++){
					if(ticks_msimg%4==i){
						DataMonitor_Send(i,LiftingMotorSetpoint[i]);
					}
				}




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

	