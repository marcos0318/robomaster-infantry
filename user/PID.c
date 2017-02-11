#include "PID.h"
int16_t pid_process( struct pid_control_states* states, int32_t* setpoint, int32_t* feedback, int32_t kp, int32_t ki, int32_t kd){

		//first update current state 
	states->last_error	= states->current_error;
	states->current_error = *setpoint - *feedback;
	states->cummulated_error += states->current_error;
	
	
		//then return the output value
	int32_t output = kp* 	states->current_error 
									+ki* states->cummulated_error 
									+kd* (states->current_error-states->last_error);
	
		
	return output;
	
}	

int16_t pid_process_gai1( struct pid_control_states* states, int32_t* setpoint, int32_t* feedback, int32_t kp, int32_t ki, int32_t kd ) {
	states->last_error	= states->current_error;
	states->current_error = *setpoint - *feedback;
	states->cummulated_error *= 0.90;
	states->cummulated_error += states->current_error;
	
	int32_t output = kp* 	states->current_error 
									+ki* states->cummulated_error 
									+kd* (states->current_error-states->last_error);

	return output;
}