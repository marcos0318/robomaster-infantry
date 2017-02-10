/*
This file is used to store the 
*/
#ifndef HELPERS
#define HELPERS
#include "stdint.h"

//First, the buffers

int32_t buffer_out(int32_t* b, int32_t length, int32_t counter);
void buffer_in(int32_t* b , int32_t length, int32_t counter, int32_t input);

//Second, the adjustment of the largest set point of filter rate

int32_t abs(int32_t x);

void wheel_setpoints_adjust(int32_t * sp1, int32_t* sp2, int32_t* sp3, int32_t* sp4, int32_t limit);


#endif