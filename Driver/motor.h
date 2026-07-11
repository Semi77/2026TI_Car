#ifndef __MOTOR_H
#define __MOTOR_H
#include <stdint.h>

extern volatile int g_stop_flag;

void Motor_Control(int16_t speedA,int16_t speedB);
void Motor_Forward(int16_t speed);
void Motor_Stop(void);
#endif
