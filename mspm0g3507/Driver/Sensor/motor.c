#include "motor.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

volatile int g_stop_flag=0;

int32_t MAX_SPEED=800;
int32_t MIN_SPEED=-800;

void Motor_Control(int16_t speedA,int16_t speedB)
{
    if(speedA>MAX_SPEED)speedA=MAX_SPEED;
    if(speedA<MIN_SPEED)speedA=MIN_SPEED;

    if(speedB>MAX_SPEED)speedB=MAX_SPEED;
    if(speedB<MIN_SPEED)speedB=MIN_SPEED;


    if (speedA > 0)	//正转
	{
        DL_GPIO_setPins(MOTOR_AIN_PORT, MOTOR_AIN_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN_PORT, MOTOR_AIN_MOTOR_AIN2_PIN);
	}
	else if (speedA < 0)		//反转
	{
		speedA = -speedA;
        DL_GPIO_clearPins(MOTOR_AIN_PORT, MOTOR_AIN_MOTOR_AIN1_PIN);
		DL_GPIO_setPins(MOTOR_AIN_PORT, MOTOR_AIN_MOTOR_AIN2_PIN);
	}
    else 
    {
        DL_GPIO_clearPins(MOTOR_AIN_PORT, MOTOR_AIN_MOTOR_AIN1_PIN);
		DL_GPIO_clearPins(MOTOR_AIN_PORT, MOTOR_AIN_MOTOR_AIN2_PIN);
    }    
	
    //右轮
	if (speedB > 0)
	{
		DL_GPIO_setPins(MOTOR_BIN_PORT, MOTOR_BIN_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_BIN_PORT, MOTOR_BIN_MOTOR_BIN2_PIN);
	}
	else if (speedB < 0)
	{
		speedB = -speedB;
        DL_GPIO_clearPins(MOTOR_BIN_PORT, MOTOR_BIN_MOTOR_BIN1_PIN);
		DL_GPIO_setPins(MOTOR_BIN_PORT, MOTOR_BIN_MOTOR_BIN2_PIN);
	}
	else 
    {
        DL_GPIO_clearPins(MOTOR_BIN_PORT, MOTOR_BIN_MOTOR_BIN1_PIN);
		DL_GPIO_clearPins(MOTOR_BIN_PORT, MOTOR_BIN_MOTOR_BIN2_PIN);
    }
    
	//改变占空比输出 [0, 1000]
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, speedA, DL_TIMER_CC_0_INDEX);
	DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, speedB, DL_TIMER_CC_1_INDEX);
}

/* 该函数控制小车向前行驶，参数speed表示左右电机的目标PWM值。 */
void Motor_Forward(int16_t speed)
{
    Motor_Control(speed, -speed);
}

void Motor_Stop(void)
{
	DL_GPIO_clearPins(MOTOR_AIN_PORT, MOTOR_AIN_MOTOR_AIN1_PIN);
	DL_GPIO_clearPins(MOTOR_AIN_PORT, MOTOR_AIN_MOTOR_AIN2_PIN);
	
	DL_GPIO_clearPins(MOTOR_BIN_PORT, MOTOR_BIN_MOTOR_BIN1_PIN);
	DL_GPIO_clearPins(MOTOR_BIN_PORT, MOTOR_BIN_MOTOR_BIN2_PIN);
}
