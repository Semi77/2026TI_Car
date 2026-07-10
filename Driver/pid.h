#ifndef __PID_H
#define __PID_H


#include <math.h>


typedef struct
{
	float Point;       //设定目标值
	float Input;       //当前反馈值
	float Output;      //输出值
    float Kp;          //比例系数
	float Ki;          //积分系数
	float Kd;          //微分系数
	float Last_Error;   //上一次误差值
	float Integral;    //误差积分值
	float Max_Integral; // 积分限幅
    float Min_Integral; // 积分限幅
    float Max_Output;   // 输出限幅
    float Min_Output;   // 输出限幅
}PID_Controller;

extern PID_Controller pid;



void PID_Init(PID_Controller*pid, float Kp, float Ki, float Kd, float max_output, float min_output);
float PID_Compute(PID_Controller*pid, float input);
void PID_SetInput(PID_Controller*pid, float input);
void PID_SetPoint(PID_Controller*pid, float point);

#endif