#include "pid.h"
#include "ti_msp_dl_config.h"

//设置PID初始化
void PID_Init(PID_Controller*pid, float Kp, float Ki, float Kd, float max_output, float min_output)
{
	pid->Point = 0.0f;              
	pid->Input = 0.0f;
	pid->Output = 0.0f;
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Last_Error = 0.0f;
    pid->Integral = 0.0f;
    pid->Max_Integral = 100.0f;  
    pid->Min_Integral = -100.0f;
    pid->Max_Output = max_output;  
    pid->Min_Output = min_output;
}

//通过目标输入PID计算出输出值
float PID_Compute(PID_Controller*pid, float input)
{
	float error = pid->Point - input;                    //设计误差值
	pid->Integral += error;                              //误差值累积
	float derivative = error - pid->Last_Error;           //偏差值
	pid->Output = pid->Kp * error + pid->Ki * pid->Integral + pid->Kd * derivative;
	pid->Last_Error = error;
	return pid->Output;
}

//设置当前值，进去计算
void PID_SetInput(PID_Controller*pid, float input)
{
	pid->Input = input;
}

//设置目标
void PID_SetPoint(PID_Controller*pid, float point)
{
	pid->Point = point;
}
