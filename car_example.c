#include "ti_msp_dl_config.h"
#include "test.h"

#ifndef TEST_MODE
#define TEST_MODE TEST_MODE_MOTOR
#endif

#if TEST_MODE == TEST_MODE_OLED
int main(void)
{
    SYSCFG_DL_init();
    Test_OLED_Run();

    while (1) {
    }
}
#elif TEST_MODE == TEST_MODE_MOTOR
int main(void)
{
    SYSCFG_DL_init();
    Test_Motor_Run();

    while (1) {
    }
}
#else
#include "ti_msp_dl_config.h"
#include "motor.h"
#include "oled_hardware_i2c.h"
#include "mpu6050.h"
#include "gray.h"
#include "pid.h"
#include "uart.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <ti/driverlib/dl_timera.h> 

//定义宏
#define SPEED_STRAIGHT  -100
#define SPEED_CURVE     -70
#define SPEED_SPIN      -70
#define LINE_FOUND_COUNT 3

//定义状态机
typedef enum {
    MODE_TASK_1 = 1,
    MODE_TASK_2 = 2,
    MODE_TASK_3 = 3,
    MODE_TASK_4 = 4
} Task_Mode_t;

typedef enum
{
    MODE_A_TO_B = 0,
    MODE_B_TO_C = 1,
    MODE_C_TO_D = 2,
    MODE_D_TO_A = 3,
    MODE_STOP,
}Car_State_t;

volatile Task_Mode_t g_Current_Mode = MODE_TASK_2; //设置任务1状态
volatile Car_State_t g_Current_State = MODE_A_TO_B;
volatile uint32_t g_Run_Time_Count = 0;            //运动时间
volatile uint8_t g_Line_Check_Count = 0;           //巡线检测数
volatile float g_Gray_Error = 0;                   //巡线返回误差
volatile uint8_t g_Is_Running = 1;                 //运行使能
volatile float current_yaw = 0.0f;

//PID的结构体
PID_Controller gray_pid;   //速度闭环
PID_Controller gyro_pid;    //角度闭环

// void CONTROL_TIMER_INST_IRQHandler(void)
// {
//     //运行更新
//     switch (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST)) 
//     {
//         case DL_TIMER_IIDX_ZERO:
//             //更新MPU6050
//             MPU6050_Update_Yaw(5.0f); 
//             error = Gray_Test();

//         switch (g_Current_Mode) 
//         {
//             case MODE_TASK_1:
//             {
//                 float current_yaw=MPU_Get_Yaw();
//             }
            
//         }    
            
//     }
// }

uint32_t g_irq_count = 0;
// Timer中断可能比主循环处理更快：用计数器避免“丢掉若干次更新”
volatile uint8_t g_mpu_flag = 0;

int main(void)
{
    SYSCFG_DL_init();
    
    MPU6050_Init();
    delay_ms(2000);
    MPU6050_Calibration();
    OLED_Init();
    PID_Init(&gyro_pid, 1.5f, 0, 0.8f,100.0f, -100.0f);
    PID_Init(&gray_pid, 30.0f, 0, 0.8f, 50, -50);
    //定时器和中断使能
    DL_TimerA_startCounter(PWM_MOTOR_INST);
    DL_TimerA_startCounter(CONTROL_TIMER_INST);
    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    OLED_ShowNum(50, 6, 666, 3, 8);
    Motor_Control(70, 70);
    while (1)
    {
        current_yaw = MPU_Get_Yaw();            
        // 在 OLED 上显示 yaw
        OLED_ShowString(70, 1, (uint8_t *)"Y:", 16);
        OLED_ShowNum(80, 1, (uint32_t)(current_yaw + 360.0f) % 360, 3, 16);
        UART_Printf("yaw:%f\r\n",current_yaw);
        Motor_Control(70, 70);
           
    }
}
    
// void CONTROL_TIMER_INST_IRQHandler()
// {
//     if(DL_TimerA_getPendingInterrupt(CONTROL_TIMER_INST) & DL_TIMER_IIDX_ZERO)
//     {
//         DL_TimerA_clearInterruptStatus(CONTROL_TIMER_INST, DL_TIMER_IIDX_ZERO);
//         g_mpu_flag=1;
//     }
// }


// void CONTROL_TIMER_INST_IRQHandler(void)
// {
//     if (DL_TimerA_getPendingInterrupt(CONTROL_TIMER_INST) == DL_TIMER_IIDX_ZERO)
//     {
//         DL_TimerA_clearInterruptStatus(CONTROL_TIMER_INST, DL_TIMER_IIDX_ZERO);
        
//         //步骤1：传感器更新 (5ms周期)
//         MPU6050_Update_Yaw(10.0f);    // MPU6050姿态更新（dt=10ms）
//         g_Gray_Error = Gray_Test(); // 灰度传感器误差更新

//         // 步骤2：停车保护
//         if (!g_Is_Running)
//         {
//             Motor_Stop();
//             g_Run_Time_Count = 0;
//             return;
//         }

//         // 步骤3：运行计时&核心逻辑
//         g_Run_Time_Count++;
//         int16_t left_speed = 0, right_speed = 0;
//         current_yaw = MPU_Get_Yaw(); // 当前偏航角（Z轴）
//         float pid_gyro = 0.0f;
//         float pid_gray = 0.0f;
//         switch (g_Current_Mode)
//         {
//             // ===================== 任务1：直线盲跑+姿态矫正 =====================
//             case MODE_TASK_1:
//                 {
//                     // 姿态PID矫正：保持yaw=0（直线）
//                     PID_SetPoint(&gyro_pid, 0.0f);
//                     pid_gyro=PID_Compute(&gyro_pid, current_yaw);


//                     // 速度分配：基础速度+姿态矫正
                    // left_speed = SPEED_STRAIGHT + pid_gyro;
                    // right_speed = SPEED_STRAIGHT - pid_gyro;
                    // if(left_speed>150)left_speed=150;
                    // if(left_speed<-150)left_speed=-150;
                    // if(right_speed>150)right_speed=150;
                    // if(right_speed<-150)right_speed=-150;
                    
//                     // 检测到线且稳定（防误判）：停车
//                     // if (Check_Line_Found(g_Gray_Error) && g_Run_Time_Count > 200)
//                     // {
//                     //     g_Line_Check_Count++;
//                     //     if (g_Line_Check_Count >= LINE_FOUND_COUNT)
//                     //     {
//                     //         Motor_Stop();
//                     //         g_Is_Running = 0;
//                     //         g_Line_Check_Count = 0;
//                     //         g_Run_Time_Count = 0;
//                     //     }
//                     // }
//                     // else
//                     // {
//                     //     g_Line_Check_Count = 0;
//                     // }
                    
//                     Motor_Control(left_speed, right_speed);
//                     break;
//                 }
                
//             case MODE_TASK_2:
//                 {
//                     switch (g_Current_State) 
//                     {
//                         case MODE_A_TO_B:{
//                             PID_SetPoint(&gyro_pid, 0.0f);
//                             pid_gyro=PID_Compute(&gyro_pid, current_yaw);
//                             left_speed = SPEED_STRAIGHT + pid_gyro;
//                             right_speed = SPEED_STRAIGHT - pid_gyro;

//                             if(left_speed>150)left_speed=150;
//                             if(left_speed<-150)left_speed=-150;
//                             if(right_speed>150)right_speed=150;
//                             if(right_speed<-150)right_speed=-150;
//                             if (g_Run_Time_Count > 300)                          
//                             {    
//                                 if(Check_Line_Found(g_Gray_Error))
//                                 {
//                                     g_Line_Check_Count++;
//                                     if (g_Line_Check_Count >= 4)
//                                     {
//                                         g_Current_State = MODE_B_TO_C;
//                                         g_Line_Check_Count = 0;
//                                         g_Run_Time_Count = 0;
//                                     }
//                                 }
//                             }
//                             Motor_Control(left_speed, right_speed);
//                             break;
//                         }
//                         case MODE_B_TO_C:{
                            // PID_SetPoint(&gray_pid, 0.0f);
                            // pid_gray=PID_Compute(&gray_pid, g_Gray_Error);
                            // left_speed = SPEED_CURVE +pid_gray;
                            // right_speed = SPEED_CURVE -pid_gray;
                            // if(left_speed>100)left_speed=100;
                            // if(left_speed<-100)left_speed=-100;
                            // if(right_speed>100)right_speed=100;
                            // if(right_speed<-100)right_speed=-100;
                            
                            
//                             if (g_Run_Time_Count > 300)                          
//                             {    
//                                 if(Check_Line_Found(g_Gray_Error==0))
//                                 {
//                                     g_Line_Check_Count++;
//                                     if (g_Line_Check_Count >= 4)
//                                     {
//                                         g_Current_State = MODE_C_TO_D;
//                                         g_Line_Check_Count = 0;
//                                         g_Run_Time_Count = 0;
//                                     }
//                                 }
//                             }
                            
//                             Motor_Control(left_speed, right_speed);
//                             break;
//                         }
//                         case MODE_C_TO_D:{
//                             MPU_Reset_Yaw();
//                             PID_SetPoint(&gyro_pid, 0.0f);
//                             pid_gyro=PID_Compute(&gyro_pid, current_yaw);
//                             left_speed = SPEED_STRAIGHT + pid_gyro;
//                             right_speed = SPEED_STRAIGHT - pid_gyro;

//                             if(left_speed>150)left_speed=150;
//                             if(left_speed<-150)left_speed=-150;
//                             if(right_speed>150)right_speed=150;
//                             if(right_speed<-150)right_speed=-150;
//                             if (g_Run_Time_Count > 300)                          
//                             {    
//                                 if(Check_Line_Found(g_Gray_Error))
//                                 {
//                                     g_Line_Check_Count++;
//                                     if (g_Line_Check_Count >= 4)
//                                     {
//                                         g_Current_State = MODE_D_TO_A;
//                                         g_Line_Check_Count = 0;
//                                         g_Run_Time_Count = 0;
//                                     }
//                                 }
//                             }
//                             Motor_Control(left_speed, right_speed);
//                             break;
//                         }
//                         case MODE_D_TO_A:{
//                             PID_SetPoint(&gray_pid, 0.0f);
//                             pid_gray=PID_Compute(&gray_pid, g_Gray_Error);
//                             left_speed = SPEED_CURVE +pid_gray;
//                             right_speed = SPEED_CURVE -pid_gray;
//                             if(left_speed>100)left_speed=100;
//                             if(left_speed<-100)left_speed=-100;
//                             if(right_speed>100)right_speed=100;
//                             if(right_speed<-100)right_speed=-100;
                            
                            
//                             if (g_Run_Time_Count > 300)                          
//                             {    
//                                 if(Check_Line_Found(g_Gray_Error==0))
//                                 {
//                                     g_Line_Check_Count++;
//                                     if (g_Line_Check_Count >= 4)
//                                     {
//                                         g_Current_State = MODE_STOP;
//                                         g_Line_Check_Count = 0;
//                                         g_Run_Time_Count = 0;
//                                     }
//                                 }
//                             }
                            
//                             Motor_Control(left_speed, right_speed);
//                             break;
//                         }
//                         case MODE_STOP:{
//                             Motor_Stop();
//                             break;
//                         }
//                         Motor_Control(left_speed, right_speed);
//                         break;
//                     }
//                 }
//         }                
//     }
// }
#endif
