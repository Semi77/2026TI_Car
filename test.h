#ifndef TEST_H
#define TEST_H

#define TEST_MODE_ORIGINAL    0U
#define TEST_MODE_OLED        1U
#define TEST_MODE_MOTOR       2U

#ifndef MOTOR_TEST_SPEED
#define MOTOR_TEST_SPEED       1000
#endif

#define MOTOR_RUN_TIME_MS      1000U
#define MOTOR_STOP_TIME_MS     500U
#define MOTOR_CYCLE_PAUSE_MS   1500U

void Test_OLED_Run(void);
void Test_Motor_Run(void);

#endif
