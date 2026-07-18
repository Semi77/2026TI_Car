# 2026TI_Car

基于 TI MSPM0 系列的智能小车工程，当前仓库同时保留 `MSPM0G3507` 和 `MSPM0G3519` 两套工程。

## 当前状态

- `mspm0g3507`：原始开发版本，包含完整小车驱动、IMU 抽象接口、后端适配层和测试代码。
- `mspm0g3519`：已从 3507 迁移主要驱动和整车控制逻辑，目标芯片为 `MSPM0G3519`，封装为 `LQFP-100(PZ)`。
- `mspm0g3519` 当前主控不使用 `IMU.h` 抽象接口，而是直接使用串口陀螺仪驱动 `Driver/IMU/uart_gyro` 获取 Yaw 和 Z 轴角速度。

## 目录结构

```text
.
├─ mspm0g3507/
│  ├─ car_example.c
│  ├─ car_example.syscfg
│  └─ Driver/
│     ├─ Algorithm/
│     ├─ BSP/
│     ├─ Display/
│     ├─ IMU/
│     ├─ Module/
│     └─ Sensor/
├─ mspm0g3519/
│  ├─ empty.c
│  ├─ empty.syscfg
│  ├─ test.c
│  ├─ test.h
│  └─ Driver/
│     ├─ Algorithm/
│     ├─ BSP/
│     ├─ Display/
│     ├─ IMU/
│     │  ├─ ICM45686/
│     │  ├─ MPU6050/
│     │  └─ uart_gyro/
│     └─ Sensor/
└─ .gitignore
```

## MSPM0G3519 工程说明

`mspm0g3519/empty.c` 是当前 3519 工程的主程序入口，主要功能包括：

- 灰度巡线 PID 控制
- 丢线盲走
- 左右转弯状态机
- 串口陀螺仪 Yaw 角度闭环转向
- HC-SR04 超声波测距
- OLED 状态显示
- ST7735S 彩屏距离和按键显示
- MaixCam 串口处理
- 按键扫描
- 电机 PWM 和方向控制

启动流程中会初始化串口陀螺仪，执行零偏校准和 Yaw 清零，然后等待第一帧 Yaw 数据。如果没有等到串口陀螺仪数据，程序会保持停车。

## MSPM0G3519 外设配置

`mspm0g3519/empty.syscfg` 当前配置基于 `MSPM0G3519 / LQFP-100(PZ)`，主要外设如下：

| 模块名 | 用途 |
| --- | --- |
| `ADC12_0` | 灰度传感器模拟采样 |
| `PWM_MOTOR` | 电机 PWM 输出 |
| `MOTOR_AIN` / `MOTOR_BIN` | 电机方向控制 GPIO |
| `Gray_Address` | 灰度传感器地址选择 GPIO |
| `HC_SR04_TRIG` | 超声波触发 GPIO |
| `HC_SR04_CAPTURE` | 超声波 Echo 捕获 |
| `I2C_OLED` | OLED 显示 |
| `SPI_LCD` | ST7735S 彩屏 |
| `GPIO_LCD_CONTROL` | 彩屏 RES/DC/CS/BLK 控制 |
| `KEY_INPUT_A` / `KEY_INPUT_B` | 按键输入 |
| `UART_MAXI` | MaixCam 串口 |
| `UART_BLUETOOTH` | 蓝牙串口 |
| `UART_GYRO` | 串口陀螺仪 |
| `CONTROL_TIMER` | 10ms 控制定时器 |

SysConfig 生成时目前存在一个警告：`UART_TEST` 的建议外设分配在 3519 上会被自动调整，但生成结果无错误。

## IMU 迁移策略

3519 工程没有迁移 3507 的 IMU 抽象接口层：

- 未使用 `Driver/IMU/IMU.h`
- 未使用 `Driver/IMU/imu_backend_config.h`
- 未使用 `Driver/IMU/imu_mpu6050_backend.c`
- 未使用 `Driver/IMU/Backend/*`
- 未使用 `Driver/Module/imu_host.*`

当前实际参与主控的陀螺仪代码是：

```text
mspm0g3519/Driver/IMU/uart_gyro/uart_gyro.c
mspm0g3519/Driver/IMU/uart_gyro/uart_gyro.h
```

`ICM45686` 和 `MPU6050` 底层驱动文件已保留在 3519 工程中，但当前主控没有直接调用它们。

## 构建环境

建议使用：

- Code Composer Studio Theia
- MSPM0 SDK `2.11.00.07`
- SysConfig `1.26.2`
- TI Arm Clang `4.0.4.LTS` 或兼容版本

`Debug/` 是本地构建输出目录，已被 `.gitignore` 排除，不提交 `.out`、`.o`、`.map` 等编译产物。

## 构建步骤

1. 在 CCS 中导入 `mspm0g3519` 工程。
2. 打开 `empty.syscfg`，确认目标为 `MSPM0G3519 / LQFP-100(PZ)`。
3. 重新生成 SysConfig 文件。
4. 选择 Debug 配置并构建。
5. 烧录前确认实际接线与 `empty.syscfg` 中的引脚一致。

## 注意事项

- `mspm0g3519/empty.c` 中所有主控状态机的角度数据来自串口陀螺仪缓存值。
- 串口陀螺仪使用 `UART_GYRO`，波特率为 `115200`。
- 启动校准期间小车应保持静止。
- 3507 和 3519 的封装不同，若硬件接线与当前 SysConfig 不一致，需要优先调整 `empty.syscfg`。
- 修改或新增 C 函数时，请保持每个函数前有一句简体中文功能注释。
