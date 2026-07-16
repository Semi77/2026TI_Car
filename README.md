# 2026 TI 智能小车

基于 TI MSPM0G3507 的裸机小车控制工程。当前固件使用八路灰度传感器巡线，以 ICM45686 提供偏航角和角速度，驱动双路直流电机，并在 SSD1306 OLED 上显示运行数据。

工程包含 ST7735S 彩屏、MPU6050、MaxiCam 和蓝牙串口驱动。默认入口仍直接使用 OLED 和 ICM45686；ST7735S 当前用于独立显示测试，屏幕统一接口尚未接入主程序。

## 当前固件

`car_example.c` 提供唯一的 `main()`。上电后程序执行以下流程：

1. 调用 `SYSCFG_DL_init()` 初始化时钟、GPIO、ADC、PWM、I²C、SPI、UART、DMA 和控制定时器。
2. 初始化 MaxiCam 串口与八路灰度传感器。
3. 初始化 ICM45686，等待 2 秒后执行静止校准。初始化或读取失败时停止电机并停留在故障循环。
4. 启用 IMU 数据就绪中断、UART2 DMA 遥测和 OLED。
5. TIMA1 每 10 ms 触发一次灰度采样、状态判断和电机控制。
6. OLED 每 100 ms 刷新偏航角、Z 轴角速度、灰度数字量、车辆状态以及第 0/7 路归一化值。
7. IMU 遥测按 50 Hz 发送 56 字节帧。

车辆控制状态包括：

| 状态 | 行为 |
| --- | --- |
| `CAR_STATE_TRACK` | 按八路灰度加权误差执行 PD 巡线 |
| `CAR_STATE_ADVANCE_BEFORE_TURN` | 检测到路口后先直行 800 ms |
| `CAR_STATE_TURN_LEFT` | 根据 IMU 偏航角左转约 83° |
| `CAR_STATE_TURN_RIGHT` | 根据 IMU 偏航角右转约 83° |
| `CAR_STATE_BLIND` | 丢线后按固定左右轮补偿直行，重新检测到黑线后恢复巡线 |

## 功能状态

| 模块 | 当前状态 |
| --- | --- |
| 双路电机 | 已接入默认固件，PWM 比较值限制在 `0..800` |
| 八路灰度 | 已接入默认固件，12 位 ADC，数字量 `0` 表示黑色 |
| ICM45686 | 默认 IMU 后端，200 Hz 数据就绪中断，六轴 Mahony 姿态解算 |
| MPU6050 | 保留兼容后端；加速度、温度等通用接口仍返回占位数据 |
| SSD1306 OLED | 默认显示设备，I²C 地址 `0x3C`，分辨率 128×64 |
| ST7735S | SPI 驱动和红绿蓝白循环测试已实现，默认入口未调用 |
| MaxiCam | UART 接收与帧解析已初始化，当前车辆状态机未消费坐标 |
| 蓝牙串口 | 环形接收缓冲和 OLED 联调函数已实现，默认入口未调用 |

## 开发环境

| 项目 | 配置 |
| --- | --- |
| MCU | MSPM0G3507，LQFP-64(PM) |
| 内核 | Arm Cortex-M0+ |
| 系统主频 | 32 MHz |
| MSPM0 SDK | 2.10.00.04 |
| SysConfig | 1.26.2 |
| 编译器 | TI Arm Clang 4.0.4.LTS |
| 运行方式 | 裸机，无 RTOS |
| 调试配置 | SEGGER J-Link，见 `targetConfigs/MSPM0G3507.ccxml` |

## 目录结构

```text
.
├─ car_example.c                  默认小车控制入口
├─ car_example.syscfg             MSPM0 外设与引脚配置
├─ test.c / test.h                OLED、电机、UART、蓝牙、MaxiCam 和彩屏测试
├─ Driver/
│  ├─ Algorithm/                  Mahony、Kalman、PID 和轮询 UART 工具
│  ├─ BSP/                        ADC、串口端口与 delay_ms/delay_us
│  ├─ Display/                    SSD1306 OLED 与 ST7735S 驱动
│  ├─ IMU/
│  │  ├─ Backend/ICM45686/        ICM45686 通用 IMU 接口实现
│  │  ├─ ICM45686/                芯片驱动和 MSPM0 I²C 适配
│  │  └─ MPU6050/                 MPU6050 驱动
│  ├─ Module/                     IMU 上位机遥测封装
│  └─ Sensor/                     电机与八路灰度传感器
└─ targetConfigs/                 CCS 目标配置
```

`Debug/` 是 CCS 生成目录，仓库通过 `.gitignore` 排除该目录。

## 主要硬件接口

下表来自 `car_example.syscfg`。更改接线后应在 SysConfig 中修改引脚并重新生成配置文件。

| 模块 | 外设 | MCU 引脚 | 配置 |
| --- | --- | --- | --- |
| OLED | I2C0 | SDA PA10，SCL PA11 | 400 kHz |
| IMU | I2C1 | SDA PB3，SCL PB2，INT PA12 | 400 kHz，INT 上升沿 |
| ST7735S 数据 | SPI1 | SCK PB9，MOSI PB8 | Mode 0，8 MHz，MSB first |
| ST7735S 控制 | GPIO | RES PB10，DC PB11，CS PB14，BLK PB26 | 单向写入，无 MISO |
| 电机 PWM | TIMA0 | A PA8，B PA9 | 边沿对齐 PWM |
| 电机方向 | GPIO | AIN1 PB15，AIN2 PB17，BIN1 PB13，BIN2 PB16 | 双 H 桥方向控制 |
| 灰度模拟量 | ADC0 | PA27 | 12 位，软件触发 |
| 灰度地址 | GPIO | AD0 PB20，AD1 PB24，AD2 PB25 | 选择 8 路模拟通道 |
| 调试/IMU 遥测 | UART2 | TX PA21，RX PA22 | 115200，DMA TX |
| MaxiCam | UART1 | TX PA17，RX PA18 | 9600 |
| 蓝牙 | UART3 | TX PA14，RX PA13 | 9600，RX 中断 |

## 关键参数

默认参数集中在 `car_example.c`：

| 参数 | 当前值 |
| --- | ---: |
| 控制周期 | 10 ms |
| OLED 刷新周期 | 100 ms |
| 巡线基础速度 | 400 |
| 电机速度上限 | 800 |
| 灰度 PID | `Kp=30.0, Ki=0.0, Kd=5.0` |
| 盲走基础速度 | 280 |
| 盲走左轮补偿 | 20 |
| 转向速度 | 180 |
| 目标转角 | 83° |
| 转角容差 | ±5° |
| 路口转向前直行 | 800 ms |

灰度初始标定值为每路白场 `1800`、黑场 `300`。更换赛道、传感器高度或照明后，应重新采集 8 路黑白值并更新 `g_gray_white[]` 与 `g_gray_black[]`。

## IMU 后端

`Driver/IMU/imu_backend_config.h` 在编译期选择 IMU：

```c
#define IMU_BACKEND_MPU6050    (1)
#define IMU_BACKEND_ICM45686   (2)

#define IMU_SELECTED_BACKEND   IMU_BACKEND_ICM45686
```

两个后端实现同一组 `IMU_*` 接口。工程当前按 ICM45686 配置数据就绪中断；切换到 MPU6050 前，需要核对量程、数据周期和返回数据完整性。

## 显示模块现状

显示驱动位于 `Driver/Display/`：

- `oled_hardware_i2c.c/.h` 驱动 SSD1306，提供字符、数字、中文点阵和位图接口。
- `st7735s.c/.h` 驱动 128×160 ST7735S，提供旋转、填充、RGB565 位图、字符和整数接口。

默认固件直接调用 `OLED_Init()` 和 `OLED_UpdateSensorValues()`。两种显示设备尚未通过统一的 `Screen_*` 接口选择，因此当前构建会编译两套驱动，但主程序只使用 OLED。`Test_ST7735S_ColorCycleRun()` 可用于验证彩屏。

`delay_ms()` 和 `delay_us()` 已统一放在 `Driver/BSP/delay.c`。传感器或显示驱动不再负责提供公共延时函数。

## 构建

1. 在 Code Composer Studio 中导入当前目录下的 CCS 工程。
2. 安装 MSPM0 SDK 2.10.00.04，并让 CCS 使用 `car_example.syscfg` 中声明的产品版本。
3. 选择 MSPM0G3507 与 Debug 配置。
4. 执行 `Project > Clean`，再执行 `Project > Build Project`。

本次提交使用 TI Arm Clang 4.0.4.LTS 完成 Debug 编译和链接，目标文件为：

```text
Debug/timx_timer_mode_periodic_sleep.out
```

## 测试入口

`test.c/.h` 包含以下测试函数：

- `Test_OLED_Run()`
- `Test_Motor_Run()`
- `Test_UART_Run()`
- `Test_MaxiCamOLED_Run()`
- `Test_BluetoothOLED_Run()`
- `Test_ST7735S_ColorCycleRun()`

`test.h` 中的 `TEST_MODE_*` 宏没有连接到入口选择逻辑。运行测试前需要临时让唯一的 `main()` 调用目标测试函数，测试完成后恢复小车入口。

电机测试前应架空车轮并准备断开电机电源。默认小车入口在 IMU 校准完成后会进入巡线控制。

## 已知限制

- OLED I²C 超时逻辑依赖 `g_ms_counter`，当前工程没有递增该计数器。总线被拉低时，相关等待路径可能无法按预期超时。
- MPU6050 后端只维护 Z 轴角速度和偏航角，通用三轴数据接口尚未补齐。
- 六轴姿态算法没有磁力计提供绝对航向，偏航角会随时间漂移。
- `ADC.c` 使用全局完成标志和阻塞等待，不适合在中断与主循环中并发调用。
- 完整车辆状态机仍集中在 `car_example.c`，主函数尚未完成业务模块拆分。
- `README.html` 来自 TI 原始定时器示例，不描述当前小车固件。

## 提交约定

提交源码、SysConfig 文件和 CCS 工程元数据。不要提交 `Debug/` 下的目标文件、映射文件和自动生成的 Makefile。
