# 2026 TI 智能小车

基于 TI MSPM0G3507 的裸机小车工程。仓库中保留了巡线、IMU、电机、OLED、ST7735S 彩屏、MaxiCam、蓝牙串口和 HC-SR04 超声波等驱动代码。

当前可编译入口已经切换为 **HC-SR04 超声波 + ST7735S 彩屏联调程序**：`car_example.c` 前半段完整整车巡线程序被 `#if 0` 暂时停用，文件末尾的 `main()` 才是当前实际运行入口。

## 当前固件入口

当前 `main()` 位于 `car_example.c` 末尾，运行流程如下：

1. 调用 `SYSCFG_DL_init()` 初始化 SysConfig 生成的时钟、GPIO、SPI、I2C、UART、ADC、PWM、捕获定时器和 10 ms 控制定时器等外设。
2. 调用 `HC_SR04_Init()` 初始化 HC-SR04 的 TRIG 引脚、ECHO 捕获定时器中断和测距软件状态。
3. 调用 `ST7735S_Init(ST7735S_ROTATION_0)` 初始化 128x160 彩屏。
4. 在彩屏上显示 `HC-SR04 TEST`、`DIST:` 和初始距离 `---.-cm`。
5. 主循环每 10 ms 调用一次 `HC_SR04_Process10ms()`。
6. 当 `HC_SR04_TakeResult()` 返回新结果时，在 ST7735S 上刷新距离。

距离显示单位为厘米，保留 1 位小数；无有效回波、超时或超范围时显示 `---.-cm`。

## 已暂停的整车控制代码

`car_example.c` 中 `#if 0` 包裹的旧入口仍保留在仓库中，便于后续恢复。该部分包含：

| 模块 | 已保留的功能 |
| --- | --- |
| 八路灰度 | 12 位 ADC 采样、黑白标定数组、数字量识别和加权误差计算 |
| 双路电机 | PWM 限幅、方向控制、巡线和转向动作 |
| ICM45686 | 数据就绪中断、静止校准、偏航角和 Z 轴角速度读取 |
| OLED | 128x64 SSD1306 运行数据显示 |
| MaxiCam | UART 接收与坐标帧解析 |
| 状态机 | 巡线、路口前进、左转、右转、盲走恢复 |

如果要恢复整车控制，需要删除或改回 `car_example.c` 顶部的 `#if 0` 编译开关，并处理文件中当前末尾 HC-SR04 测试入口与旧入口之间的 `main()` 冲突。

## HC-SR04 测距实现

超声波驱动位于 `Driver/Sensor/hc_sr04.c/.h`。

| 项目 | 当前值 |
| --- | --- |
| 触发周期 | 每 6 个调度 tick 触发一次，即约 60 ms |
| 主循环调度周期 | 10 ms |
| TRIG 高电平宽度 | 12 us |
| 回波超时 | 3 个调度 tick，即约 30 ms |
| 有效脉宽范围 | 116 us 到 26100 us |
| 距离换算 | `distance_mm = (pulse_us * 10 + 29) / 58` |
| 滤波方式 | 最近最多 5 个有效样本的中值滤波 |

`HC_SR04_Process10ms()` 是非阻塞调度入口；ECHO 下降沿中断在 `HC_SR04_CAPTURE_INST_IRQHandler()` 中读取上升沿和下降沿捕获值并保存脉宽。

## 主要硬件接口

下表来自 `car_example.syscfg`。更改接线后应在 SysConfig 中修改引脚并重新生成配置文件。

| 模块 | 外设 | MCU 引脚 | 配置 |
| --- | --- | --- | --- |
| HC-SR04 TRIG | GPIO | PA1 | 输出，初始低电平 |
| HC-SR04 ECHO | TIMG8 Capture | PA0 | combined capture，CC1 下降沿中断 |
| ST7735S 数据 | SPI1 | SCK PB9，MOSI PB8 | Mode 0，单向写入 |
| ST7735S 控制 | GPIO | RES PB10，DC PB11，CS PB14，BLK PB26 | GPIO 控制 |
| OLED | I2C0 | SDA PA10，SCL PA11 | Fast mode |
| IMU | I2C1 | SDA PB3，SCL PB2，INT PA12 | Fast mode，INT 上升沿 |
| 电机 PWM | TIMA0 | A PA8，B PA9 | PWM 输出 |
| 电机方向 | GPIO | AIN1 PB15，AIN2 PB17，BIN1 PB13，BIN2 PB16 | 双 H 桥方向控制 |
| 灰度模拟量 | ADC0 | PA27 | 12 位 ADC |
| 灰度地址 | GPIO | AD0 PB20，AD1 PB24，AD2 PB25 | 选择 8 路模拟通道 |
| 调试/IMU 遥测 | UART2 | TX PA21，RX PA22 | 115200，DMA TX |
| MaxiCam | UART1 | TX PA17，RX PA18 | 9600 |
| 蓝牙 | UART3 | TX PA14，RX PA13 | 9600，RX 中断 |
| 按键 | GPIO | KEY1 PA28，KEY2 PA29，KEY3 PB21 | 上拉输入 |

## 目录结构

```text
.
├─ car_example.c                  当前 HC-SR04 + ST7735S 测试入口，旧整车入口被 #if 0 保留
├─ car_example.syscfg             MSPM0 外设与引脚配置
├─ test.c / test.h                OLED、电机、UART、蓝牙、MaxiCam 和 ST7735S 测试函数
├─ Driver/
│  ├─ Algorithm/                  Mahony、Kalman、PID 和轮询 UART 工具
│  ├─ BSP/                        ADC、delay、按键、蓝牙串口、MaxiCam 串口和 IMU 遥测串口
│  ├─ Display/                    SSD1306 OLED 与 ST7735S 驱动
│  ├─ IMU/                        MPU6050、ICM45686 与统一 IMU 后端
│  ├─ Module/                     IMU 上位机遥测封装
│  └─ Sensor/                     电机、灰度传感器和 HC-SR04 超声波
└─ targetConfigs/                 CCS 目标配置
```

`Debug/` 是 CCS 生成目录，仓库通过 `.gitignore` 排除该目录。

## 开发环境

| 项目 | 配置 |
| --- | --- |
| MCU | MSPM0G3507，LQFP-64(PM) |
| 内核 | Arm Cortex-M0+ |
| MSPM0 SDK | 2.10.00.04 |
| SysConfig | 1.26.2 |
| 编译器 | TI Arm Clang 4.0.4.LTS |
| 运行方式 | 裸机，无 RTOS |
| 调试配置 | SEGGER J-Link，见 `targetConfigs/MSPM0G3507.ccxml` |

## 构建

1. 在 Code Composer Studio 中导入当前目录下的 CCS 工程。
2. 安装 MSPM0 SDK 2.10.00.04，并让 CCS 使用 `car_example.syscfg` 中声明的产品版本。
3. 选择 MSPM0G3507 与 Debug 配置。
4. 执行 `Project > Clean`，再执行 `Project > Build Project`。

当前 README 仅根据仓库代码描述工程状态；没有在本次提交中重新确认 CCS 实机编译结果。

## 测试函数

`test.c/.h` 中保留了以下测试函数：

- `Test_OLED_Run()`
- `Test_Motor_Run()`
- `Test_UART_Run()`
- `Test_MaxiCamOLED_Run()`
- `Test_BluetoothOLED_Run()`
- `Test_ST7735S_ColorCycleRun()`

这些函数没有接入当前 `main()` 的入口选择逻辑。需要运行时，应临时改写入口或手动调用目标测试函数。

## 已知限制

- 当前默认固件只做 HC-SR04 距离显示，不会执行巡线、电机控制、IMU 遥测、MaxiCam 坐标消费或 OLED 刷新。
- `car_example.c` 中同时保留旧整车入口和当前测试入口，恢复整车代码时必须处理 `main()` 冲突。
- `README.html` 来自 TI 原始定时器示例，不描述当前小车固件。
- `Debug/` 下的目标文件、映射文件和自动生成 Makefile 不应提交。
