# 2026 TI 智能小车控制工程

本工程面向 TI MSPM0G3507，包含双路直流电机控制、八路灰度巡线、ICM45686/MPU6050 姿态测量、SSD1306 OLED、ST7735S 彩屏、MaxiCam 坐标接收、蓝牙串口和调试串口等模块。工程使用 TI DriverLib 和 SysConfig，不依赖 RTOS。

仓库当前编译入口是 **ST7735S 彩屏自检程序**。完整小车控制程序与其他测试入口保留在 `car_example.c` 的 `#if 0` 条件编译块中，默认固件不会驱动电机、采集灰度或运行姿态解算。开发者在改动入口前必须确认全工程只保留一个 `main()`。

## 1. 当前固件行为

上电后程序按以下顺序执行：

1. `SYSCFG_DL_init()` 配置系统时钟、GPIO、PWM、I²C、UART、SPI、ADC、DMA 和定时器。
2. `Test_ST7735S_ColorCycleInit()` 初始化 ST7735S，显示方向为 `ST7735S_ROTATION_0`，有效分辨率为 128×160。
3. 主循环依次显示红、绿、蓝、白全屏画面，每种颜色保持 1 秒。
4. 每个画面绘制 `ST7735S`、当前颜色名称、字符 `#` 和整数 `-123`，用于检查填充、ASCII 字模和整数显示接口。

当前入口位于 `car_example.c` 文件末尾：

```c
int main(void)
{
    SYSCFG_DL_init();

    if (!Test_ST7735S_ColorCycleInit()) {
        while (1) {
        }
    }

    while (1) {
        Test_ST7735S_ColorCycleProcess();
    }
}
```

## 2. 目标硬件与开发环境

| 项目 | 工程配置 |
| --- | --- |
| MCU | MSPM0G3507 |
| 内核 | Arm Cortex-M0+，Thumb，软浮点 ABI |
| 封装 | LQFP-64（PM） |
| 系统主频 | 32 MHz |
| 开发方式 | 裸机、TI DriverLib、SysConfig |
| MSPM0 SDK | 2.10.00.04 |
| SysConfig | 1.26.2 |
| 编译器 | TI Arm Clang；工程产品记录为 4.0.2.00，代码生成选项记录为 4.0.4.LTS |
| 调试连接 | SEGGER J-Link，配置文件为 `targetConfigs/MSPM0G3507.ccxml` |
| 默认构建配置 | Debug，优化等级 `-O2`，DWARF 3 |

`.ccsproject` 中的原始示例路径仍指向 MSPM0 SDK 2.09.00.01，但 `car_example.syscfg` 和当前产品依赖已经切换到 2.10.00.04。导入工程后应以 SysConfig 文件声明的 2.10.00.04 为准。

## 3. 硬件接线与引脚配置

所有信号名来自 `car_example.syscfg`。PA/PB 编号表示 MSPM0G3507 端口引脚，修改接线时应先在 SysConfig 中调整，再重新生成 `ti_msp_dl_config.c/.h`。

### 3.1 ST7735S 彩屏

| 彩屏信号 | MCU 引脚 | 外设/方向 | 配置 |
| --- | --- | --- | --- |
| SCL/SCK | PB9 | SPI1 SCLK，输出 | Mode 0，8 MHz |
| SDA/MOSI | PB8 | SPI1 PICO，输出 | 8 位，MSB 先发 |
| RES/RST | PB10 | GPIO 输出 | 上电默认高，初始化时低电平复位 |
| DC/A0 | PB11 | GPIO 输出 | 低电平命令，高电平数据 |
| CS | PB14 | GPIO 输出 | 低电平有效，上电默认高 |
| BLK/LED | PB26 | GPIO 输出 | 高电平打开背光，上电默认低 |

驱动只使用单向写入，不配置 MISO。当前旋转表针对带偏移的 128×160 ST7735S 模组，方向 0/180 使用 `x=2、y=1` 偏移，方向 90/270 使用 `x=1、y=2` 偏移。若画面整体错位，应在 `Driver/st7735s.c` 的 `s_rotation_table` 中按屏幕批次调整偏移。

### 3.2 双路电机与电机驱动器

| 功能 | MCU 引脚 | 外设 | 说明 |
| --- | --- | --- | --- |
| 左轮/A 路 PWM | PA8 | TIMA0 CCP0 | 边沿对齐 PWM，32 kHz |
| 左轮/AIN1 | PB15 | GPIO 输出 | A 路方向控制 |
| 左轮/AIN2 | PB17 | GPIO 输出 | A 路方向控制 |
| 右轮/B 路 PWM | PA9 | TIMA0 CCP1 | 边沿对齐 PWM，32 kHz |
| 右轮/BIN1 | PB13 | GPIO 输出 | B 路方向控制 |
| 右轮/BIN2 | PB16 | GPIO 输出 | B 路方向控制 |

PWM 计数周期为 1000，`Motor_Control()` 将速度限制在 `-800～800`，因此现有驱动把最大占空比限制为约 80%。速度正负号决定 H 桥方向，绝对值决定 PWM 比较值。按照当前车辆安装方向，车体前进需要 A 轮反转、B 轮正转，`Motor_Forward(speed)` 实际调用 `Motor_Control(-speed, speed)`。

MCU 引脚只能连接电机驱动器的逻辑输入，不能直接驱动电机。电机电源应由驱动器和独立电源承担，MCU、传感器与驱动器必须共地。

### 3.3 八路灰度传感器

| 灰度模块信号 | MCU 引脚 | 配置 |
| --- | --- | --- |
| 模拟输出 | PA27 | ADC0 通道 0，12 位，参考电压 VDDA=3.3 V |
| 地址 AD0 | PB20 | GPIO 输出 |
| 地址 AD1 | PB24 | GPIO 输出 |
| 地址 AD2 | PB25 | GPIO 输出 |

驱动使用三根地址线选择 8 个通道，共用 PA27 读取模拟量。每个通道连续采样 8 次并取平均，一轮更新包含 64 次 ADC 转换。ADC 由软件触发，转换完成中断置位 `gCheckADC`，采样函数在等待期间执行 `__WFE()`。

`Driver/No_Mcu_Ganv_Grayscale_Sensor_Config.h` 中的关键设置如下：

- `Sensor_Edition = Class`。
- `Sensor_ADCbits = _12Bits`，归一化满量程为 4096。
- `Direction = 1`，驱动将物理通道顺序反转后写入数组。
- 数字量中 `0` 表示黑色，`1` 表示白色。
- 地址输出使用取反逻辑，修改灰度板型号时必须核对地址真值表。

完整小车程序当前使用统一的白场校准值 1800 和黑场校准值 300。驱动按每个通道计算两个迟滞阈值：

```text
白阈值 = (2 × 白场值 + 黑场值) / 3
黑阈值 = (白场值 + 2 × 黑场值) / 3
```

输入位于两个阈值之间时保留该通道上一次的数字状态。更换赛道、传感器高度或照明条件后，应实测 8 路黑白 ADC 值并更新 `g_gray_white[]`、`g_gray_black[]`。

### 3.4 ICM45686 / MPU6050 IMU

| IMU 信号 | MCU 引脚 | 外设配置 |
| --- | --- | --- |
| SDA | PB3 | I2C1 SDA，400 kHz |
| SCL | PB2 | I2C1 SCL，400 kHz |
| INT/DRDY | PA12 | GPIO 上升沿中断，优先级 1 |

当前后端由 `Driver/IMU/imu_backend_config.h` 选择：

```c
#define IMU_SELECTED_BACKEND IMU_BACKEND_ICM45686
```

ICM45686 使用 7 位地址 `0x69`，配置为加速度计 ±4 g、陀螺仪 ±1000 dps、输出数据率 200 Hz、低通带宽 ODR/4、低噪声模式。INT1 输出高电平脉冲，PA12 每次上升沿累计一个待处理样本，主循环中的 `IMU_DRDY_Process()` 按 5 ms/样本更新姿态。待处理计数最多累计 5 次，防止主循环长时间阻塞后产生过大的积分步长。

姿态解算使用六轴 Mahony 滤波器，不使用磁力计。俯仰角和横滚角可借助重力方向校正，偏航角只能对陀螺仪 Z 轴积分，长时间运行会产生漂移。

MPU6050 后端保留在工程中，可将宏切换为 `IMU_BACKEND_MPU6050`。该兼容后端只维护 Z 轴角速度和偏航角，三轴加速度、温度等通用接口返回占位值，切换前应补齐并验证功能。

两路 I²C 在 SysConfig 中均未启用内部上拉。SDA 和 SCL 应通过合适阻值上拉到 3.3 V，常用范围为 2.2 kΩ～10 kΩ，具体阻值取决于总线长度和模块自带电阻。

### 3.5 SSD1306 OLED

| OLED 信号 | MCU 引脚 | 外设配置 |
| --- | --- | --- |
| SDA | PA10 | I2C0 SDA，400 kHz |
| SCL | PA11 | I2C0 SCL，400 kHz |

OLED 驱动固定使用 7 位地址 `0x3C`，支持 128×64 屏幕、ASCII 字符、数字、中文点阵和位图。`OLED_DrawSensorLabels()` 与 `OLED_UpdateSensorValues()` 用于显示偏航角、Z 轴角速度、灰度数字量、小车状态以及第 0/7 路归一化值。

驱动提供 `oled_i2c_sda_unlock()`：当 SDA 被从机持续拉低时，函数临时把 SCL 改为 GPIO 并输出时钟，随后恢复 I²C0。硬件仍需外部上拉。

### 3.6 三路串口

三路 UART 均使用 8 数据位、无校验、1 停止位、无硬件流控。

| 用途 | 外设 | TX | RX | 波特率 | 接收/发送方式 |
| --- | --- | --- | --- | --- | --- |
| 调试与 IMU 上位机 | UART2 | PA21 | PA22 | 115200 | TX 可使用 DMA0 |
| MaxiCam | UART1 | PA17 | PA18 | 9600 | RX 中断解析 |
| 蓝牙模块 | UART3 | PA14 | PA13 | 9600 | RX 中断，128 字节环形缓冲区 |

MCU TX 应接模块 RX，MCU RX 应接模块 TX，并连接公共地。外部模块的串口电平必须兼容 3.3 V CMOS。

### 3.7 按键、调试口和定时器

| 功能 | 引脚/外设 | 配置 |
| --- | --- | --- |
| KEY1 | PA28 | 输入，内部上拉，按下通常读低电平 |
| KEY2 | PA29 | 输入，内部上拉，按下通常读低电平 |
| SWDIO | PA19 | SWD 调试数据 |
| SWCLK | PA20 | SWD 调试时钟 |
| 控制定时器 | TIMA1 | 10 ms 周期，中断优先级 2 |

按键目前没有接入活动入口。SWD 引脚应保留给 J-Link，除非硬件设计和固件同时改为其他用途。

## 4. SysConfig 外设总表

| SysConfig 实例 | 硬件实例 | 主要参数 | 代码用途 |
| --- | --- | --- | --- |
| `PWM_MOTOR` | TIMA0 | 32 MHz，周期 1000，边沿对齐 | 双电机 PWM，约 32 kHz |
| `CONTROL_TIMER` | TIMA1 | 总线时钟/8，装载值 39999 | 10 ms 控制节拍 |
| `I2C_OLED` | I2C0 | 400 kHz，控制器模式 | SSD1306 OLED |
| `I2C_MPU6050` | I2C1 | 400 kHz，控制器模式 | ICM45686 或 MPU6050 |
| `UART_TEST` | UART2 | 115200，FIFO，DMA TX | 调试打印和 56 字节 IMU 帧 |
| `UART_MAXI` | UART1 | 9600 | MaxiCam 坐标接收 |
| `UART_BLUETOOTH` | UART3 | 9600，FIFO，RX 中断 | 蓝牙收发 |
| `SPI_LCD` | SPI1 | Mode 0，8 位，MSB，8 MHz | ST7735S 单向写入 |
| `ADC12_0` | ADC0 | 12 位，重复模式，软件触发 | 八路灰度模拟输入 |
| `DMA_CH0` | DMA0 通道 0 | 字节宽度，源地址递增 | UART2 发送 |

`SYSCFG_DL_init()` 会初始化并启动 PWM_MOTOR 与 CONTROL_TIMER，即使当前彩屏测试不使用它们。电机方向 GPIO 上电默认为低，PWM 比较值默认为 0，因此上电后电机应保持停止。

## 5. 软件目录与模块职责

```text
.
├─ car_example.c                 程序入口、停用的整车状态机和串口测试
├─ car_example.syscfg            时钟、外设和引脚的唯一配置源
├─ test.c / test.h               OLED、电机、UART、蓝牙、彩屏测试
├─ Driver/
│  ├─ motor.c                    双路 H 桥方向和 PWM 控制
│  ├─ ADC.c                      灰度 ADC 阻塞式采样接口
│  ├─ No_Mcu_Ganv_*.c/.h         八路灰度选通、滤波、校准和二值化
│  ├─ gray.c                     简化的动态阈值巡线误差计算
│  ├─ oled_hardware_i2c.c        SSD1306 I²C 显示驱动
│  ├─ st7735s.c                  ST7735S SPI 显示驱动
│  ├─ maxicam_uart.c             MaxiCam 坐标帧状态机
│  ├─ bluetooth_uart.c           蓝牙串口环形接收缓冲区
│  ├─ uart.c                     轮询式 UART_Printf
│  ├─ pid.c / kalman.c           通用 PID 与卡尔曼算法
│  ├─ IMU/                       统一 IMU API、后端选择和 DRDY 调度
│  ├─ ICM45686/                  TDK InvenSense ICM45686 寄存器驱动
│  ├─ ICM45686_Port/             I²C 与板级适配
│  ├─ Algorithm/mahony6d.c       六轴 Mahony 姿态解算
│  ├─ Module/imu_host.c          上位机 IMU 帧封装与 50 Hz 调度
│  └─ BSP/imu_uart_port.c        UART2 DMA 发送适配
└─ targetConfigs/                MSPM0G3507 J-Link 调试配置
```

SysConfig 生成的 `ti_msp_dl_config.c/.h`、启动文件、链接脚本和构建产物位于 `Debug/`。该目录已被 `.gitignore` 排除，开发者必须在本机重新生成，不能把 `Debug/` 当作源码来源。

## 6. 完整小车控制逻辑

完整逻辑位于 `car_example.c` 第一个 `#if 0` 区域。该入口完成以下初始化：

1. 初始化全部 SysConfig 外设和 MaxiCam UART。
2. 使能 ADC0 转换完成中断，载入八路灰度黑白校准值。
3. 初始化 ICM45686，等待 2 秒后执行静止校准。
4. 初始化 IMU 数据就绪中断、UART2 DMA 上位机输出和 OLED。
5. 启动 TIMA1 10 ms 控制中断。
6. 主循环持续处理 MaxiCam、IMU 数据就绪、巡线状态机、OLED 刷新和 50 Hz IMU 上位机发送。

### 6.1 控制节拍

TIMA1 每 10 ms 进入 `CONTROL_TIMER_INST_IRQHandler()`，中断只执行三项轻量操作：

- `g_10ms_ticks` 加一。
- 设置 `g_10ms_flag`，通知主循环执行一轮控制。
- 调用 `IMU_HostTimerTick10ms()`，每两个节拍请求发送一帧 IMU 数据。

灰度采样、电机运算和 OLED 通信都在主循环执行。若一轮处理超过 10 ms，`g_10ms_flag` 只保留一个待处理标志，程序不会补做全部丢失周期。

### 6.2 巡线误差与电机输出

巡线状态对 8 路数字量使用权重：

```text
通道：  0   1   2   3   4   5   6   7
权重： -7  -5  -3  -1  +1  +3  +5  +7
```

程序对所有检测到黑线的通道求平均权重。没有通道检测到黑线时沿用上一次误差。PID 参数和速度限制为：

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `GRAY_PID_KP` | 30.0 | 比例项 |
| `GRAY_PID_KI` | 0.0 | 积分项，当前关闭 |
| `GRAY_PID_KD` | 5.0 | 微分项 |
| `MOTOR_BASE_SPEED` | 400 | 巡线基础速度 |
| `MOTOR_MAX_SPEED` | 800 | 单轮软件上限 |
| 积分限幅 | ±100 | 防止积分累积失控 |

输出修正后，左轮使用负号、右轮使用正号以适配当前电机安装方向。若车辆转向与误差方向相反，应先核对灰度通道顺序、电机极性和 `Direction`，不要只通过交换 PID 正负号掩盖接线问题。

### 6.3 小车状态机

| 状态 | 行为 | 进入/退出条件 |
| --- | --- | --- |
| `TRACK` | 灰度 PID 巡线 | 默认状态；检测路口或丢线后切换 |
| `ADVANCE_BEFORE_TURN` | 按盲走基础速度前进 | 保持 800 ms 后进入指定转向 |
| `TURN_LEFT` | 两轮均给负速度，原地左转 | 目标偏航为进入值 +83°，误差进入 ±5° 后停止 |
| `TURN_RIGHT` | 两轮均给正速度，原地右转 | 目标偏航为进入值 -83°，误差进入 ±5° 后停止 |
| `BLIND` | 不使用角度 PID，固定差速前进 | 左轮 300、右轮 280；重新检测黑线后回到巡线 |

状态判定使用以下约束：

- 进入新状态后至少等待 100 个控制周期，即 1 秒，再接受部分状态切换。
- 连续 3 个周期满足条件才确认丢线或重新找到黑线，相当于 30 ms 去抖。
- 低 4 位全黑且高 4 位未全黑时判为左转特征；高 4 位全黑且低 4 位未全黑时判为右转特征。
- 8 路均为白色时进入盲走。
- 偏航误差经过 ±180° 环绕处理，避免跨越 -180°/180° 时选择错误转向方向。

`GYRO_PID_KP/KI/KD` 和 `GYRO_PID_MAX_OUTPUT` 已定义，但当前盲走代码没有使用角度 PID。车辆依靠 `BLIND_LEFT_COMPENSATION=20` 做固定左轮补偿。

### 6.4 IMU 上电校准

完整小车入口在初始化 IMU 后等待 2 秒，再调用 `IMU_Calibrate()`。校准流程要求车辆静止：

- 连续 20 个样本满足静止条件后开始统计。
- 收集 200 个有效样本，单次采样间隔 5 ms。
- 最多尝试 800 次，约 4 秒。
- 加速度模长必须位于 850～1150 mg。
- 任一轴角速度绝对值不得超过 5 dps。
- Z 轴静态方差用于生成 `0.05～0.50 dps` 的动态死区。

读取失败时完整小车入口立即停车并停在死循环。校准质量告警不会阻止启动，程序使用已获得的最佳样本继续运行。上电校准期间应将车辆放在水平、静止且电机不振动的位置。

## 7. 通信协议

### 7.1 MaxiCam 坐标帧

MaxiCam 驱动接收固定格式的 ASCII 帧：

```text
#XXXX|YYYY$
```

`XXXX` 和 `YYYY` 必须各包含 4 个十进制数字，例如：

```text
#0123|0456$
```

UART1 中断逐字节运行状态机。收到完整帧后，中断区只保存待处理坐标，主循环调用 `MaxiCam_UART_Process()` 转移数据，再通过 `MaxiCam_UART_GetLatestPoint()` 读取一次。新帧会覆盖尚未取走的上一帧。

### 7.2 蓝牙串口

蓝牙驱动使用 128 字节环形缓冲区。RX 中断写入，主循环通过 `Bluetooth_UART_Read()` 批量读取。缓冲区满时丢弃新字节并增加溢出计数，可用 `Bluetooth_UART_GetOverflowCount()` 诊断主循环处理不及时的问题。

保留的蓝牙 OLED 测试会把可打印 ASCII 字符显示到 OLED，`` 被忽略，`\n` 换行，非可打印字节显示为 `?`。屏幕写满 8 行后自动清屏。

### 7.3 IMU 上位机帧

`imu_host.c` 每 20 ms 构造一个 56 字节二进制帧，通过 UART2 DMA 发送：

| 偏移 | 长度 | 内容 |
| ---: | ---: | --- |
| 0 | 2 | 帧头 `AA 55` |
| 2 | 1 | 类型 `02` |
| 3 | 1 | 载荷长度 `36` |
| 4 | 6 | 三轴加速度，`int16` 小端，比例 2048 LSB/g |
| 10 | 6 | 三轴角速度，`int16` 小端，比例 16.4 LSB/(dps) |
| 16 | 12 | Roll/Pitch/Yaw，3 个 IEEE-754 `float` 小端 |
| 28 | 2 | 温度，`int16` 小端，单位 0.01 ℃ |
| 30 | 12 | 三轴加速度，3 个 `float`，单位 g |
| 42 | 12 | 三轴角速度，3 个 `float`，单位 dps |
| 54 | 1 | 从字节 2 到 53 的 8 位累加校验和 |
| 55 | 1 | 帧尾 `5A` |

DMA 正忙时新帧不会覆盖发送缓冲区，驱动增加丢帧计数。可通过 `IMU_UART_GetDropCount()` 检查 UART 带宽或调度问题。

## 8. 选择运行入口

`car_example.c` 目前保留四类入口/测试。`test.h` 中的 `TEST_MODE_*` 宏没有连接到入口选择逻辑，修改这些宏不会切换固件。

| 入口 | 当前状态 | 位置/调用 |
| --- | --- | --- |
| 完整小车控制 | `#if 0` 停用 | 文件顶部的大型条件编译块 |
| MaxiCam + OLED 测试 | `#if 0` 停用 | 第二个条件编译块 |
| 蓝牙 + OLED 测试 | `#if 0` 停用 | 文件后部 `Test_BluetoothOLED_*` 入口 |
| ST7735S 四色测试 | 启用 | 文件末尾默认 `main()` |

切换入口时执行以下检查：

1. 将目标入口外层的 `#if 0` 改为 `#if 1`。
2. 将当前 ST7735S `main()` 或其他入口置于 `#if 0`，确保预处理后只存在一个 `main()`。
3. 检查目标模块的硬件已经连接，电机测试前把车轮架空。
4. 清理并重新构建工程，确认链接器没有报告重复入口。

`Test_Motor_Run()` 内的 `MOTOR_TEST_SPEED` 默认是 1000，但 `Motor_Control()` 会把它限制为 800。该测试会按左轮正反转、右轮正反转、双轮同向正反转的顺序循环，首次运行必须断开车轮负载或架空车体。

## 9. 编译、生成与烧录

### 9.1 导入工程

1. 安装支持 MSPM0 的 Code Composer Studio，并安装 MSPM0 SDK 2.10.00.04、SysConfig 1.26.2 和 TI Arm Clang。
2. 在 CCS 中选择 **Import Project**，导入包含 `.project` 和 `.ccsproject` 的本目录。
3. 若 CCS 提示产品版本不匹配，优先安装工程声明版本；如需升级版本，先备份并检查 SysConfig 生成差异。
4. 打开 `car_example.syscfg`，确认器件为 MSPM0G3507、封装为 LQFP-64(PM)，让 SysConfig 重新生成配置文件。

### 9.2 构建

1. 选择 Debug 配置。
2. 执行 **Project → Clean**。
3. 执行 **Project → Build Project**。
4. 生成的固件为 `Debug/timx_timer_mode_periodic_sleep.out`，同时生成 `.map` 和链接信息文件。

工程使用 `Debug/` 中的自动生成 makefile。不要手工编辑 `Debug/ti_msp_dl_config.c` 或 `.h`，因为下次运行 SysConfig 会覆盖这些文件。外设和引脚改动应写入 `car_example.syscfg`。

### 9.3 烧录与调试

1. 使用 J-Link 连接 SWDIO、SWCLK、GND 和目标板参考电压。
2. 在 CCS 中确认活动目标配置为 `targetConfigs/MSPM0G3507.ccxml`。
3. 编译后启动 Debug，下载 `.out` 文件并运行。
4. 当前默认入口应显示红、绿、蓝、白四色循环；若屏幕无反应，先检查 PB9/PB8 波形、CS/DC/RES 电平和背光控制。

如果使用 LaunchPad 板载 XDS110，需要在 CCS 中新建或替换目标配置，现有 `.ccxml` 只声明 SEGGER J-Link。

## 10. 调试顺序

建议按硬件风险从低到高验证：

1. 运行 ST7735S 四色测试，确认系统时钟和 SPI。
2. 运行 UART 测试，确认 115200 8N1 输出。
3. 运行 OLED 或蓝牙 OLED 测试，确认 I²C0 和 UART3。
4. 单独读取灰度 ADC，记录每路黑白标定值。
5. 初始化 IMU，静止观察偏航、零偏、方差和死区。
6. 将车轮架空后运行电机测试，核对 A/B 路方向和 PWM。
7. 启用完整小车入口，先用低速验证灰度误差方向，再调整 PID、转角和盲走补偿。

## 11. 已知限制与开发注意事项

- 默认固件只运行 ST7735S 自检，不能代表整车控制已经启用。
- 完整小车代码放在 `car_example.c` 的条件编译块中，后续开发宜逐步迁移到独立业务模块，保持主循环只负责初始化和任务调度。
- `OLED_WR_Byte()` 的超时判断依赖 `g_ms_counter`，但当前工程没有递增该计数器。I²C 总线异常时，该超时路径不能按设计退出，启用 OLED 前应补充可靠时基或改为有限次数轮询。
- `ADC.c` 使用全局完成标志和阻塞等待，只适合当前单调用者结构。不要在中断和主循环中同时调用 `adc_getValue()`。
- ICM45686 I²C 读写一次最多传输 16 字节，超时后会复位控制器 FIFO。
- 六轴姿态算法没有绝对航向参考，偏航会随时间漂移。精确转角依赖良好的静止校准和较短运行时间。
- `pid.c` 中声明了积分和输出限幅字段，但 `PID_Compute()` 当前没有执行限幅；完整巡线入口使用自己的 PID 运算和限幅逻辑。
- `gray.c` 的 `Gray_Test()` 与完整小车入口中的灰度算法是两套独立实现，修改其中一处不会同步另一处。
- `README.html` 是 TI 原始示例说明，内容仍描述定时器闪灯示例，不代表本工程现状。开发请以本文件和源码为准。
- 屏幕、IMU、蓝牙模块和电机驱动板的供电能力取决于具体模块版本。接线前应核对模块原理图，确保电源电压、逻辑电平和峰值电流符合器件要求。

## 12. 修改配置时的核对清单

- 改引脚：修改 `car_example.syscfg`，重新生成，核对接线表和驱动宏名。
- 改系统时钟：检查所有基于 `CPUCLK_FREQ` 的延时、SPI、UART、PWM 和定时器周期。
- 改 PWM 周期：同步检查 `MAX_SPEED`、测试速度和电机驱动器允许的 PWM 频率。
- 改灰度板方向：核对 `Direction`、通道权重、左右路口位图和电机修正方向。
- 改 IMU 型号：切换 `IMU_SELECTED_BACKEND`，检查 I²C 地址、量程、ODR、中断极性和返回单位。
- 改入口：保证全工程只有一个 `main()`，清理后重新链接。
- 提交代码：提交源码、SysConfig 和工程元数据，不提交 `Debug/` 生成目录。

## 13. 关键参数索引

| 参数 | 文件 | 当前值 |
| --- | --- | ---: |
| 彩屏颜色保持时间 | `test.c` | 1000 ms |
| 电机速度限制 | `Driver/motor.c` | ±800 |
| 灰度控制周期 | `car_example.c` | 10 ms |
| OLED 刷新周期 | `car_example.c` | 100 ms |
| 巡线基础速度 | `car_example.c` | 400 |
| 盲走基础速度 | `car_example.c` | 280 |
| 盲走左轮补偿 | `car_example.c` | 20 |
| 原地转向速度 | `car_example.c` | 180 |
| 目标转角 | `car_example.c` | 83° |
| 转角容差 | `car_example.c` | ±5° |
| 转向前直行时间 | `car_example.c` | 800 ms |
| ICM45686 数据率 | `icm45686_port.c` | 200 Hz |
| IMU 上位机发送率 | `imu_host.c` | 50 Hz |
| 蓝牙 RX 缓冲区 | `bluetooth_uart.c` | 128 字节 |

开发者应在每次实车调参后同步更新本表和对应章节，使文档中的数值与源码保持一致。
