# nuedc_e_car_mspm0

## 1. 项目简介

本仓库是 **2025 年全国大学生电子设计竞赛 E 题：简易自行瞄准装置** 中小车部分的 TI MSPM0G3507 迁移工程。

当前工程面向：

- 立创·天猛星 MSPM0G3507 高配版开发板
- TI MSPM0G3507
- Keil MDK
- TI MSPM0 SDK
- SysConfig
- MSPM0 DriverLib

工程来源于原 STM32F103 E 题小车工程：

- <https://github.com/Greyd0ve/nuedc-e-line-car>

本仓库目标是将原有 `App/Control` 小车逻辑迁移到 MSPM0G3507，并用 MSPM0 DriverLib 重写底层硬件驱动。

本仓库当前只包含自动寻迹小车部分代码，不包含二维云台、激光瞄准、视觉识别或瞄准模块控制代码。

迁移记录可参考仓库根目录下的 `migration_report_mspm0.md`。`build_mspm0_port.log` 是本地构建日志，默认不建议提交。

## 2. 当前状态

- 当前状态：第一轮可编译迁移完成
- 构建结果：`0 Error(s), 0 Warning(s)`
- 烧录状态：尚未烧录当前小车工程
- 实测状态：尚未上车实测
- 安全状态：默认不启动车轮

已完成：

- `App/Control` 主体逻辑已迁入
- `Hardware/System` 已重写为 MSPM0 DriverLib 版本
- Keil Rebuild 通过：`0 Error(s), 0 Warning(s)`

尚未完成：

- 当前小车工程烧录验证
- 实车上车测试
- 完整循迹闭环调参

## 3. 题目相关要求

E 题中与小车相关的关键要求：

- 小车寻迹和电机控制必须使用 TI MSPM0 系列 MCU
- 行驶轨迹为 `100cm x 100cm` 正方形黑线
- 黑色轨迹线宽约 `1.8cm ± 0.2cm`
- 小车逆时针循迹
- 圈数 `N` 可设定为 `1~5`
- 基本要求小车 `N` 圈行驶时间 `t <= 20s`
- 行驶过程中不得遥控小车运动
- 小车尺寸不超过 `25cm x 15cm x 25cm`

## 4. 硬件平台

- MCU：TI MSPM0G3507
- 开发板：立创·天猛星 MSPM0G3507 高配版开发板
- 电机驱动：TB6612FNG
- 电机：左右两路直流减速电机
- 编码器：左右两路正交编码器
- 灰度传感器：CD4051 八路灰度采集
- 显示：OLED，当前为临时 stub，I2C 真驱动尚未接入
- 通信：UART1，另预留可选调试串口引脚
- 输入：4 路按键
- 下载调试：SWD，当前使用 J-Link 兼容下载器

## 5. IO 分配

### TB6612FNG

| 信号 | MSPM0 引脚 | 作用 |
| --- | --- | --- |
| PWMA | PA12 / A12 | 左电机 PWM |
| AIN1 | PB06 / B06 | 左电机方向 1 |
| AIN2 | PB07 / B07 | 左电机方向 2 |
| PWMB | PA13 / A13 | 右电机 PWM |
| BIN1 | PB08 / B08 | 右电机方向 1 |
| BIN2 | PB09 / B09 | 右电机方向 2 |
| STBY | PB23 / B23 | TB6612 STBY |

### 编码器

| 信号 | MSPM0 引脚 | 作用 |
| --- | --- | --- |
| E1A | PB10 / B10 | 左编码器 A 相，中断输入 |
| E1B | PB11 / B11 | 左编码器 B 相，普通输入 |
| E2A | PB00 / B00 | 右编码器 A 相，中断输入 |
| E2B | PB01 / B01 | 右编码器 B 相，普通输入 |

### 灰度 CD4051

| 信号 | MSPM0 引脚 | 作用 |
| --- | --- | --- |
| OUT | PA14 / A14 | CD4051 输出采样 |
| AD0 | PA15 / A15 | CD4051 地址位 0 |
| AD1 | PA16 / A16 | CD4051 地址位 1 |
| AD2 | PA17 / A17 | CD4051 地址位 2 |

### UART

| 信号 | MSPM0 引脚 | 作用 |
| --- | --- | --- |
| UART1_TX | PA08 / A08 | UART1 发送 |
| UART1_RX | PA09 / A09 | UART1 接收 |
| DBG_TX | PA10 / A10 | 可选调试发送 |
| DBG_RX | PA11 / A11 | 可选调试接收 |

### 按键

| 信号 | MSPM0 引脚 | 作用 |
| --- | --- | --- |
| K1 | PB15 / B15 | 按键 1 |
| K2 | PB16 / B16 | 按键 2 |
| K3 | PB02 / B02 | 按键 3 |
| K4 | PB03 / B03 | 按键 4 |

### OLED

| 信号 | MSPM0 引脚 | 作用 |
| --- | --- | --- |
| OLED SDA | PA00 / A00 | OLED I2C 数据线 |
| OLED SCL | PA01 / A01 | OLED I2C 时钟线 |

### SWD

| 信号 | 说明 |
| --- | --- |
| 3V3 | 目标板电源参考 |
| GND | 地 |
| SWDIO / DIO | SWD 数据 |
| SWCLK / CLK | SWD 时钟 |
| RST | 复位 |

## 6. 软件架构

主要目录：

| 目录 | 说明 |
| --- | --- |
| `User/` | 入口与主循环 |
| `App/` | E 题小车应用逻辑、串口命令、循迹逻辑 |
| `Control/` | PID 等控制算法 |
| `Hardware/` | MSPM0G3507 硬件驱动封装 |
| `System/` | 定时器与系统调度 |
| `keil/` | Keil MDK 工程文件 |

迁移关系：

- `App/Control`：从原 STM32 工程迁移，保留上层小车控制逻辑
- `Hardware/System`：使用 MSPM0 DriverLib 重写，替代 STM32 标准外设库访问
- `ti_msp_dl_config.c/.h`：由 SysConfig 生成，提供底层外设初始化配置

## 7. 当前真实驱动与临时 Stub

真实 MSPM0 DriverLib 驱动：

- PWM
- Motor
- Encoder
- Grayscale
- Key
- Serial
- Timer

临时 stub：

- OLED
- BeepLed

OLED 与 BeepLed 当前用于保证迁移阶段可编译和安全占位，后续需要按实际硬件补齐真驱动。

## 8. 开发环境

| 项目 | 路径或版本 |
| --- | --- |
| Keil | `D:\Keil_v5_38a\UV4\UV4.exe` |
| Keil 版本 | `µVision V5.39.0.0` |
| Compiler | `ArmClang V6.21` |
| MSPM0 SDK | `D:\ti2\mspm0_sdk_2_02_00_05` |
| SysConfig | `C:\ti\sysconfig_1.21.1` |
| J-Link | `D:\Program Files (x86)\SEGGER\JLink\JLink.exe` |
| J-Link 版本 | `V6.98e` |

Flash Algorithm：

- `MSPM0G1X0X_G3X0X_MAIN_128KB.FLM`
- 路径：`C:\Users\zch20\AppData\Local\Arm\Packs\TexasInstruments\MSPM0G1X0X_G3X0X_DFP\1.3.1\02_Flash_Programming\FlashARM\MSPM0G1X0X_G3X0X_MAIN_128KB.FLM`

已验证 SWD 信息：

```text
VTref = 3.300V
Found SW-DP with ID 0x6BA02477
CPUID = 0x410CC601
```

## 9. 构建方法

使用 Keil V5.39 打开工程：

```text
C:\Users\zch20\Desktop\Myproject\Github仓库\nuedc_e_car_mspm0\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx
```

打开后执行 `Rebuild`。

当前已验证构建结果：

```text
0 Error(s), 0 Warning(s)
```

也可以使用 Keil 命令行构建，示例：

```bat
"D:\Keil_v5_38a\UV4\UV4.exe" -b "C:\Users\zch20\Desktop\Myproject\Github仓库\nuedc_e_car_mspm0\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx" -j0 -o "C:\Users\zch20\Desktop\Myproject\Github仓库\nuedc_e_car_mspm0\build_mspm0_port.log"
```

命令行构建不是必须流程，日常可直接使用 Keil 图形界面。

如果命令行构建因中文路径或编码问题失败，优先使用 Keil GUI 打开工程并执行 Rebuild。

## 10. 下载器注意事项

当前下载器识别为：

```text
J-Link ARM-OB STM32 / SAM-ICE
```

该下载器可通过 SWD 访问 MSPM0G3507。

如果 Keil/J-Link 无法连接目标芯片，需要检查 `JLinkSettings.ini` 是否错误残留：

```text
Device="ARM7"
```

正确配置应为：

```text
Device="Cortex-M0+"
```

本仓库当前只记录注意事项，不包含烧录或实车运行步骤。

## 11. 安全说明

当前工程默认不启动车轮，迁移阶段不要直接上车运行。

已做的默认安全处理：

1. `Motor_Init()` 后调用 `Motor_StopAll()`
2. 默认 `g_carEnable = 0`
3. 默认目标速度为 `0`
4. PWM compare 为 `0`
5. 方向引脚默认拉低
6. `main()` 不主动调用 `ECar_Start()`
7. 串口远程 start 默认禁用：`ECAR_ENABLE_REMOTE_START = 0`

后续上电测试建议先架空车轮，按 UART、按键、灰度、编码器、PWM 的顺序逐项验证。

## 12. 下一步计划

1. UART 回显
2. Key 按键测试
3. 灰度 `raw[8]` 打印
4. 编码器架空测试
5. PWM 架空电机测试
6. OLED I2C 真驱动
7. 最后再启用 `ECar_Start()` 低速循迹

## 13. Git 提交建议

建议提交：

- `App/`
- `Control/`
- `Hardware/`
- `System/`
- `User/`
- `keil/*.uvprojx`
- `*.syscfg`
- `ti_msp_dl_config.c`
- `ti_msp_dl_config.h`
- `README.md`
- `.gitignore`
- `migration_report_mspm0.md`

不建议提交：

- `Objects/`
- `Listings/`
- `DebugConfig/`
- `*.axf`
- `*.hex`
- `*.map`
- `*.o`
- `*.d`
- `JLinkLog.txt`
- `JLinkSettings.ini`
- `build_mspm0_port.log`
