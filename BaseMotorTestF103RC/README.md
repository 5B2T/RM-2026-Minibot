# F103RCT6 底座电机最小测试

独立工程，仅测试JGY370底座电机与AB编码器，不修改F407/遥控器程序。HSI内部时钟+PLL为64MHz，不依赖板载晶振。目标STM32F103RCT6：256KB Flash、48KB RAM，使用高密度`STM32F103xE`头文件/启动向量。没有CubeMX `.ioc` 或Keil工程，源码为CMake+ARM GCC；复用仓库`ArmController`内已有ST SDK与系统调用，不可只拷走本文件夹而丢掉依赖。

## 新版电机插座

依据工作区`jgy 电机接线.jpg`（2026-10-23更新）。图片为编码器圆板在左、白插座在右；按图从上到下：

| 位置 | 新图标识 | 本次连接 |
|---|---|---|
| 1 | M1：电机端子 | TB6612 A01/AO1 |
| 2 | GND：霍尔供电负 | 公共GND |
| 3 | C1：编码器A相 | F103 PA6 / TIM3_CH1 |
| 4 | C2：编码器B相 | F103 PA7 / TIM3_CH2 |
| 5 | VCC：霍尔供电正 | 3.3V |
| 6 | M2：电机端子 | TB6612 A02/AO2 |

**新版没有标线色，旧的红黑蓝绿黄白对应关系作废。** 背面看插座会镜像，按PCB标识/通断核对，不能凭线色或插头翻面后的方向推断。图中M1/M2是同一个底座电机的两个端子，不是底盘轮号；两根电机线均接驱动输出，不直接接MCU。

新图写明霍尔供电3.3V/5V、自带上拉整形，本测试统一用3.3V以匹配MCU输入。两个信号按AB四倍频计数；本例只报计数，不套用旧减速比/6600计数每圈，不宣称角度/转速已标定。

## TB6612与RCT6

| TB6612 | 接到 |
|---|---|
| PWMA | PB6 / TIM4_CH1，2kHz（原PA8已释放） |
| AIN1 | PB0 |
| AIN2 | PB1 |
| STBY | PB10，程序控制，不另外硬接高 |
| AO1/AO2 | 电机M1/M2 |
| VCC | 3.3V逻辑电源 |
| VM | 现有12V电机电源正极（按实物12V电机） |
| GND | MCU GND、编码器GND、电机电源负极共地 |

B通道不使用；未用的PWMB/BIN1/BIN2可接GND，BO1/BO2悬空。RCT6板本身按其规定的USB/5V/3.3V入口供电，**12V只接驱动VM，不能接MCU的5V/3.3V**。先断电接线，机械臂固定并留出旋转空间；不要继续套F407的PF8/PD5/PD6/PD7接法。

可选串口：适配器TX→PA10、RX←PA9、GND共地，3.3V TTL、115200 8N1、无流控。SWD：SWDIO→PA13、SWCLK→PA14、GND共地，VTref接目标3.3V；BOOT0保持0。串口TX不要接NRST。以下引脚均按芯片名，具体板上排针位置以板卡丝印为准。

官方核对：[ST STM32F103RC数据手册](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)，Table 2与Table 5。

## 测试方法

上电PWM=0、STBY=0，**不会自己转**。串口工具选普通文本/ASCII，每次发一个字符，大小写均可、回车换行忽略，不用原ARMDBG命令或FireWater：

| 字符 | 动作 |
|---|---|
| `F` | 约30%正向点动1秒，自动停 |
| `R` | 约30%反向点动1秒，自动停 |
| `T` | 正向1秒→停0.5秒→反向1秒→停止，仅一次 |
| `S` | 停止并取消测试 |

每个新运动命令先关闭输出10ms，避免直接换向；单次到期停止，**不要开启自动循环发送F/R/T**，连续重发会重新计时。正向只是AIN1高/AIN2低的电气定义，实际机械转向和计数正负由接线决定。

没有串口也可用Ozone/J-Link Watch修改`g_test_command`：1正向、2反向、3一轮测试、4停止；主循环消费后回到0。观察`g_ready`（应为1）、`g_error`（应为0）、`g_pwm`、`g_encoder_count`、`g_encoder_delta`、`g_encoder_ab`。编码器10ms采样；ab的bit0=C1、bit1=C2，低速显示只是瞬时电平，不是跳变计数。

串口每200ms输出一行`ms=... pwm=... enc=... d=... ab=... mode=... err=...`。输出为状态文本，`enc`为有符号累计计数，`d`为最近一次10ms计数差。`g_error` 1/2时钟、3～5 PWM、6/7编码器、8 UART初始化失败，90～93 CPU故障。累计计数溢出时饱和；无闭环、无自动寻零、无编码器断线检测。

建议先F确认转动与计数，再R确认相反方向，最后T；若会转但计数一直为0，先查新图第2/5脚供电及第3/4脚C1/C2。软件定时依赖CPU运行，先发S再打断点，不能以暂停调试器代替停机。

## 构建与烧录

将`arm-none-eabi-gcc`和Ninja加入PATH，在本目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

输出`build/Debug/BaseMotorTestF103RC.elf/.hex/.bin`（Release同理）。J-Link/Ozone目标选择`STM32F103RC`，SWD；HEX自带地址，BIN写入0x08000000。不要给F407或遥控C8T6烧录这个固件。

主机逻辑测试（使用电脑本机GCC，不是ARM GCC）：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -I . motor_test.c tests/test_motor_test.c -o build/test_motor_test.exe
./build/test_motor_test.exe
```

覆盖启动停止、正反/单循环各阶段边界、立即停止、换向间隔、命令重发、计时回绕和编码器回绕。Debug/Release编译及上述软件逻辑验证通过。2026-10-23因PA8被板卡占用，PWM改PB6/TIM4_CH1，已烧录F103RC并校验五个加载段，读回64MHz、ready=1、error=0、PWM=0及STBY低；未启动电机，接线和真实编码器计数待运动实测。
