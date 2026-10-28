# 机械臂电位器映射与 PWM 控制（2026-10-25历史方案）

**以下保留远端历史设计记录，不作为当前实现或烧录依据。当前配置见[主控侧机械臂目标角接入说明](主控侧机械臂目标角接入说明.md)。**

当前五路舵机均为位置模式，行程300/300/270/180/180°，不使用旧肩部150°偏移或连续舵机计时逻辑；夹爪6/45°。电位器有效ADC为1～4094，J1新锚点1354→95°、2425→5°，新参数待更新中继。底座目标±180°且不作360°回绕，不能将历史视觉等价角取模后直接发给受机械限位约束的舵机。自动序列失联继续完成、手动/POT失联停止，细则以当前接入说明为准。

ArmViewer中的0～333°及Sign/Span/Zero是历史显示坐标与绘图调节，不等于当前舵机指令角；中继串口ANG才是当前标定后的角度。此处历史参数不得覆盖现有固件标定。

## 历史正文

范围：五路电位器 ADC → 关节角 → 舵机脉宽／底座 PWM。
与代码冲突时**以代码为准**，锚点见 §9。

---

## 2. 不变量（违反即错）

- ADC 端点用**实测行程端点**，不用 0/4095；当前统一取 `200 / 3900`。
- 任一通道 `ADC==0` 或超出 `[AdcMin,AdcMax]` → 该关节**保持不动**，不得按 0° 处理。
- `Servo_Pulse()` 在脉宽或角度越界时**返回 0（输出 0，不是钳位）**，上层必须保证不越界。
- `Dir<0` 时 **ADC 最小端对应 `DegMax`**（`MapAngle` 把分子取反）。
- 角度有 360° 周期等价；下发舵机前必须归一化到 `[0, MaxAngleDeg]`。
- 底座编码器是**相对量**，上电位置即零点，目标角先减 `θ_encoder(power-on)`。
- 软限位在 `MapAngle` 内部钳位，越界不报错。

---

## 3. 换算链（五级线性 + 钳位）

```text
① r = clamp((ADC - AdcMin) / (AdcMax - AdcMin), 0, 1);  若 Dir<0: r = 1 - r
② θ = θ_min + r * (θ_max - θ_min)                        # 等价 θ = Sign*(Zero + Span*r)
③ θ* = clamp(θ, SoftMin, SoftMax);  |Δθ*| <= RateMax*Δt  # 芯片2: 500cdeg/20ms = 250°/s
④ 舵机: ratio = θ_servo/MaxAngleDeg; Dir<0 → ratio = 1-ratio
        pulse_us = UsMin + ratio*(UsMax - UsMin)         # 500..2500
   肩: θ_servo = 150 + θ_shoulder                        # ROBOT_ARM_SHOULDER_ZERO_DEG
   底座: θ_enc = counts*360/6600; BasePWM = clamp(Kp*e+Ki*∫e, -1, 1) * 300
⑤ BSP_Arm_Apply(PulseUs[], BasePWM)；STBY 仅 BasePWM!=0 时拉高
```

示例（肩 90°）：`θ_servo=240 → ratio=0.8 → Dir=-1 → 0.2 → pulse=500+0.2*2000=900µs`。

---

## 4. 编号映射（易错）

| 电位器 | 引脚 | `Arm_Joint` | 类型 |
|---|---|---|---|
| J1 BASE | PA0 | `ARM_BASE`=5 | 直流+编码器 6600cpr |
| J2 SHOULDER | PA1 | `ARM_SHOULDER`=0 | 位置舵机 |
| J3 ELBOW | PA2 | `ARM_ELBOW`=1 | 位置舵机 |
| J4 WRISTP | PA3 | `ARM_WRIST_PITCH`=2 | 位置舵机 |
| J5 WRISTR | PA4 | `ARM_WRIST_ROLL`=3 | 位置舵机 |
| — | — | `ARM_GRIPPER`=4 | 按键两档 0°/45° |

枚举顺序（肩,肘,腕俯仰,腕旋转,夹爪,底座）**≠** 电位器通道顺序（底座,肩,肘,腕俯仰,腕旋转）。
帧 `Reserved[5..14]` 用**电位器顺序**，解出后按上表重排。

---

## 5. 实测参数（2026-10-25，视觉拟合）

采集时串口 `STALE`、五路 ADC 读到 0；数值为最后一次调定值，接回供电后需复验。

| 通道 | span | zero | sign | r=0 角度 | r=1 角度 |
|---|---|---|---|---|---|
| 底座 | 340 | -170 | -1 | +170 | -170 |
| 肩 | 335 | 193 | -1 | -193 | -528 |
| 肘 | 315 | -163 | +1 | -163 | +152 |
| 腕俯仰 | 300 | -155 | -1 | +155 | -145 |
| 腕旋转 | 360 | -180 | +1 | -180 | +180（未标定） |

换算到固件（`Dir = sign`）：

```text
sign=+1: DegMin = zero        , DegMax = zero + span
sign=-1: DegMin = -(zero+span), DegMax = -zero
```

```c
// ArmController/UserApp/arm_targets.c DefaultCal[]，顺序=电位器顺序，单位 0.01°
{ 200, 3900, -17000, +17000, -1, <softmin>, <softmax> },  // 底座
{ 200, 3900, -52800, -19300, -1, <softmin>, <softmax> },  // 肩
{ 200, 3900, -16300, +15200, +1, <softmin>, <softmax> },  // 肘
{ 200, 3900, -14500, +15500, -1, <softmin>, <softmax> },  // 腕俯仰
{ 200, 3900, -18000, +18000, +1, <softmin>, <softmax> },  // 腕旋转
```

软限位先按舵机可达范围填，装配后收紧到机械极限 ±5°。

**两个已知冲突**：

1. 肩 335°、肘 315° 的拟合行程 > 常见舵机 300° 行程 → 必须压 `Span` 或改增量式（§7）。
2. 拟合角是视觉等价角（如肩 -528 ≡ +192），舵机只能 0..`MaxAngleDeg`，下发前取模归一化。

---

## 6. 执行器配置与不一致

`UserApp/robot/robot_config.h:63-68`：

| 关节 | 模式 | UsMin/Neu/Max | MaxAngleDeg | Dir |
|---|---|---|---|---|
| 肩 | POSITION | 500/1500/2500 | 300 | -1 |
| 肘 | **CONTINUOUS** | 500/1500/2500 | 0 | -1 |
| 腕俯仰 | **CONTINUOUS** | 500/1500/2500 | 0 | +1 |
| 腕旋转 | POSITION | 500/1500/2500 | 180 | +1 |
| 夹爪 | POSITION | 500/1500/2500 | 180 | +1 |

实机 5 个关节**全部是位置舵机**，但肘/腕俯仰仍配为 `CONTINUOUS` → ④ 走速率分支，角度到不了。改为：

```c
{ SERVO_POSITION, 500, 1500, 2500, <实测行程>, <方向> },
```

底座实测：归一化 0.25/0.5/0.75/1.0 → 1.5/3.7/6.2/9.0 rpm，可作前馈。

---

## 7. 行程不足时的增量式

```text
θ*(k) = clamp(θ*(k-1) + K*(r(k) - r(k-1)), SoftMin, SoftMax)
```

有漂移，必须配复位（可复用 `R_LEFT+R_RIGHT` 退出 POT 模式的组合）。

---

## 8. 失效处理（全部必须实现）

| 场景 | 动作 |
|---|---|
| 100 ms 无有效帧 | 清目标、输出归零，机械臂保持最后位置 |
| STOP 帧 | 锁存；RESUME 需四个速度先归零 |
| 通道 ADC 无效 | 该关节保持不动 |
| 越软限位 | 钳位 |
| 舵机脉宽越界 | 上游保证不越界（越界即输出 0） |
| 上电 | 底座取当前为零点；舵机先回 `PRESET` 再跟随 |

验收：单关节慢扫偏差 ≤3° 无跳变；端点停软限位；断链 100 ms 内停；重上电不甩；`-Replay` 回放与实机逐帧一致。

---

## 9. 代码锚点

```text
Modules/actuator/servo.c:11-17       舵机脉宽公式与越界返回 0
UserApp/components/arm/arm.c:29-33   PositionPulse / 肩零点
UserApp/components/arm/arm.c:256     底座 BasePWM
UserApp/components/arm/arm.c:312     底座编码器角度
UserApp/components/arm/arm_def.h:5   关节枚举顺序
UserApp/robot/robot_config.h:63-68   ROBOT_ARM_SERVO_CONFIG
UserApp/robot/robot_config.h:71-95   肩零点/底座 PWM 限幅/CPR
ArmController/UserApp/arm_targets.c  DefaultCal / MapAngle / 限速 / 软限位
tools/ArmViewer.ps1  $Cal, $Draw     显示侧同一套参数（Sign/Span/Zero ↔ Dir/DegMin/DegMax）
```

---

## 10. 待办

- [ ] 五路 ADC 端点分别实测（现统一 200/3900）
- [ ] J5 腕旋转标定
- [ ] 肘/腕俯仰改 `SERVO_POSITION` 并填 `MaxAngleDeg`
- [ ] 肩/肘 `Span` 压进舵机行程或改增量式
- [ ] 软限位按机械极限收紧
