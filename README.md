# rr2

DM-MC02-H7 / STM32H723VGT6 firmware for the R2 controller.

## Hardware configuration

- MCU: `STM32H723VGT6`
- Clock: 24 MHz HSE, SYSCLK 480 MHz
- FDCAN1: chassis, 4 x DJI M3508/C620, classic CAN 1 Mbps
- FDCAN2: lift, 4 x DJI M3508/C620, classic CAN 1 Mbps
- FDCAN3: RobStride gripper/end-effector motors, classic CAN 1 Mbps
- USB HS CDC: upper-computer command and feedback link
- USART1: 921600 8N1, initialized by CubeMX but not used by `CommApp`

Hardware/peripheral settings are owned by `rr2.ioc`. No CubeMX peripheral
configuration is duplicated in the Apps.

## App architecture

The chassis, lift, and communication functions are three independent Apps:

- `App/chassis_app.*`: owns the FDCAN1 DJI motors/groups, mecanum kinematics,
  speed control, and actual chassis-velocity feedback.
- `App/lift_app.*`: owns the FDCAN2 DJI motors/groups, continuous multi-turn
  position control, and actual position feedback.
- `App/comm_app.*`: owns USB CDC frame parsing/sending and the two global
  mailboxes.

`g_comm_app_command` is written only by `CommApp` and read by the other Apps.
`g_comm_app_feedback` is written by `ChassisApp`/`LiftApp` and read by
`CommApp`; communication code no longer calls chassis or lift control APIs.

Minimal integration:

```c
ChassisApp_Init();
LiftApp_Init();
bsp_can_init();       /* Start CAN only after all DJI groups are registered. */
CommApp_Init();

while (1) {
    CommApp_RunPeriodic();
    ChassisApp_RunPeriodic();
    LiftApp_RunPeriodic();
}
```

The USB CDC generated file still includes `Modules/comm_protocol.h`. That file
is now only a compatibility forwarder from `Comm_OnUsbReceived()` to
`CommApp_OnUsbReceived()`; no generated USB file needs to be edited.

## Command and feedback protocol

Each frame is fixed at 58 bytes:

```text
0xAA + 14 x IEEE-754 float32 little-endian + 0x55
```

Field order:

1. chassis `vx` in m/s
2. chassis `vy` in m/s
3. chassis `wz` in rad/s
4. front lift linear position in m
5. rear lift linear position in m
6. kfs lift linear position in m
7. kfs root rotation in rad
8. kfs tip rotation in rad
9. kfs grip position in m
10. weapon rotation in rad
11. weapon grip position in m
12. chassis odometry X in m
13. chassis odometry Y in m
14. chassis odometry yaw in rad (continuous, unwrapped)

The feedback frame uses the same order. Chassis velocity fields are calculated
from all four actual motor speeds; lift fields are the actual average
continuous positions of the front and rear motor pairs. If a subsystem is not
ready or has an offline motor, that subsystem's feedback fields are sent as
zero.

The odometry pose (fields 12-14) is integrated from per-tick deltas of each
wheel's continuous multi-turn encoder count, converted to wheel displacement
in meters and passed through the mecanum forward kinematics (position
difference method, no time dependence). The pose is anchored at power-on
(X=0, Y=0, yaw=0) and is frozen while the chassis is offline.

里程计清零与复位：X/Y/yaw 仅在开机 `ChassisApp_Init` 时清零，运行期间无
复位机制；电机离线时内部位姿冻结不清零，但帧内字段随 `chassis_valid=0`
发 0，恢复后从冻结值继续积分。

This is a breaking protocol change: command frames must also carry 14 floats
(fields 12-14 are currently ignored by the firmware, send zeros). Host PC
software must be updated to the 58-byte frame, otherwise command frames no
longer parse.

The latest valid command is retained indefinitely; there is currently no
command timeout. A lift command received before a motor becomes online is
retained. Each motor starts controlling independently after its own first
feedback establishes that motor's software zero.

## 武器夹爪舵机位置与校准

武器夹爪使用 UART7 上 ID=6 的 STS3215 舵机。上位机位置命令和反馈的
单位均为 m，`WeaponGripApp` 按下式与舵机原始位置互换：

```text
raw = 3072 - position_m * 34133.33
position_m = (3072 - raw) / 34133.33
```

| 夹爪机械位置 | 上位机位置 | 舵机 raw |
|---|---:|---:|
| 闭合（夹爪逻辑零位） | 0 m | 3072 |
| 半开 | 约 0.015 m | 约 2560 |
| 全开（舵机电子中位） | 0.03 m | 2048 |

因此，舵机的电子中位 `raw=2048` 对应夹爪全开；夹爪命令的逻辑零位
`0 m` 对应闭合 `raw=3072`，两个“零位”不是同一个基准。

重新安装舵盘或夹爪后，校准步骤如下：

1. 失能舵机，确保装配时不会突然运动。
2. 把夹爪放在机械全开位。
3. 通过舵机调试工具执行“中位校准”，使该位置反馈 `raw≈2048`；不要把
   夹爪闭合位校为舵机中位或 `raw=0`。
4. 安装舵盘和连杆后，先下发 `0.03 m` 验证全开位基本不动，再逐步减小
   命令至 `0 m`，检查闭合位且确认机构没有顶死。

固件启动时只使能舵机，在收到第一个有效上位机命令前不下发位置目标。
中位校准不会在启动时自动执行，需要使用舵机调试工具单独完成。

## Chassis configuration

User-adjustable chassis settings are in `App/chassis_app.h`:

- `CHASSIS_APP_MOTOR_CONFIG_INIT`: motor ID, installation direction, and
  per-motor speed PID.
- `CHASSIS_APP_VX_DIRECTION` / `CHASSIS_APP_VY_DIRECTION`: upper-computer
  coordinate reversal, each set to `1.0f` or `-1.0f`.
- `CHASSIS_APP_VX_SCALE` / `VY_SCALE` / `WZ_SCALE`: execution multipliers.
- `CHASSIS_APP_ODOMETRY_X_SCALE` / `Y_SCALE` / `YAW_SCALE`: pose odometry
  calibration multipliers applied to the integrated displacements.
- Wheel dimensions, chassis dimensions, reduction ratio, current limit,
  offline timeout, and control period.

The logical wheel order is `RF, LF, LB, RB`. The current vehicle mapping is
`LF=1, LB=2, RB=3, RF=4`.

The inverse kinematics are:

```text
RF = vx - vy - R*wz
LF = vx + vy + R*wz
LB = vx - vy + R*wz
RB = vx + vy - R*wz
R  = chassis_half_length + chassis_half_width
```

Actual feedback uses the exact inverse matrix and reverses each motor's
installation direction before calculation. Coordinate direction is converted
back to the upper-computer convention. Scale is not divided out, so feedback
reports actual executed speed.

## Lift configuration

User-adjustable lift settings are in `App/lift_app.h`:

- `LIFT_APP_MOTOR_CONFIG_INIT`: motor ID, per-motor direction (`1` normal,
  `-1` reversed), position/speed PID, maximum speed, and current limit.
- Control period, offline timeout, and PID integral limits.
- `LIFT_APP_SYNC_KP_RPM_PER_DEG`, `LIFT_APP_SYNC_KD_RPM_S_PER_DEG`, and
  `LIFT_APP_SYNC_MAX_CORRECTION_RPM`: front/rear pair synchronization gains
  and per-motor correction limit.
- `LIFT_APP_FOUR_SYNC_KP_RPM_PER_DEG`,
  `LIFT_APP_FOUR_SYNC_KD_RPM_S_PER_DEG`, and
  `LIFT_APP_FOUR_SYNC_MAX_CORRECTION_RPM`: stronger front-to-rear average
  position synchronization gains and four-motor final correction limit.

The lift uses the same physical mapping as the chassis:
`LF=1, LB=2, RB=3, RF=4`. Motors 1/4 receive the front target and motors 2/3
receive the rear target. LF/RB are currently reversed; RF/LB are not reversed.
All four lift motors currently use an `1800 rpm` position-loop speed limit.
Each pair uses its normalized motor-shaft position difference to apply equal
and opposite speed corrections. The initial synchronization controller is
P-only (`1.0 rpm/deg`) with a per-motor correction limit of `100 rpm`; the
configurable derivative gain initially remains zero. When the stored front and
rear targets are exactly equal and all four feedback values are valid, an
additional controller synchronizes the front-pair and rear-pair average
positions with `12.0 rpm/deg`, zero derivative gain, and a `200 rpm` final
per-motor limit. Pair and four-motor corrections are combined and uniformly
scaled when necessary without changing the zero average of the four
synchronization corrections.
There is no App-level all-online or pair-online gate. An offline motor is
independently forced to zero current by `dji_motor`, while other online motors
continue controlling, but synchronization correction for that motor pair is
cleared whenever either feedback is unavailable. Four-motor synchronization
falls back to the valid pair controllers if any one of the four feedback values
is unavailable or if the front and rear targets differ.
`LIFT_APP_METERS_PER_OUTPUT_RAD` defines the output-side mechanical conversion
as `0.01242 m/rad`, and
`LIFT_APP_MOTOR_REDUCTION_RATIO` is `19.0`. Therefore one motor-shaft radian
corresponds to `0.01242 / 19 m`. Commands and feedback are converted internally
with `double`; the USB protocol remains `float32`.

## RS 电机 PID 参数

FDCAN3 上挂载的 RobStride RS 系列电机，通过 `rs_motor_write_parameter()` 可在线读写
内部 PID 寄存器。以下列出三款在用电机型号的全部 PID 相关初始值。

### 电机型号速查

| 型号 | TYPE | 关节 | motor_id | 速度范围 rad/s | 扭矩范围 Nm | MIT kp 范围 | MIT kd 范围 |
|------|------|------|----------|---------------|------------|------------|------------|
| RS00 | 0 | KFS 末端旋转 | 3 | -33 ~ 33 | -14 ~ 14 | 0 ~ 500 | 0 ~ 5 |
| RS03 | 3 | KFS 根部旋转 | 2 | -20 ~ 20 | -60 ~ 60 | 0 ~ 5000 | 0 ~ 100 |
| RS05 | 5 | 武器旋转 / 夹爪 | 4, 5 | -50 ~ 50 | -5.5 ~ 5.5 | 0 ~ 500 | 0 ~ 5 |

### 出厂默认 PID 值（0x70XX 参数组）

以下参数通过通信类型 0x12（参数写入）读写，所有 float 均为 IEEE 754 小端。

|   地址      |      名称      |  类型 | RS00 默认| RS03 默认| RS05 默认|   说明   |
|------------|---------------|-------|---------|---------|--------|------|
| 0x7010     | cur_kp        | float |  0.17   |  0.17   | 0.17   | 电流环比例增益 |
| 0x7011     | cur_ki        | float |  0.012  |  0.012  | 0.012  | 电流环积分增益 |
| 0x7014     | cur_filt_gain | float |  —      |  —      | —      | 电流滤波系数 |
| **0x701E** |  **loc_kp**   | float |  **40** |  **60** | **40** | **位置环比例增益 (CSP 核心)** |
| 0x701F     | spd_kp        | float |  6      |  6      | 6      | 速度环比例增益 |
| 0x7020     | spd_ki        | float |  0.02   |  0.02   | 0.02   | 速度环积分增益 |
| 0x7021     | spd_filt_gain | float |  0.1    |  0.1    | 0.1    | 速度滤波系数 |
| 0x702A     | damper        | uint8 |  0      |  0      | 0      | 阻尼开关，0=关 1=开 |

### 出厂默认 PID 值（0x20XX 参数组）

旧版参数库，与 0x70XX 功能重叠。当前代码统一使用 0x70XX 组，此表仅供对照。

| 地址 | 名称 | 类型 | RS00 默认 | RS03 默认 | RS05 默认 | 说明 |
|------|------|------|----------|----------|----------|------|
| 0x2011 | cur_filt_gain | float | 0.9 | 0.9 | 0.9 | 电流滤波系数 |
| 0x2012 | cur_kp | float | 0.025 | 0.025 | 0.025 | 电流环 Kp |
| 0x2013 | cur_ki | float | 0.0258 | 0.0258 | 0.0258 | 电流环 Ki |
| 0x2014 | spd_kp | float | 2 | 2 | 2 | 速度环 Kp |
| 0x2015 | spd_ki | float | 0.021 | 0.021 | 0.021 | 速度环 Ki |
| 0x2016 | loc_kp | float | 30 | 30 | 30 | 位置环 Kp |
| 0x2017 | spd_filt_gain | float | 0.1 | 0.1 | 0.1 | 速度滤波系数 |
| 0x2026 | damper | uint8 | 0 | 0 | 0 | 阻尼开关 |

### 控制架构

```
上位机目标角度 (loc_ref, 0x7016)
        │
        ▼
  位置环 (loc_kp, 0x701E)     ← 误差 → 速度指令
        │
        ▼
  速度环 (spd_kp 0x701F       ← 误差 → 电流指令
         + spd_ki 0x7020)
        │
        ▼
  电流环 (cur_kp 0x7010       ← 误差 → PWM 输出
         + cur_ki 0x7011)
        │
        ▼
      电机
```

CSP 模式（run_mode=5）下，MCU 只需下发 `loc_ref` 和 `limit_spd`，上述三级 PID
全部在电机内部运行。

### 常见症状与调参方向

| 症状 | 原因 | 调法 |
|------|------|------|
| 能推动但回弹（刚度不够） | loc_kp 太小 | ↑ loc_kp，每次 +20~30 |
| 到位后来回振荡 | loc_kp 过大或 spd_kp 过大 | ↓ loc_kp，↓ spd_kp |
| 恒定外力下偏位 | spd_ki 太小（静差） | ↑ spd_ki |
| 到位后停不稳、抖动 | spd_kp 或 spd_ki 过大 | ↓ spd_kp，↓ spd_ki |
| 到位慢、反应迟钝 | loc_kp 太小 或 limit_spd 太小 | ↑ loc_kp，↑ limit_spd |

### 当前代码中的 PID 覆盖值

`App/kfs_rotate_app.c` 中两个旋转关节初始化后写入了以下值，覆盖出厂默认。

**RS03 根部（2x 出厂）：**

```c
rs_motor_write_parameter(&s_root.motor, RS_PARAM_LOC_KP,  120.0f);  // 默认 60
rs_motor_write_parameter(&s_root.motor, RS_PARAM_SPD_KP,  12.0f);   // 默认 6
rs_motor_write_parameter(&s_root.motor, RS_PARAM_SPD_KI,  0.05f);   // 默认 0.02
```

**RS00 末端（2.5x 出厂）：**

```c
rs_motor_write_parameter(&s_tip.motor, RS_PARAM_LOC_KP,  100.0f);  // 默认 40
rs_motor_write_parameter(&s_tip.motor, RS_PARAM_SPD_KP,  15.0f);   // 默认 6
rs_motor_write_parameter(&s_tip.motor, RS_PARAM_SPD_KI,  0.08f);   // 默认 0.02
```

> **kp/kd 匹配原则**：增大 loc_kp 时必须同比增大 spd_kp，保持出厂比例。
> RS03 比例 = 60/6 = 10:1，RS00 比例 = 40/6 ≈ 6.67:1。
> 偏离出厂比例会导致欠阻尼振荡（spd_kp 相对太小）或过阻尼迟钝（spd_kp 相对太大）。

武器旋转（RS05）当前使用出厂默认值，如需覆盖参照同样写法。

### 运行时调参

`rs_motor_write_parameter()` 可在电机在线时热调，无需先停机关闭：

```c
#include "rs_motor.h"

/* 提高 RS00 位置保持刚度 */
rs_motor_write_parameter(&s_tip.motor, RS_PARAM_LOC_KP, 120.0f);

/* 读取当前反馈确认效果 */
rs_motor_feedback_t fb;
rs_motor_get_feedback(&s_tip.motor, &fb);
/* fb.angle_rad  — 当前位置 */
/* fb.speed_rad_s — 当前速度 */
/* fb.torque_nm   — 当前扭矩 */
```

> **注意**：`RS_PARAM_DAMPER` 为 uint8 类型，当前 `rs_motor_write_parameter()` 只支持
> float 参数，写入 damper 需用其他方式（或先用厂商上位机设定）。其余七个
> `RS_PARAM_*` 均为 float，可直接使用。

## DJI motor objects and groups

Each C620 is represented by one caller-allocated `dji_motor_t`. A
caller-allocated `dji_motor_group_t` contains at most four motors sharing one
CAN command frame:

- command ID `0x200`: motor IDs 1 through 4
- command ID `0x1FF`: motor IDs 5 through 8

Unused current slots are automatically sent as zero. FDCAN receive callbacks
are centralized in the BSP and dispatch feedback to registered groups.

```c
static dji_motor_t motor = {
    .config = {
        .motor_id = 1U,
        .offline_timeout_ms = 100U,
        .control_period_ms = 1U,
        .current_limit = 5000.0f,
        .max_speed_rpm = 300.0f,
        .speed_pid = {6.0f, 0.5f, 0.0f, 30000.0f},
        .position_pid = {6.5f, 0.0f, 0.0f, 0.0f},
    },
};
static dji_motor_group_t group = {
    .config = {
        .hfdcan = &hfdcan2,
        .command_id = DJI_MOTOR_CMD_ID_1_TO_4,
        .motor_count = 1U,
        .motors = {&motor},
    },
};

(void)dji_motor_init(&motor);
(void)dji_motor_group_init(&group);
(void)dji_motor_position_control(&motor, 7200.0);
(void)dji_motor_group_update(&group, HAL_GetTick());
```

Use `dji_motor_get_feedback()` for an interrupt-safe snapshot of the actual
single-turn fields and continuous multi-turn position.

## 遥控器操作 (FS-i6 / IBUS)

通过 FS-i6 遥控器 + IA6B/IA10B 接收机，可以在没有上位机的情况下直接控制
底盘和升降。

### 硬件接线

接收机侧面 B/VCC 口（3 针竖排）：

```
USART10 口（4pin） →  FS-i6接收机 B/VCC 口（侧面竖排 3 针）
────────────────────────────────────────────────────
VCC  (5V)          →  VCC  (中间)
GND                →  GND  (最下)
RX   (PE2)         →  SENS (最上, IBUS 数据)
TX   (PE3)         →  不接
```

> USART10 口的 VCC 由 PC15 使能（可控 5V 电源），上电后 `MX_GPIO_Init()` 自动拉高。

接收机需要和遥控器**对码**后才能使用：接收机按住 BIND 键上电 → 红灯快闪 →
遥控器进对码模式 → 红灯常亮即成功。

### 通道映射

| 摇杆 / 开关 | 通道 | 控制对象 | 说明 |
|------------|------|---------|------|
| 右摇杆 上下 | CH2 | 底盘 vx | 推上 = 前进，推下 = 后退，最大 ±0.5 m/s |
| 右摇杆 左右 | CH1 | 底盘 vy | 推右 = 右移，推左 = 左移，最大 ±0.5 m/s |
| 左摇杆 左右 | CH4 | 底盘 wz | 推右 = 逆时针旋转，推左 = 顺时针旋转，最大 ±1.0 rad/s |
| 三档拨杆 SWC | CH5 | 升降 | 往上 = 平台上升，回中 = 停止，往下 = 平台下降 |

升降速度 0.05 m/s，行程限制 0 ~ 0.30 m。左摇杆上下（CH3 / 油门）和两档开关
SWA（CH6）暂未使用。

> **升降方向反了？** 改 `App/rc_control.c` 里的 `RC_LIFT_DIRECTION` 为 `-1.0f`，
> 或者在 FS-i6 遥控器菜单 `Functions → Reverse → CH5` 设为 Rev。

### 遥控 vs 上位机优先级

遥控有效时，上位机（USB CDC）的底盘和升降指令被忽略，但上位机仍能接收
反馈数据（可正常监控状态）。

遥控信号丢失超过 **100 ms** 后：
- 底盘速度清零
- 升降保持当前位置
- 上位机控制自动恢复

### 调试

Ozone 中观测以下变量确认链路正常：

| 变量 | 含义 | 正常值 |
|------|------|--------|
| `s_valid_frame_count` | 有效帧计数 | 持续递增 |
| `s_channels[0]` | CH1 原始值 | 1000~2000，中位 ~1500 |
| `s_vx_m_s` | 归一化 vx | 推右摇杆上下时 ±1.5 m/s |
| `s_lift_position_m` | 累积升降位置 | CH5 拨杆往上时递增 |
| `g_comm_app_command.valid` | RC 控制有效 | 1 |

### TOF200C 测距调试

Ozone 中观测以下变量确认 TOF 传感器工作正常：

**距离数据：**

| 变量 | 含义 | 正常值 |
|------|------|--------|
| `g_tof200c_latest.distance_mm` | 距离（毫米） | 30~2000，随目标变化 |
| `g_tof200c_latest.range_valid` | 数据有效 | `true` (1) |
| `g_tof200c_latest.range_status` | 测距状态码 | `0` = 正常 |
| `g_tof200c_latest.sample_count` | 采样计数 | 持续递增（约 5 Hz） |
| `g_tof200c_latest.timestamp_ms` | 最后一次采样时刻 | 周期性更新 |

**传感器状态：**

| 变量 | 含义 | 在线时 |
|------|------|--------|
| `g_tof200c.state.connection` | 连接状态 | `2` = ONLINE |
| `g_tof200c.internal.recovery_backoff_ms` | 退避间隔 | `0` = 正常，非 0 = 离线重试中 |
| `g_tof200c.fault.offline_count` | 累计离线次数 | 不变 = 稳定 |
| `g_tof200c.fault.recovery_count` | 累计恢复次数 | 热拔插后递增 |
| `g_tof200c_read_status` | 最近一次读取状态 | `0` = OK，`4` = STALE_DATA |

**快速检测：** 把 `g_tof200c_latest.distance_mm` 加到 Watch 窗口，用手或物体在传感器前移动，数值应跟随变化。如果始终为 0 且 `range_valid = false`，检查传感器接线和 I2C2 通信。

## Build

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

Open `rr2.ioc` in STM32CubeMX 6.17.0 when hardware configuration changes, then
regenerate using the configured STM32Cube H7 V1.11.2 package.

deg ≈ rad × 57.3

+X 轴（正前）：车头正前方。前进时 X 增加。
+Y 轴（正左）：车身正左方。向左横移（平移）时 Y 增加。
+Yaw（正向旋转）：逆时针旋转（即原地向左转）。左转时 Yaw 增加。
