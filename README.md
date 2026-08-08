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

Each frame is fixed at 46 bytes:

```text
0xAA + 11 x IEEE-754 float32 little-endian + 0x55
```

Field order:

1. chassis `vx` in m/s
2. chassis `vy` in m/s
3. chassis `wz` in rad/s
4. front lift linear position in m
5. rear lift linear position in m
6. through 11. reserved

The feedback frame uses the same order. Chassis fields are calculated from all
four actual motor speeds; lift fields are the actual average continuous
positions of the front and rear motor pairs. If a subsystem is not ready or
has an offline motor, that subsystem's feedback fields are sent as zero.

The latest valid command is retained indefinitely; there is currently no
command timeout. A lift command received before a motor becomes online is
retained. Each motor starts controlling independently after its own first
feedback establishes that motor's software zero.

## Chassis configuration

User-adjustable chassis settings are in `App/chassis_app.h`:

- `CHASSIS_APP_MOTOR_CONFIG_INIT`: motor ID, installation direction, and
  per-motor speed PID.
- `CHASSIS_APP_VX_DIRECTION` / `CHASSIS_APP_VY_DIRECTION`: upper-computer
  coordinate reversal, each set to `1.0f` or `-1.0f`.
- `CHASSIS_APP_VX_SCALE` / `VY_SCALE` / `WZ_SCALE`: execution multipliers.
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
