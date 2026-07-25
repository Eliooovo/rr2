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

The logical wheel order is `RF, LF, LB, RB`. The current vehicle is numbered
clockwise from the left-front wheel as IDs `1, 2, 3, 4`, so the table maps
`RF=2, LF=1, LB=4, RB=3`.

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

Motors 1/2 receive the front target and are not reversed; motors 3/4 receive
the rear target and are currently reversed.
There is no App-level all-online or pair-online gate. An offline motor is
independently forced to zero current by `dji_motor`, while other online motors
continue controlling. `LIFT_APP_METERS_PER_OUTPUT_RAD` defines the output-side
mechanical conversion as `0.01242 m/rad`, and
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

## Build

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

Open `rr2.ioc` in STM32CubeMX 6.17.0 when hardware configuration changes, then
regenerate using the configured STM32Cube H7 V1.11.2 package.
