# rr2

DM-MC02-H7 / STM32H723VGT6 firmware scaffold for the new R2 controller.

## CubeMX configuration

- MCU: `STM32H723VGTx` (`STM32H723VGT6`, LQFP100)
- Clock: 24 MHz HSE, SYSCLK 480 MHz
- Debug: SWD on `PA13/PA14`
- FDCAN1: `PD0` RX, `PD1` TX, classic CAN, 1 Mbps
- FDCAN2: `PB5` RX, `PB6` TX, classic CAN, 1 Mbps
- FDCAN3: `PD12` RX, `PD13` TX, classic CAN, 1 Mbps
- USART1: `PA9` TX, `PA10` RX, 921600 8N1
- Toolchain: CMake + GNU Arm Embedded

## Bus assignment

- `FDCAN1`: chassis, 4 x DJI 3508 mecanum wheel motors
- `FDCAN2`: lift, 4 x DJI 3508 lift motors
- `FDCAN3`: gripper / small weapon
  - Lingzu RS00 gripper lift
  - Lingzu RS00 gripper rotate
  - Lingzu RS05 gripper open/close
  - Lingzu RS05 end-effector rotate
- `USART1`: upper computer communication
- Servo: end-effector clamp servo still needs a PWM timer/pin decision before CubeMX can configure it.

## Build

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Open `rr2.ioc` in STM32CubeMX to adjust pins/peripherals, then regenerate with `generate.mxscript` if needed.

## 电机Can ID修改

现在代码不再假设底盘四个电机一定是 1,2,3,4。比如换车后四个底盘电机 ID 是 4,1,3,2，你只需要改 Modules/chassis.h:36：

  #define CHASSIS_MOTOR_CONFIG_INIT                       \
      {                                                   \
          {1U,  1, 6.0f, 1.0f, 0.0f}, /* RF 右前 */      \
          {4U, -1, 6.0f, 1.0f, 0.0f}, /* LF 左前 */      \
          {3U, -1, 6.0f, 1.0f, 0.0f}, /* LB 左后 */      \
          {2U,  1, 6.0f, 1.0f, 0.0f}, /* RB 右后 */      \
      }

  代码会根据 ID 自动决定发哪一帧：

  ID 1~4 -> 发 0x200
  ID 5~8 -> 发 0x1FF

  所以如果以后底盘 ID 不是 1,2,3,4，比如 5,6,7,8，也可以适应。只要还是 DJI C620 的 ID 范围 1~8，并且还在同一条 CAN 上，就不用改底盘控制逻辑，
  只改配置表。

  不能适应的情况是：

  1. 电机不是 C620 / DJI 3508 协议
  2. 电机 ID 超过 1~8
  3. 四个底盘电机不在 FDCAN1，而换到了 FDCAN2/FDCAN3
  4. 四个轮子数量或底盘类型变了，比如不是四麦轮

  这些情况才需要改底层发送或底盘运动学。

  ## 转动角度： float Lift_GetPositionDeg（） #lift.c
      g_dji_motors[i].total_angle_deg - s_zero_offset_deg[i]
       (电机原始累积角度，绝对累计值)         （i号lift电机启动测试时记录的零点）

  ## 反馈帧：
  front_lift/rear_lift :真实角度（Lift_GetPositionDeg（））读的是电机实际位置：g_dji_motors[i].total_angle_deg，只要电机在线就有值

  ## 传位置：#define LIFT_BOOT_TEST_DEG           3600.0f
  aa 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55
  aa 00 00 00 00 00 00 00 00 00 00 00 00 00 00 61 45 00 00 61 45 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55