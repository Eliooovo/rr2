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

## 模块开关与调试指南

### 上电自测 (不用上位机，上电自动跑)

| 文件 | 宏 | 作用 |
|------|-----|------|
| `Modules/chassis.h` | `CHASSIS_BOOT_TEST_ENABLE` | `1` = 上电底盘自转 1 秒后停止 |
| `Modules/chassis.h` | `CHASSIS_BOOT_TEST_VX/VY/WZ` | 分别设为 `1.0f` 测前进/横移/旋转 (保持 0 或 1) |
| `Modules/lift.h` | `LIFT_BOOT_TEST_ENABLE` | `1` = 上电后升降电机等在线→设零→转到 LIFT_BOOT_TEST_DEG 度 |
| `Modules/lift.h` | `LIFT_BOOT_TEST_DEG` | 目标角度 (度)，如 `180.0f` |

### 全局限速 (测试 + 上位机指令都生效)

| 文件 | 宏 | 作用 |
|------|-----|------|
| `Modules/chassis.h` | `CHASSIS_VX_SCALE` | 前后方向限速系数 (0.0~1.0) |
| `Modules/chassis.h` | `CHASSIS_VY_SCALE` | 左右方向限速系数 |
| `Modules/chassis.h` | `CHASSIS_VW_SCALE` | 旋转方向限速系数 |

> 电机实际速度 = 上位机/BOOT_TEST 值 × SCALE。调试时先设小值 (如 0.2)，确认方向正确后再加大。

### 底盘 PID 参数

| 文件 | 位置 | 说明 |
|------|------|------|
| `Modules/chassis.h` | `CHASSIS_MOTOR_CONFIG_INIT` | 每个电机的 `speed_kp/ki/kd`，四个值独立可调 |
| `Modules/chassis.h` | `CHASSIS_CURRENT_LIMIT` | PID 输出电流限幅 (C620 最大 16384) |

### 底盘电机方向

| 文件 | 位置 | 说明 |
|------|------|------|
| `Modules/chassis.h` | `direction` 字段 | 试车时若某轮反转，改对应值为 `-1` |

### 升降 PID 参数

| 文件 | 位置 | 说明 |
|------|------|------|
| `Modules/lift.h` | `LIFT_MOTOR_CONFIG_INIT` | 每电机的 `position_kp/ki/kd` (位置环) + `speed_kp/ki/kd` (速度环) |
| `Modules/lift.h` | `max_speed_rpm` | 位置环输出限幅，即最大运动速度 |
| `Modules/lift.h` | `current_limit` | 速度环输出限幅 (C620 最大 16384) |

### 串口通讯切换

| 文件 | 宏 | 作用 |
|------|-----|------|
| `Modules/comm_protocol.h:` | `COMM_USB_TEST_FRAME_ENABLE` | `1` = 每秒发固定 46 字节测试帧验证 USB CDC 链路；`0` = 发真实反馈帧 (20ms 周期) |

> 切到真实反馈帧 (`COMM_USB_TEST_FRAME_ENABLE = 0`) 且把上电自测全关 (`BOOT_TEST_ENABLE = 0`) 后，底盘和升降会完全听从 Jetson 上位机通过 USB 虚拟串口发来的指令。
>
> 帧格式: `0xAA` + 11 个 float (小端序) + `0x55`，共 46 字节。字段顺序见 `Modules/comm_protocol.h` 的 `CommFrameFloats`。

### 升降电机 Can ID

| 文件 | 位置 | 说明 |
|------|------|------|
| `Modules/lift.h` | `LIFT_MOTOR_CONFIG_INIT` 的 `motor_id` 字段 | 当前设为 `{1U, 2U, 3U, 4U}`，换 ID 改这里 |

### 底盘麦轮公式

文件 `Modules/chassis.c` (`Chassis_SetVelocity`) 和 `:204-207` (`Chassis_SetVelocityRpm`):

```c
// 标准麦轮解算 (DJI 滚子布局)，不要随意改符号
rf = vx - vy - wz*L    // 右前
lf = vx + vy + wz*L    // 左前
lb = vx - vy + wz*L    // 左后
rb = vx + vy - wz*L    // 右后
```

### 典型调试流程

```
1. CHASSIS_BOOT_TEST_ENABLE = 1, BOOT_TEST_VX=1, VY=0, WZ=0 → 确认前进方向
   → 某轮反转? 改 direction = -1

2. CHASSIS_BOOT_TEST_ENABLE = 1, VX=0, VY=1, WZ=0 → 确认横移方向
   → 四个轮子各自转动方向正确但车体横移方向反了?
   → 交换 VX_SCALE/VY_SCALE 的符号 (改 -0.4)

3. CHASSIS_BOOT_TEST_ENABLE = 1, VX=0, VY=0, WZ=1 → 确认旋转方向

4. 确认无误 → BOOT_TEST_ENABLE 全关 → COMM_USB_TEST_FRAME_ENABLE = 0

5. 连 Jetson，上位机发指令控制
```
6.上位机发送指令+下位机响应： Comm_RunPeriodic(); 
                          Chassis_RunPeriodic();开启

## 选中一个git，右键选择“复制提交哈希”（eg：880985aa00ae9001ce78267b9054acc993f27f89）给ai，ai能查看当时的修改/保存

## 波特率： CAN 通信的物理层约定，就像两个人打电话必须说同一种语言、同一个语速。
如果双方（我的stm32cubemx配置的和电机）另一个波特率不匹配：

  MCU @ 1Mbps:    |‾|_|‾‾|__|‾|_|‾|_|‾‾|     ← 1 bit = 1μs
  MCU @ 600kbps:  |‾‾|___|‾‾‾|___|‾‾|___|     ← 1 bit = 1.67μs

## 编译失败：右下角出现 “CMake 可执行文件错误: "cube-cmake"。请检查以确保它已安装，或者 "cmake.cmakePath" 设置的值包含正确的路径”
 检查vscode配置是否写死了："cmake.cmakePath": "cube-cmake"
 VS Code CMake Tools 插件不会去调用系统默认的 cmake，而是专门去找一个叫 cube-cmake 的可执行文件。以前不报错，说明当时你的环境里大概率满足了其中一种情况：
1、cube-cmake 这个命令以前存在，比如 STM32CubeIDE / STM32 VS Code 插件装过或配置过它。
2、以前 VS Code 的环境变量里能找到它，但现在终端/VS Code 启动环境变了。
3、STM32 相关插件、Cube 工具链、路径配置或软链接被更新、卸载、移动了。
4、你以前不是用这个工作区的 .vscode/settings.json 配置编译，而是用了系统 CMake 或别的 profile。
5、VS Code/CMake Tools 插件更新后更严格地按 cmake.cmakePath 执行，不再兜底找普通 cmake。
你的机器现在的实际情况是：/usr/bin/cmake 存在，cube-cmake 不存在。
   "cmake.cmakePath": "/usr/bin/cmake",  ->正确

后续任务： 抬升->发的m->下位机转成应该转的度数