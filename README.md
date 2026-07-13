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
