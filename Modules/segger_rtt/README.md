# SEGGER RTT

本目录直接集成 SEGGER 官方 RTT target source，为使用 J-Link/Ozone 时提供非阻塞调试输出。
工程不再增加 App、对象层或二次 API 封装，调用方直接使用 `SEGGER_RTT_*` 接口。

## 来源与许可证

- 上游仓库：<https://github.com/SEGGERMicro/RTT>
- 固定版本：`V8.58.0`
- 发布包：`SEGGER.RTT.8.58.0.pack`
- 发布包 SHA-256：`4bb3aa005922141960d013378cfd89d2148353520e40679195385a8305f58de6`
- 许可证见本目录的 `LICENSE.md`。

`SEGGER_RTT.c`、`SEGGER_RTT.h`、`SEGGER_RTT_ConfDefaults.h` 和
`SEGGER_RTT_printf.c` 保持官方版本不变。工程自己的配置仅位于
`SEGGER_RTT_Conf.h`，升级上游代码时不得覆盖该文件。

## 特例说明

> **禁止普通 Module 参考本模块的结构。**

RTT 是调试器与目标 RAM 之间的第三方基础设施，官方实现固有地使用全局控制块和静态缓冲区。
因此本模块经明确批准，不采用项目一般 Module 所要求的调用方对象、对象指针初始化以及
`config/internal/state` 组织方式，也直接公开官方 API。

该例外只适用于本目录。其他设备、协议和功能 Module 不得模仿这里的全局状态、无对象 API、
官方源码直接暴露或缺少分层封装等做法，仍须遵守仓库 `AGENTS.md` 的一般模块开发规范。

## 工程配置

- 仅启用 channel 0：一个 1024 字节上行缓冲区和一个 16 字节下行缓冲区。
- channel 0 使用 `SEGGER_RTT_MODE_NO_BLOCK_SKIP`；空间不足时丢弃当前写入，不等待主机。
- 控制块和缓冲区通过 `segger_rtt.ld` 放入 `.segger_rtt`，实际位于 STM32H723 的 DTCM。
- D-Cache 保持启用；DTCM 不经过 D-Cache，因此 J-Link 后台读写不会产生缓存不一致。
- `.segger_rtt` 为 `NOLOAD`，启动时必须显式调用 `SEGGER_RTT_Init()`。
- 本工程不重定向标准库 `printf()`，也不编译 RTT syscall 和汇编优化文件。

不要删除 `segger_rtt.ld`，也不要把 `.segger_rtt` 改回普通 `.bss`。当前普通 `.bss` 位于启用
D-Cache 的 AXI SRAM，J-Link 后台访问与 CPU 缓存可能看到不一致的数据。

## 使用方法

系统启动代码在 `HAL_Init()`和 `SystemClock_Config()`完成后、所有外设与业务 App 初始化前执行：

```c
#include "SEGGER_RTT.h"

SEGGER_RTT_Init();
SEGGER_RTT_WriteString(0, "[rr2] RTT ready\r\n");
```

其他位置只需包含官方头文件并调用 channel 0：

```c
#include "SEGGER_RTT.h"

SEGGER_RTT_WriteString(0, "motor online\r\n");
SEGGER_RTT_printf(0, "motor id=%u state=%d pos=%d\r\n",
                  motor_id,
                  state,
                  position);
```

`SEGGER_RTT_printf()` 是精简格式化函数，只支持 `%c`、`%d`、`%u`、`%x`、`%s`、`%p` 和 `%%`；
不支持 `%f`。浮点值应先转换成定点整数，或由调用方自行格式化后使用 `SEGGER_RTT_Write()`。

RTT 本身不需要周期函数或反初始化函数。J-Link 会在 CPU 运行时后台读取环形缓冲区。

## 实时性与并发约束

- 非阻塞模式只保证不会等待 RTT 缓冲区腾出空间，格式化和复制本身仍消耗 CPU 时间。
- 禁止在高频中断和严格实时控制热路径中调用 `SEGGER_RTT_printf()`。
- 中断中确需记录信息时，优先只更新调试快照，由主循环低频输出。
- 不得把影响业务状态的表达式放进日志参数，日志被删除或丢弃时业务行为必须保持不变。
- 日志可能因缓冲区已满而丢失，不得把 RTT 用作可靠通信链路或故障安全机制。

## Ozone 查看

1. 使用当前构建生成且带符号的 `rr2.elf` 创建或更新 Ozone 工程。
2. 连接 J-Link、下载固件并运行 CPU。
3. 打开 Ozone 的 RTT Terminal 窗口；Ozone 应通过 ELF 中的 `_SEGGER_RTT` 自动定位控制块。
4. 每次复位后应看到 `[rr2] RTT ready`。

Ozone、J-Link RTT Viewer、GDB Server 等通常不能同时独占同一个 J-Link。使用 Ozone 时不要同时
启动另一个 RTT/J-Link 客户端。若终端没有输出，先确认加载的 ELF 与板上固件一致，并检查
`_SEGGER_RTT` 是否位于 `0x20000000` 到 `0x2001FFFF` 的 DTCM 地址范围。
