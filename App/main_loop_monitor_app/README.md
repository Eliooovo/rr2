# Main Loop Monitor App

该 App 使用 Cortex-M7 DWT 周期计数器测量一轮主循环任务耗时；耗时超过预期周期时，通过 SEGGER RTT 0 号通道输出非阻塞警告。

## 使用前必须修改

烧录前，必须在 `main_loop_monitor_app.c` 中把以下宏修改为项目要求的实际主循环频率：

```c
#define MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ 1000U
```

该值是主循环的预期最低频率，App 据此计算允许的最长周期：

```text
周期上限（us） = 1000000 / 预期频率（Hz）
```

例如：

| 预期频率 | 周期上限 |
| --- | --- |
| 1000 Hz | 1000 us |
| 500 Hz | 2000 us |
| 100 Hz | 10000 us |

代码中的 `1000U` 只是按当前 1 ms 控制任务给出的初始示例值，不能代替项目的实际频率要求。

## 接入方式

在其他 App 都初始化完成后初始化监测 App：

```c
MainLoopMonitorApp_Init();
```

在主循环所有任务执行完成后调用周期函数：

```c
while (1) {
    /* Other App RunPeriodic calls. */
    MainLoopMonitorApp_RunPeriodic();
}
```

`MainLoopMonitorApp_RunPeriodic()` 必须位于主循环末尾。App 会在完成本次检测和可能的 RTT 输出后重新记录起点，因此 RTT 告警自身的耗时不会触发连锁超时。

## RTT 输出

启动成功：

```text
[rr2] RTT ready
```

主循环超时：

```text
[WARN] main loop timeout: elapsed=1237 us, limit=1000 us (1000 Hz), count=1
```

如果 DWT 不可用或无法启用，App 会通过 RTT 输出错误并停止检测，不影响其他 App 继续运行。

## 需要核对

- J-Link RTT Viewer 使用 0 号上行通道。
- RTT 缓冲区保持非阻塞模式，避免日志反向阻塞主循环。
- 实机烧录后确认 DWT 计时和 RTT 警告均正常。
