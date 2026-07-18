#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==========================================================================
 * DJI M3508 电机驱动 (配 C620 电调)
 *
 * 硬件说明:
 *   - M3508 是 DJI RoboMaster 系列的三相无刷减速电机
 *   - C620 电调通过 CAN 总线与 MCU 通信
 *   - 电调以 1kHz 频率主动上报电机状态 (无需 MCU 请求)
 *   - MCU 发送电流指令给电调，电调内部完成电流闭环
 *
 * CAN 协议:
 *   - 反馈帧 ID: 0x201~0x208 (对应电机 ID 1~8)
 *   - 命令帧 ID: 0x200 (电机 1~4), 0x1FF (电机 5~8)
 *   - 每帧 8 字节，每个电机电流占 2 字节 (大端序)
 *
 * 电流范围:
 *   - C620 最大电流: ±16384 (对应 ±20A 左右)
 *   - 正电流 = 电机正转 (取决于实际接线和安装方向)
 * ========================================================================== */

/* --- 电机数量 ---
 * 根据 README.md 的总线分配:
 *   FDCAN1: 底盘 4 个 3508 (麦轮电机)
 *   FDCAN2: 升降 4 个 3508
 * 总共 8 个，所以这里定义 8。
 */
#define DJI_MOTOR_COUNT             8U

/* --- CAN 反馈帧 ID (电调 → MCU) ---
 * 0x201 对应电机 1，0x202 对应电机 2，以此类推到 0x208。
 */
#define DJI_MOTOR_FEEDBACK_ID_BASE  0x201U
#define DJI_MOTOR_FEEDBACK_ID_MIN   0x201U
#define DJI_MOTOR_FEEDBACK_ID_MAX   0x208U

/* --- CAN 命令帧 ID (MCU → 电调) ---
 * 0x200: 电机 1~4 的电流指令
 * 0x1FF: 电机 5~8 的电流指令
 */
#define DJI_MOTOR_CMD_ID_1_TO_4     0x200U
#define DJI_MOTOR_CMD_ID_5_TO_8     0x1FFU

/* --- 编码器参数 ---
 * C620 反馈的单圈编码器范围: 0~8191
 * 即每圈 8192 个脉冲，精度约 0.044°
 * 8192 / 2 = 4096 用于判断过圈方向 (半圈跳变即视为过圈)
 */
#define DJI_MOTOR_ENCODER_RANGE     8192
#define DJI_MOTOR_ENCODER_HALF      4096

/* --- 电流限幅 ---
 * C620 电流指令范围 ±16384，超出部分裁剪到此值
 */
#define DJI_MOTOR_CURRENT_LIMIT     16384

/* ==========================================================================
 * DjiMotorState - 单个电机的完整状态
 *
 * 由 DjiMotor_HandleFeedback() 在收到 CAN 反馈时更新。
 * 上层可以直接读取这个结构体获取电机实时状态。
 * ========================================================================== */
typedef struct {
    uint8_t id;                /* 电调 ID，范围 1~8，对应 CAN 反馈 ID 0x201~0x208 */
    uint8_t online;            /* 在线标志: 0=未收到过反馈, 1=已在线 (收到过至少一次 CAN 反馈) */

    /* --- 单圈编码器 (来自 C620 反馈) --- */
    uint16_t encoder;          /* 当前单圈机械角度编码器值，范围 0~8191 (0~360°) */
    uint16_t last_encoder;     /* 上一次反馈的 encoder 值，用于计算过圈方向 */

    /* --- 速度与电流 (来自 C620 反馈) --- */
    int16_t speed_rpm;         /* 当前转速，单位 rpm (转/分钟) */
    int16_t given_current;     /* 电调回传的当前转矩电流值 (不是 MCU 发出的命令值) */

    /* --- 温度 --- */
    uint8_t temperature;       /* 电调温度，单位 ℃，超过 80℃ 建议降低功率 */

    /* --- 多圈累计 (由本驱动软件计算，非 C620 直接反馈) --- */
    int32_t round_count;       /* 累计圈数，正向过一圈 +1，反向过一圈 -1 */
    int32_t total_encoder;     /* 多圈累计编码器值 = round_count * 8192 + encoder */
    float   total_angle_deg;   /* 多圈累计角度，单位度。例如 361.0° 表示正向转了一圈多 */

    /* --- 时间戳 --- */
    uint32_t update_count;     /* 累计收到该电机反馈的次数 */
    uint32_t last_update_ms;   /* 最近一次收到反馈的时间，由调用方传入 (如 HAL_GetTick()) */
} DjiMotorState;

/* 全局电机状态数组，索引 = motor_id - 1 */
extern DjiMotorState g_dji_motors[DJI_MOTOR_COUNT];

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/* 初始化所有电机状态为默认值 (清零，ID 设为 1~8，online=0) */
void DjiMotor_Init(void);

/* ==========================================================================
 * CAN ID 判断与转换
 * ========================================================================== */

/* 判断一个 CAN std_id 是否为 DJI 电机反馈帧 (0x201~0x208) */
uint8_t DjiMotor_IsFeedbackId(uint16_t std_id);

/* 从 CAN 反馈 ID 提取电机编号。
 * 例如 std_id=0x203 返回 3，非反馈帧返回 0 */
uint8_t DjiMotor_GetMotorIdFromFeedbackId(uint16_t std_id);

/* ==========================================================================
 * CAN 反馈处理 (接收侧)
 *
 * 调用时机: 在 CAN 接收中断/回调中，收到 0x201~0x208 的帧时调用。
 *
 * 参数:
 *   std_id: CAN 消息 ID
 *   data:   8 字节 CAN 数据
 *   now_ms: 当前系统时间 (ms)，用于记录 last_update_ms
 *
 * 返回值: 解析到的电机 ID (1~8)，非 DJI 反馈帧返回 0
 * ========================================================================== */
uint8_t DjiMotor_HandleFeedback(uint16_t std_id, const uint8_t data[8], uint32_t now_ms);

/* ==========================================================================
 * 状态查询
 * ========================================================================== */

/* 获取指定电机的状态指针，motor_id 范围 1~8，非法 ID 返回 NULL */
DjiMotorState *DjiMotor_GetState(uint8_t motor_id);

/* ==========================================================================
 * 电流命令 (发送侧)
 *
 * 重要: 这些函数只更新软件缓存，不直接发送 CAN 帧。
 * 上层需要定时调用 Build 函数打包数据，然后通过 HAL FDCAN 发送。
 *
 * 典型使用流程 (1kHz 定时器/PID 任务):
 *   1. 根据控制算法算出目标电流
 *   2. DjiMotor_SetCurrent(motor_id, current) 设置电流
 *   3. DjiMotor_BuildAllCurrentFrames(buf1, buf2) 打包成 CAN 帧
 *   4. HAL_FDCAN_AddMessageToTxFifo() 发送到 CAN 总线
 * ========================================================================== */

/* 将所有电机的电流命令清零 (用于紧急停止或初始化) */
void DjiMotor_ClearCurrents(void);

/* 设置单个电机的电流指令，自动限幅到 ±16384。
 * motor_id: 1~8
 * current:  目标电流，范围 [-16384, 16384] */
void DjiMotor_SetCurrent(uint8_t motor_id, int16_t current);

/* 读取当前缓存的电流指令 (非电调回传的实际电流) */
int16_t DjiMotor_GetCurrent(uint8_t motor_id);

/* 将 4 个电机的电流指令打包成一帧 8 字节数据。
 * cmd_id: 0x200 (打包电机 1~4) 或 0x1FF (打包电机 5~8)
 * data:   输出的 8 字节，按 DJI 协议大端序排列
 * 返回值: 1=成功, 0=cmd_id 非法 */
uint8_t DjiMotor_BuildCurrentFrame(uint16_t cmd_id, uint8_t data[8]);

/* 同时打包两帧:
 * data_1_to_4: 电机 1~4 的 8 字节 (CAN ID 0x200)
 * data_5_to_8: 电机 5~8 的 8 字节 (CAN ID 0x1FF) */
void DjiMotor_BuildAllCurrentFrames(uint8_t data_1_to_4[8], uint8_t data_5_to_8[8]);

#ifdef __cplusplus
}
#endif

#endif
