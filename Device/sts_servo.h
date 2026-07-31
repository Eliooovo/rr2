#ifndef STS_SERVO_H
#define STS_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/** STS 系列舵机驱动状态码。 */
typedef enum {
    STS_SERVO_OK = 0,
    STS_SERVO_ERROR_PARAM,
    STS_SERVO_ERROR_CONFIG,
    STS_SERVO_ERROR_NOT_INIT,
    STS_SERVO_ERROR_TIMEOUT,
    STS_SERVO_ERROR_CRC,
} sts_servo_status_t;

#define STS_SERVO_OK              STS_SERVO_OK
#define STS_SERVO_ERROR_PARAM     STS_SERVO_ERROR_PARAM
#define STS_SERVO_ERROR_CONFIG    STS_SERVO_ERROR_CONFIG
#define STS_SERVO_ERROR_NOT_INIT  STS_SERVO_ERROR_NOT_INIT
#define STS_SERVO_ERROR_TIMEOUT   STS_SERVO_ERROR_TIMEOUT
#define STS_SERVO_ERROR_CRC       STS_SERVO_ERROR_CRC

/** 舵机配置，调用方在 init 前填写。 */
typedef struct {
    UART_HandleTypeDef *huart; /**< HAL UART 句柄。                  */
    uint8_t             id;    /**< 舵机总线 ID，有效范围 1~253。     */
} sts_servo_config_t;

/** 舵机反馈数据，由 get_feedback 填充。 */
typedef struct {
    float   pos_rad;        /**< 当前位置 [rad]，范围 0 ~ 2π。       */
    float   speed_rad_s;    /**< 当前速度 [rad/s]。                  */
    int16_t load;           /**< 当前负载（原始值）。                 */
    uint8_t voltage_x10;    /**< 供电电压 [0.1 V]。                  */
    uint8_t temperature_c;  /**< 内部温度 [°C]。                      */
    uint8_t moving;         /**< 运动标志，1 表示正在运动。           */
} sts_servo_feedback_t;

/** 舵机运行状态。 */
typedef struct {
    uint8_t online;  /**< 通信正常。 */
    uint8_t enabled; /**< 扭矩已使能。 */
} sts_servo_state_t;

/** 驱动私有数据，调用方不应直接访问。 */
typedef struct {
    uint8_t  initialized;
    uint32_t last_update_ms;

    /* 发送缓冲，供 SCS 协议层 writeSCS/writeByteSCS 累积数据。 */
    uint8_t  tx_buf[128];
    uint8_t  tx_len;
} sts_servo_internal_t;

/** STS 系列舵机对象。 */
typedef struct {
    sts_servo_config_t   config;
    sts_servo_feedback_t feedback;
    sts_servo_state_t    state;
    sts_servo_internal_t internal;
} sts_servo_t;

/**
 * @brief 初始化舵机对象并探测在线状态。
 *
 * @param servo 舵机对象指针，config 成员必须已填写。
 * @return STS_SERVO_OK 初始化成功且舵机在线。
 */
sts_servo_status_t sts_servo_init(sts_servo_t *servo);

/**
 * @brief 使能扭矩输出，舵机保持当前位置。
 *
 * @param servo 舵机对象指针。
 * @return STS_SERVO_OK 使能成功。
 */
sts_servo_status_t sts_servo_enable(sts_servo_t *servo);

/**
 * @brief 失能扭矩输出，舵机可自由转动。
 *
 * @param servo 舵机对象指针。
 * @return STS_SERVO_OK 失能成功。
 */
sts_servo_status_t sts_servo_disable(sts_servo_t *servo);

/**
 * @brief 设置目标位置。
 *
 * 内部将弧度转为 0~4095 原始值，调用 WritePosEx。
 *
 * @param servo   舵机对象指针。
 * @param pos_rad 目标位置 [rad]。
 * @param speed   目标速度，0 表示由舵机内部控制。
 * @param acc     加速度等级，0 表示由舵机内部控制。
 * @return STS_SERVO_OK 写入成功。
 */
sts_servo_status_t sts_servo_set_position(sts_servo_t *servo,
                                          float pos_rad,
                                          uint16_t speed,
                                          uint8_t acc);

/**
 * @brief 设置目标位置（原始编码器值）。
 *
 * 直接传入舵机原始位置值 0~4095，不经过弧度转换。
 *
 * @param servo 舵机对象指针。
 * @param raw   目标位置原始值 [0, 4095]。
 * @param speed 目标速度，0 表示由舵机内部控制。
 * @param acc   加速度等级，0 表示由舵机内部控制。
 * @return STS_SERVO_OK 写入成功。
 */
sts_servo_status_t sts_servo_set_position_raw(sts_servo_t *servo,
                                              int16_t raw,
                                              uint16_t speed,
                                              uint8_t acc);

/**
 * @brief 批量读取舵机反馈寄存器并填入 feedback 成员。
 *
 * 一次 Read 读取地址 56~70 共 15 字节。
 *
 * @param servo 舵机对象指针。
 * @param fb    外部提供的反馈缓冲区，填入后也写入 servo->feedback。
 * @return STS_SERVO_OK 读取且校验成功。
 */
sts_servo_status_t sts_servo_get_feedback(sts_servo_t *servo,
                                          sts_servo_feedback_t *fb);

/**
 * @brief 周期更新在线状态。
 *
 * 调用方应以不慢于离线超时周期（如 100 ms）的频率调用。
 *
 * @param servo 舵机对象指针。
 * @param now_ms HAL_GetTick() 返回值。
 * @return STS_SERVO_OK。
 */
sts_servo_status_t sts_servo_update(sts_servo_t *servo, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* STS_SERVO_H */
