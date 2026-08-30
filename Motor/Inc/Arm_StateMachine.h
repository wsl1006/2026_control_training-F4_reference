#ifndef __ARM_STATE_MACHINE_H
#define __ARM_STATE_MACHINE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * ============================================================
 *                    机械臂数量
 * ============================================================
 */

#define ARM_NUM                 2U

#define ARM_LEFT                0U
#define ARM_RIGHT               1U


/*
 * ============================================================
 *                    气泵状态变量
 * ============================================================
 *
 * 直接写 0 / 1 控制：
 *
 *      arm_left_pump_on  = 1  → 左臂气泵开(PA9 高)
 *      arm_left_pump_on  = 0  → 左臂气泵关
 *
 *      arm_right_pump_on = 1  → 右臂气泵开(PA2 高)
 *      arm_right_pump_on = 0  → 右臂气泵关
 *
 * 由 Arm_Task 每 5ms 应用到对应引脚。
 *
 * 机械臂收到 WORK 命令时状态机自动置 1(开始工作即保持)，
 * 收到 PULLDOWN 时自动清 0；
 * 手动写值随时生效。
 * ============================================================
 */

extern volatile bool arm_left_pump_on;

extern volatile bool arm_right_pump_on;


/*
 * ============================================================
 *                    CAN ID
 * ============================================================
 *
 * 主控 -> 从板：
 *
 *      0x010105xx
 *
 * 从板 -> 主控：
 *
 *      0x050101xx
 *
 * CAN1：
 *
 *      DJI 3508
 *
 * CAN2：
 *
 *      Zdrive AK80
 *
 * 注意：
 *
 * 这里的 CAN ID 是“主板和机械臂从板之间”的通信协议。
 *
 * DJI / Zdrive 自己的电机 CAN 协议，
 * 由各自底层驱动处理。
 * ============================================================
 */

#define ARM_CMD_BASE_ID          0x01010500U
#define ARM_FB_BASE_ID           0x05010100U

#define ARM_DEVICE_ID            0x05U


/*
 * ============================================================
 *                    主控 -> 从板 命令码
 * ============================================================
 */

typedef enum
{
    /*
     * 自检
     */
    ARM_CMD_SELF_CHECK = 0x00,

    /*
     * 机械臂总使能
     *
     * data0：
     *
     *      0 = 失能
     *      1 = 使能
     */
    ARM_CMD_ENABLE = 0x01,


    /*
     * 左机械臂 WORK_1
     *
     * 完整动作：
     *
     *      Z=0.175m
     *          ↓
     *      气泵 ON
     *          ↓
     *      Z=0.1m
     *          ↓
     *      取块完成
     */
    ARM_CMD_LEFT_WORK_1 = 0x03,


    /*
     * 左机械臂 WORK_2
     *
     * 完整动作：
     *
     *      Z=0.525m
     *          ↓
     *      气泵 OFF
     *          ↓
     *      Z=0.8m
     *          ↓
     *      放块完成
     */
    ARM_CMD_LEFT_WORK_2 = 0x04,


    /*
     * 右机械臂 WORK_1
     */
    ARM_CMD_RIGHT_WORK_1 = 0x05,


    /*
     * 右机械臂 WORK_2
     */
    ARM_CMD_RIGHT_WORK_2 = 0x06,


    /*
     * 左机械臂直接关闭气泵
     */
    ARM_CMD_LEFT_PULLDOWN = 0x07,


    /*
     * 右机械臂直接关闭气泵
     */
    ARM_CMD_RIGHT_PULLDOWN = 0x08,


    /*
     * 错误命令
     */
    ARM_CMD_ERROR = 0xEE,


    /*
     * RESET
     *
     * 回到机械臂上电时记录的位置。
     *
     * data0：
     *
     *      0 = 左右双臂
     *      1 = 左臂
     *      2 = 右臂
     */
    ARM_CMD_RESET = 0xFF

} ArmCommand_t;


/*
 * ============================================================
 *                    Queue 消息
 * ============================================================
 *
 * CAN 接收中断只负责：
 *
 *      CAN RX
 *        ↓
 *      解析命令
 *        ↓
 *      Queue
 *
 * 不在中断里控制电机。
 * ============================================================
 */

typedef struct
{
    ArmCommand_t cmd;

    uint8_t data0;

    uint8_t data1;

} ArmCommandMsg_t;


/*
 * ============================================================
 *                    从板 -> 主控 反馈
 * ============================================================
 */

#define ARM_FB_STARTED          0x00U
#define ARM_FB_FINISHED         0x01U
#define ARM_FB_REJECTED         0x02U


/*
 * ============================================================
 *                    机械臂状态
 * ============================================================
 */

typedef enum
{
    /*
     * 空闲
     */
    ARM_STATE_IDLE = 0,

    /*
     * RESET：
     *
     * 回到上电记录位置
     */
    ARM_STATE_RESET,

    /*
     * 释放气泵
     */
    ARM_STATE_PULLDOWN,

    /*
     * WORK_1：
     *
     * 取块
     */
    ARM_STATE_WORK_1,

    /*
     * WORK_2：
     *
     * 放块
     */
    ARM_STATE_WORK_2

} ArmState_t;


/*
 * ============================================================
 *                    状态机步骤
 * ============================================================
 */

typedef enum
{
    /*
     * 空闲
     */
    ARM_STEP_IDLE = 0,


    /*
     * 设置两个电机目标位置
     */
    ARM_STEP_SET_POSITION,


    /*
     * 等待两个电机到位
     */
    ARM_STEP_WAIT_POSITION,


    /*
     * 开气泵
     */
    ARM_STEP_PUMP_ON,


    /*
     * 关气泵
     */
    ARM_STEP_PUMP_OFF,


    /*
     * 动作完成
     */
    ARM_STEP_FINISH

} ArmStep_t;


/*
 * ============================================================
 *                    工作阶段
 * ============================================================
 *
 * 这个变量和 state 不一样。
 *
 * state：
 *
 *      当前正在执行什么动作。
 *
 * work_stage：
 *
 *      当前机械臂是否持有方块。
 *
 *
 * 最重要的一点：
 *
 *      RESET 不修改 work_stage。
 *
 * 例如：
 *
 *      WORK_1
 *          ↓
 *      work_stage = HAS_BLOCK
 *
 *      RESET
 *          ↓
 *      work_stage 仍然 = HAS_BLOCK
 *
 *      小车移动
 *          ↓
 *      WORK_2
 *          ↓
 *      放块
 *          ↓
 *      work_stage = IDLE
 *
 * ============================================================
 */

typedef enum
{
    /*
     * 没有方块
     */
    ARM_WORK_IDLE = 0,

    /*
     * 当前持有方块
     */
    ARM_WORK_HAS_BLOCK

} ArmWorkStage_t;


/*
 * ============================================================
 *                    每个机械臂控制结构
 * ============================================================
 */

typedef struct
{
    /*
     * 当前状态
     */
    ArmState_t state;


    /*
     * 当前状态内部步骤
     */
    ArmStep_t step;


    /*
     * Busy 锁
     *
     * false：
     *      当前机械臂空闲
     *
     * true：
     *      当前机械臂正在执行动作
     */
    volatile bool busy;


    /*
     * 当前工作阶段
     *
     * IDLE：
     *      没有方块
     *
     * HAS_BLOCK：
     *      手上有方块
     */
    ArmWorkStage_t work_stage;


    /*
     * ========================================================
     * 上电位置
     * ========================================================
     *
     * 上电后等待收到电机有效反馈，
     * 然后记录当前位置。
     *
     * RESET 时直接回到这里。
     *
     * 单位：
     *
     *      degree
     *
     * 注意：
     *
     * 这里保存的是“绝对位置”。
     *
     * 不是：
     *
     *      相对转动角度。
     */
    float power_on_dji_angle_deg;

    float power_on_ak80_angle_deg;


    /*
     * ========================================================
     * 当前目标位置
     * ========================================================
     *
     * 状态机给电机下发目标以后，
     * 每个周期可以重新写入。
     *
     * 这样可以避免切换位置模式时，
     * 驱动目标值被覆盖。
     */
    float set_dji_angle_deg;

    float set_ak80_angle_deg;


    /*
     * ========================================================
     * 当前动作命令
     * ========================================================
     *
     * 用于动作完成以后反馈给主控。
     */
    ArmCommand_t last_cmd;

} ArmCtrl_t;


/*
 * ============================================================
 *                    初始化
 * ============================================================
 */

/*
 * 创建 Queue。
 *
 * 注意：
 *
 * 不创建 FreeRTOS Task。
 *
 * Arm_Task 由 freertos.c / CMSIS-RTOS 创建。
 */
void Arm_StateMachine_Init(void);


/*
 * 机械臂 FreeRTOS Task。
 *
 * 不要在 main while 中调用。
 */
void Arm_Task(void *argument);


/*
 * ============================================================
 *                    命令接口
 * ============================================================
 */

/*
 * 普通任务调用。
 */
bool Arm_SendCommand(
    ArmCommand_t command,
    uint8_t data0,
    uint8_t data1);


/*
 * CAN 接收中断调用。
 *
 * 这个函数内部使用：
 *
 *      xQueueSendFromISR()
 */
bool Arm_SendCommandFromISR(
    ArmCommand_t command,
    uint8_t data0,
    uint8_t data1);


/*
 * ============================================================
 *                    状态查询
 * ============================================================
 */

bool Arm_IsBusy(uint8_t arm_id);

ArmState_t Arm_GetState(uint8_t arm_id);


/*
 * ============================================================
 *                    电机反馈 Ready
 * ============================================================
 *
 * 收到第一帧有效反馈以后置 true。
 *
 * Arm_Task 启动以后：
 *
 *      等待四个电机反馈
 *          ↓
 *      记录上电位置
 *
 * ============================================================
 */

extern volatile bool arm_left_dji_ready;

extern volatile bool arm_left_zdrive_ready;

extern volatile bool arm_right_dji_ready;

extern volatile bool arm_right_zdrive_ready;


/*
 * ============================================================
 *                    电机反馈通知
 * ============================================================
 *
 * 在对应 CAN RX 回调中调用。
 * ============================================================
 */

void Arm_NotifyDjiFeedback(uint8_t motor_index);

void Arm_NotifyZdriveFeedback(uint8_t motor_index);


#endif