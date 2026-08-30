#include "Arm_StateMachine.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "can.h"

#include <math.h>
#include <string.h>


/*
 * ============================================================
 *                    外部电机对象
 * ============================================================
 *
 * DJI 3508：
 *
 *      CAN1
 *
 * Zdrive AK80：
 *
 *      CAN2
 *
 * 这里假设你的工程中已经定义：
 *
 *      DJmotor[]
 *      Zmotor[]
 *
 * ============================================================
 */

extern DJMotor DJmotor[];

extern Zdrive Zmotor[];


/*
 * ============================================================
 *                    电机下标
 * ============================================================
 *
 * 当前假设：
 *
 *      左臂：
 *
 *          DJmotor[0] -> 3508
 *          Zmotor[0]  -> AK80
 *
 *
 *      右臂：
 *
 *          DJmotor[1] -> 3508
 *          Zmotor[1]  -> AK80
 *
 *
 * 如果你的数组编号不同，只修改这里。
 * ============================================================
 */

#define LEFT_DJI_INDEX          2U
#define RIGHT_DJI_INDEX         1U

#define LEFT_ZDRIVE_INDEX       0U
#define RIGHT_ZDRIVE_INDEX      1U


/*
 * ============================================================
 *                    状态机参数
 * ============================================================
 */


/*
 * 位置判断误差。
 *
 * 单位：
 *
 *      degree
 *
 * 两个电机都进入这个误差范围，
 * 才认为机械臂到位。
 */
#define ARM_POSITION_ERROR_DEG          1.0f


/*
 * 状态机任务周期。
 *
 * 单位：
 *
 *      ms
 */
#define ARM_TASK_PERIOD_MS              5U


/*
 * Queue 长度。
 */
#define ARM_QUEUE_LENGTH                8U


/*
 * ============================================================
 *                    位置角度
 * ============================================================
 *
 * !!! 这里是相对上电位置的转角 !!!
 *
 * 单位：degree
 *
 * AK80 绝对编码器每次上电零点会漂，
 * 所以不用绝对角度，改用相对转角：
 *
 *      目标位置 = 上电位置 + 这里的值
 *
 * 标定方法：
 *      上电 → 手动把机械臂摆到目标高度 →
 *      记下(当前读数 - 上电读数) → 填到这里。
 *
 * 左臂已标定(2026-08-28 实测)：
 *      0.175m: AK80 +546, 3508 +0
 *      0.1m  : AK80 +646, 3508 +0
 *      0.525m: AK80 +146, 3508 +43
 *      0.8m  : AK80 +196, 3508 +88
 *
 * 右臂未标定，仍是 0。
 * ============================================================
 */


/*
 * ============================================================
 * 左臂 WORK_1
 *
 *      Z = 0.175m
 *      ↓
 *      气泵 ON
 *      ↓
 *      Z = 0.1m
 * ============================================================
 */


/*
 * 左臂：
 *
 *      Z = 0.175m
 *      (3508 不动, 0)
 */
static const float LEFT_Z_0175_DJI_ANGLE_DEG =
    0.0f;

static const float LEFT_Z_0175_ZDRIVE_ANGLE_DEG =
    546.0f;


/*
 * 左臂：
 *
 *      Z = 0.1m
 *      (3508 不动, 0)
 */
static const float LEFT_Z_0100_DJI_ANGLE_DEG =
    0.0f;

static const float LEFT_Z_0100_ZDRIVE_ANGLE_DEG =
    646.0f;


/*
 * ============================================================
 * 右臂 WORK_1
 * ============================================================
 */


/*
 * 右臂：
 *
 *      Z = 0.175m
 */
static const float RIGHT_Z_0175_DJI_ANGLE_DEG =
    0.0f;

static const float RIGHT_Z_0175_ZDRIVE_ANGLE_DEG =
    0.0f;


/*
 * 右臂：
 *
 *      Z = 0.1m
 */
static const float RIGHT_Z_0100_DJI_ANGLE_DEG =
    0.0f;

static const float RIGHT_Z_0100_ZDRIVE_ANGLE_DEG =
    0.0f;


/*
 * ============================================================
 * 左臂 WORK_2
 *
 *      Z = 0.525m
 *      ↓
 *      气泵 OFF
 *      ↓
 *      Z = 0.8m
 * ============================================================
 */


/*
 * 左臂：
 *
 *      Z = 0.525m
 */
static const float LEFT_Z_0525_DJI_ANGLE_DEG =
    43.0f;

static const float LEFT_Z_0525_ZDRIVE_ANGLE_DEG =
    146.0f;


/*
 * 左臂：
 *
 *      Z = 0.8m
 */
static const float LEFT_Z_0800_DJI_ANGLE_DEG =
    88.0f;

static const float LEFT_Z_0800_ZDRIVE_ANGLE_DEG =
    196.0f;


/*
 * ============================================================
 * 右臂 WORK_2
 * ============================================================
 */


/*
 * 右臂：
 *
 *      Z = 0.525m
 */
static const float RIGHT_Z_0525_DJI_ANGLE_DEG =
    0.0f;

static const float RIGHT_Z_0525_ZDRIVE_ANGLE_DEG =
    0.0f;


/*
 * 右臂：
 *
 *      Z = 0.8m
 */
static const float RIGHT_Z_0800_DJI_ANGLE_DEG =
    0.0f;

static const float RIGHT_Z_0800_ZDRIVE_ANGLE_DEG =
    0.0f;


/*
 * ============================================================
 *                    Queue
 * ============================================================
 */

static QueueHandle_t arm_cmd_queue = NULL;


/*
 * ============================================================
 *                    机械臂控制对象
 * ============================================================
 */

static ArmCtrl_t arm_ctrl[ARM_NUM];


/*
 * ============================================================
 *                    总使能
 * ============================================================
 */

static volatile bool arm_enabled = false;


/*
 * ============================================================
 *                    电机反馈 Ready
 * ============================================================
 */

volatile bool arm_left_dji_ready = false;

volatile bool arm_left_zdrive_ready = false;

volatile bool arm_right_dji_ready = false;

volatile bool arm_right_zdrive_ready = false;


/*
 * ============================================================
 *                    气泵控制
 * ============================================================
 *
 * 状态变量(定义在 Arm_StateMachine.h 开头):
 *
 *      arm_left_pump_on  → PA9(左臂)
 *      arm_right_pump_on → PA2(右臂)
 *
 * 用户直接写 0 / 1 控制；
 * 机械臂 WORK 命令开始后自动置 1(PULLDOWN 清 0)。
 *
 * Arm_PumpApply() 每 5ms( Arm_Task 循环 )把状态变量
 * 应用到 GPIO。
 *
 * 注意：
 * PA9 / PA2 也是 USART1_TX / USART2_TX 的复用脚，
 * 所以引脚配置放在 Arm_PumpGpioInit()，
 * 由 Arm_Task 启动时调用(此时所有外设初始化已完成)。
 * ============================================================
 */

volatile bool arm_left_pump_on = false;

volatile bool arm_right_pump_on = false;


/*
 * 初始化气泵 GPIO(PA9 / PA2 输出, 默认关泵)。
 *
 * 必须在任务上下文、所有外设初始化完成后调用，
 * 否则会被 USART MspInit 覆盖。
 */
static void Arm_PumpGpioInit(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio_init.Pin = LEFT_PUMP_Pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LEFT_PUMP_GPIO_Port, &gpio_init);

    gpio_init.Pin = RIGHT_PUMP_Pin;
    HAL_GPIO_Init(RIGHT_PUMP_GPIO_Port, &gpio_init);

    HAL_GPIO_WritePin(LEFT_PUMP_GPIO_Port, LEFT_PUMP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_PUMP_GPIO_Port, RIGHT_PUMP_Pin, GPIO_PIN_RESET);
}


/*
 * 把气泵状态变量应用到引脚(每 5ms)。
 */
static void Arm_PumpApply(void)
{
    HAL_GPIO_WritePin(
        LEFT_PUMP_GPIO_Port,
        LEFT_PUMP_Pin,
        arm_left_pump_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        RIGHT_PUMP_GPIO_Port,
        RIGHT_PUMP_Pin,
        arm_right_pump_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


static void Arm_PumpOn(uint8_t arm_id)
{
    /*
     * 只改状态变量，引脚由 Arm_PumpApply() 周期刷新。
     */
    if (arm_id == ARM_LEFT)
    {
        arm_left_pump_on = true;
    }
    else if (arm_id == ARM_RIGHT)
    {
        arm_right_pump_on = true;
    }
}


static void Arm_PumpOff(uint8_t arm_id)
{
    if (arm_id == ARM_LEFT)
    {
        arm_left_pump_on = false;
    }
    else if (arm_id == ARM_RIGHT)
    {
        arm_right_pump_on = false;
    }
}


/*
 * ============================================================
 *                    获取 DJI 下标
 * ============================================================
 */

static uint8_t Arm_GetDjiIndex(uint8_t arm_id)
{
    if (arm_id == ARM_LEFT)
    {
        return LEFT_DJI_INDEX;
    }

    return RIGHT_DJI_INDEX;
}


/*
 * ============================================================
 *                    获取 Zdrive 下标
 * ============================================================
 */

static uint8_t Arm_GetZdriveIndex(uint8_t arm_id)
{
    if (arm_id == ARM_LEFT)
    {
        return LEFT_ZDRIVE_INDEX;
    }

    return RIGHT_ZDRIVE_INDEX;
}


/*
 * ============================================================
 *                    DJI 设置位置
 * ============================================================
 */

static void Arm_DJI_SetPosition(
    uint8_t arm_id,
    float angle_deg)
{
    uint8_t index;


    index = Arm_GetDjiIndex(arm_id);


    /*
     * 保存目标角度。
     */
    arm_ctrl[arm_id].set_dji_angle_deg =
        angle_deg;


    /*
     * DJI 3508 进入位置模式。
     */
    DJmotor[index].MODE_Set =
        DJ_Position;


    /*
     * 设置绝对目标位置。
     */
    DJmotor[index].valSet.angle_deg =
        angle_deg;
}


/*
 * ============================================================
 *                    Zdrive 设置位置
 * ============================================================
 */

static void Arm_Zdrive_SetPosition(
    uint8_t arm_id,
    float angle_deg)
{
    uint8_t index;


    index = Arm_GetZdriveIndex(arm_id);


    /*
     * 保存目标角度。
     */
    arm_ctrl[arm_id].set_ak80_angle_deg =
        angle_deg;


    /*
     * Zdrive 位置模式。
     *
     * 注意：
     *
     * 你的枚举名字是：
     *
     *      Zdrive_Postion
     *
     * 保持你的工程定义。
     */
    Zmotor[index].mode =
        Zdrive_Postion;


    /*
     * 设置绝对位置。
     */
    Zmotor[index].valSetNow.pos_deg =
        angle_deg;
}


/*
 * ============================================================
 *                    两个电机同时设置位置
 * ============================================================
 */

static void Arm_SetPosition(
    uint8_t arm_id,
    float dji_offset_deg,
    float zdrive_offset_deg)
{
    /*
     * 相对转角模式。
     *
     * 传入的是相对上电位置的转角，
     * 在这里换算成绝对目标：
     *
     *      目标 = 上电位置 + 相对转角
     *
     * AK80 绝对编码器每次上电零点会漂，
     * 所以所有动作都以"上电位置"为基准执行。
     */

    /*
     * CAN1：
     *
     * DJI 3508
     */
    Arm_DJI_SetPosition(
        arm_id,
        arm_ctrl[arm_id].power_on_dji_angle_deg +
            dji_offset_deg);


    /*
     * CAN2：
     *
     * Zdrive AK80
     */
    Arm_Zdrive_SetPosition(
        arm_id,
        arm_ctrl[arm_id].power_on_ak80_angle_deg +
            zdrive_offset_deg);
}


/*
 * ============================================================
 *                    重申目标位置
 * ============================================================
 *
 * 状态机等待电机运动时，
 * 每 5ms 重写一次目标。
 * ============================================================
 */

static void Arm_ReassertPosition(uint8_t arm_id)
{
    uint8_t dji_index;
    uint8_t zdrive_index;


    dji_index =
        Arm_GetDjiIndex(arm_id);

    zdrive_index =
        Arm_GetZdriveIndex(arm_id);


    /*
     * DJI 3508
     */
    DJmotor[dji_index].MODE_Set =
        DJ_Position;

    DJmotor[dji_index].valSet.angle_deg =
        arm_ctrl[arm_id].set_dji_angle_deg;


    /*
     * AK80
     */
    Zmotor[zdrive_index].mode =
        Zdrive_Postion;

    Zmotor[zdrive_index].valSetNow.pos_deg =
        arm_ctrl[arm_id].set_ak80_angle_deg;
}


/*
 * ============================================================
 *                    判断 DJI 到位
 * ============================================================
 */

static bool Arm_DJI_IsAtTarget(uint8_t arm_id)
{
    uint8_t index;

    float error;


    index =
        Arm_GetDjiIndex(arm_id);


    error =
        fabsf(
            DJmotor[index].valNow.angle_deg -
            arm_ctrl[arm_id].set_dji_angle_deg);


    return
        (error <= ARM_POSITION_ERROR_DEG);
}


/*
 * ============================================================
 *                    判断 AK80 到位
 * ============================================================
 */

static bool Arm_Zdrive_IsAtTarget(uint8_t arm_id)
{
    uint8_t index;

    float error;


    index =
        Arm_GetZdriveIndex(arm_id);


    /*
     * 使用 Zdrive 的实际位置反馈 pos_deg。
     *
     * 它由广播 Pur 查询回传更新(实测每次上电能到 1800° 左右，
     * 说明这条反馈是活的)。
     *
     * 不要用 posIn_deg：
     * 它只在驱动回传 PosIn 回声时更新，
     * answer_mode=2(无反馈执行)时基本不动，
     * 会导致到位判断永远不成立。
     */
    error =
        fabsf(
            Zmotor[index].valReal.pos_deg -
            arm_ctrl[arm_id].set_ak80_angle_deg);


    return
        (error <= ARM_POSITION_ERROR_DEG);
}


/*
 * ============================================================
 *                    判断整只机械臂到位
 * ============================================================
 */

static bool Arm_IsAtTarget(uint8_t arm_id)
{
    bool dji_ok;
    bool zdrive_ok;

    /*
     * 没收到反馈的电机跳过到位判断，按"已到位"处理。
     *
     * 否则 AK80 不上线时，到位条件永远不成立，
     * 机械臂动作(包括开气泵)永远不触发。
     *
     * 电机反馈正常后 ready 置位，这里自动恢复严格判断。
     */
    if (arm_id == ARM_LEFT)
    {
        dji_ok =
            arm_left_dji_ready ?
            Arm_DJI_IsAtTarget(ARM_LEFT) :
            true;

        zdrive_ok =
            arm_left_zdrive_ready ?
            Arm_Zdrive_IsAtTarget(ARM_LEFT) :
            true;
    }
    else
    {
        dji_ok =
            arm_right_dji_ready ?
            Arm_DJI_IsAtTarget(ARM_RIGHT) :
            true;

        zdrive_ok =
            arm_right_zdrive_ready ?
            Arm_Zdrive_IsAtTarget(ARM_RIGHT) :
            true;
    }

    return dji_ok && zdrive_ok;
}


/*
 * ============================================================
 *                    记录上电位置
 * ============================================================
 *
 * Arm_Task 启动以后：
 *
 *      等待四个电机收到反馈
 *              ↓
 *      记录当前位置
 *
 * 以后 RESET：
 *
 *      直接回这里。
 *
 * ============================================================
 */

static void Arm_RecordPowerOnPosition(void)
{
    /*
     * 左臂 DJI 3508
     */
    arm_ctrl[ARM_LEFT]
        .power_on_dji_angle_deg =
        DJmotor[LEFT_DJI_INDEX]
        .valNow.angle_deg;


    /*
     * 左臂 AK80
     */
    arm_ctrl[ARM_LEFT]
        .power_on_ak80_angle_deg =
        Zmotor[LEFT_ZDRIVE_INDEX]
        .valReal.posIn_deg;


    /*
     * 右臂 DJI 3508
     */
    arm_ctrl[ARM_RIGHT]
        .power_on_dji_angle_deg =
        DJmotor[RIGHT_DJI_INDEX]
        .valNow.angle_deg;


    /*
     * 右臂 AK80
     */
    arm_ctrl[ARM_RIGHT]
        .power_on_ak80_angle_deg =
        Zmotor[RIGHT_ZDRIVE_INDEX]
        .valReal.posIn_deg;
}


/*
 * ============================================================
 *                    RESET 设置目标
 * ============================================================
 */

static void Arm_StartReset(uint8_t arm_id)
{
    /*
     * 相对转角 = 0：
     *
     * Arm_SetPosition 内部会加上上电位置基准，
     * 所以这里直接传 0，0 就是"回到上电位置"。
     */
    Arm_SetPosition(
        arm_id,
        0.0f,
        0.0f);
}


/*
 * ============================================================
 *                    反馈
 * ============================================================
 */

static void Arm_FeedbackSend(
    uint8_t opcode,
    uint8_t data0,
    uint8_t data1)
{
    CAN_TxHeaderTypeDef TxHeader = {0};

    uint8_t TxData[2];

    uint32_t TxMailbox;


    /*
     * 从板 -> 主控：
     *
     *      0x050101xx
     */
    TxHeader.ExtId =
        ARM_FB_BASE_ID |
        (uint32_t)opcode;


    TxHeader.IDE =
        CAN_ID_EXT;

    TxHeader.RTR =
        CAN_RTR_DATA;

    TxHeader.DLC =
        2U;

    TxHeader.TransmitGlobalTime =
        DISABLE;


    TxData[0] =
        data0;

    TxData[1] =
        data1;


    HAL_CAN_AddTxMessage(
        &hcan1,
        &TxHeader,
        TxData,
        &TxMailbox);
}


/*
 * ============================================================
 *                    命令拒绝
 * ============================================================
 */

static void Arm_Reject(ArmCommand_t cmd)
{
    Arm_FeedbackSend(
        (uint8_t)cmd,
        ARM_FB_REJECTED,
        (uint8_t)cmd);
}


/*
 * ============================================================
 *                    开始动作
 * ============================================================
 */

static bool Arm_BeginAction(
    uint8_t arm_id,
    ArmState_t state,
    ArmCommand_t cmd)
{
    /*
     * 未使能：
     *
     *      禁止执行。
     */
    if (!arm_enabled)
    {
        Arm_Reject(cmd);

        return false;
    }


    /*
     * Busy：
     *
     *      禁止新动作。
     */
    if (arm_ctrl[arm_id].busy)
    {
        Arm_Reject(cmd);

        return false;
    }


    /*
     * 设置状态。
     */
    arm_ctrl[arm_id].state =
        state;


    /*
     * 每个动作都从 SET_POSITION 开始。
     */
    arm_ctrl[arm_id].step =
        ARM_STEP_SET_POSITION;


    /*
     * 加 Busy 锁。
     */
    arm_ctrl[arm_id].busy =
        true;


    /*
     * 保存命令。
     */
    arm_ctrl[arm_id].last_cmd =
        cmd;


    /*
     * 通知主控：
     *
     * 已接受。
     */
    Arm_FeedbackSend(
        (uint8_t)cmd,
        ARM_FB_STARTED,
        (uint8_t)cmd);


    /*
     * 机械臂开始工作(WORK)：
     *
     * 气泵直接置 1 并保持，
     * 直到 PULLDOWN 才关。
     *
     * RESET 不动气泵。
     */
    if (
        (cmd == ARM_CMD_LEFT_WORK_1) ||
        (cmd == ARM_CMD_LEFT_WORK_2) ||
        (cmd == ARM_CMD_RIGHT_WORK_1) ||
        (cmd == ARM_CMD_RIGHT_WORK_2))
    {
        Arm_PumpOn(arm_id);
    }


    return true;
}


/*
 * ============================================================
 *                    动作完成
 * ============================================================
 */

static void Arm_Finish(uint8_t arm_id)
{
    ArmCommand_t cmd;


    cmd =
        arm_ctrl[arm_id].last_cmd;


    /*
     * 回到 IDLE。
     */
    arm_ctrl[arm_id].state =
        ARM_STATE_IDLE;

    arm_ctrl[arm_id].step =
        ARM_STEP_IDLE;


    /*
     * 解 Busy。
     */
    arm_ctrl[arm_id].busy =
        false;


    /*
     * 通知主控：
     *
     * 动作完成。
     */
    Arm_FeedbackSend(
        (uint8_t)cmd,
        ARM_FB_FINISHED,
        (uint8_t)cmd);
}


/*
 * ============================================================
 *                    全部失能
 * ============================================================
 */

static void Arm_DisableAll(void)
{
    uint8_t i;


    /*
     * ========================================================
     * DJI
     * ========================================================
     */

    for (i = 0U; i < USE_DJNUM; i++)
    {
        DJmotor[i].MODE_Set =
            DJ_Disable;

        DJmotor[i].valSet.current_raw =
            0;
    }


    /*
     * ========================================================
     * Zdrive
     * ========================================================
     */

    for (i = 0U; i < USE_ZDRIVE_NUM; i++)
    {
        Zmotor[i].mode =
            Zdrive_Disable;
    }


    /*
     * 关闭气泵。
     */
    Arm_PumpOff(ARM_LEFT);

    Arm_PumpOff(ARM_RIGHT);


    /*
     * ========================================================
     * 左臂
     * ========================================================
     */

    arm_ctrl[ARM_LEFT].state =
        ARM_STATE_IDLE;

    arm_ctrl[ARM_LEFT].step =
        ARM_STEP_IDLE;

    arm_ctrl[ARM_LEFT].busy =
        false;

    arm_ctrl[ARM_LEFT].work_stage =
        ARM_WORK_IDLE;


    /*
     * ========================================================
     * 右臂
     * ========================================================
     */

    arm_ctrl[ARM_RIGHT].state =
        ARM_STATE_IDLE;

    arm_ctrl[ARM_RIGHT].step =
        ARM_STEP_IDLE;

    arm_ctrl[ARM_RIGHT].busy =
        false;

    arm_ctrl[ARM_RIGHT].work_stage =
        ARM_WORK_IDLE;
}


/*
 * ============================================================
 *                    命令处理
 * ============================================================
 */

static void Arm_ProcessCommand(
    const ArmCommandMsg_t *msg)
{
    switch (msg->cmd)
    {
        /*
         * ====================================================
         * 自检
         * ====================================================
         */

    case ARM_CMD_SELF_CHECK:
    {
        uint8_t state = 0U;
        uint8_t status = 0U;


        /*
         * state：
         *
         *      0 = 正常空闲
         *      1 = Busy
         *      2 = 电机反馈未准备好
         */
        if (arm_ctrl[ARM_LEFT].busy ||
            arm_ctrl[ARM_RIGHT].busy)
        {
            state = 1U;
        }
        else if (
            !(arm_left_dji_ready &&
              arm_left_zdrive_ready &&
              arm_right_dji_ready &&
              arm_right_zdrive_ready))
        {
            state = 2U;
        }


        /*
         * bit0：
         *
         *      总使能
         */
        if (arm_enabled)
        {
            status |= 0x01U;
        }


        /*
         * bit1：
         *
         *      四个电机都有反馈
         */
        if (
            arm_left_dji_ready &&
            arm_left_zdrive_ready &&
            arm_right_dji_ready &&
            arm_right_zdrive_ready)
        {
            status |= 0x02U;
        }


        Arm_FeedbackSend(
            ARM_CMD_SELF_CHECK,
            state,
            status);

        break;
    }


        /*
         * ====================================================
         * ENABLE
         * ====================================================
         */

    case ARM_CMD_ENABLE:

        if (msg->data0 != 0U)
        {
            /*
             * 允许动作。
             */
            arm_enabled =
                true;
        }
        else
        {
            /*
             * 失能。
             */
            arm_enabled =
                false;

            Arm_DisableAll();
        }


        Arm_FeedbackSend(
            ARM_CMD_ENABLE,
            arm_enabled ? 1U : 0U,
            0U);

        break;


        /*
         * ====================================================
         * RESET
         * ====================================================
         *
         * data0：
         *
         *      0 = 双臂
         *      1 = 左臂
         *      2 = 右臂
         *
         * 注意：
         *
         * RESET 不清 work_stage。
         *
         * 所以：
         *
         *      WORK_1
         *          ↓
         *      HAS_BLOCK
         *          ↓
         *      RESET
         *          ↓
         *      HAS_BLOCK
         *
         * ====================================================
         */

    case ARM_CMD_RESET:

        if (msg->data0 > 2U)
        {
            Arm_Reject(
                ARM_CMD_RESET);

            break;
        }


        /*
         * 左臂。
         */
        if (
            (msg->data0 == 0U) ||
            (msg->data0 == 1U))
        {
            Arm_BeginAction(
                ARM_LEFT,
                ARM_STATE_RESET,
                ARM_CMD_RESET);
        }


        /*
         * 右臂。
         */
        if (
            (msg->data0 == 0U) ||
            (msg->data0 == 2U))
        {
            Arm_BeginAction(
                ARM_RIGHT,
                ARM_STATE_RESET,
                ARM_CMD_RESET);
        }

        break;


        /*
         * ====================================================
         * 左 WORK_1
         * ====================================================
         */

    case ARM_CMD_LEFT_WORK_1:

        /*
         * 已经持有方块：
         *
         *      不允许再次取块。
         */
        if (
            arm_ctrl[ARM_LEFT].work_stage
            != ARM_WORK_IDLE)
        {
            Arm_Reject(
                ARM_CMD_LEFT_WORK_1);

            break;
        }


        Arm_BeginAction(
            ARM_LEFT,
            ARM_STATE_WORK_1,
            ARM_CMD_LEFT_WORK_1);

        break;


        /*
         * ====================================================
         * 左 WORK_2
         * ====================================================
         */

    case ARM_CMD_LEFT_WORK_2:

        /*
         * 必须已经持有方块。
         */
        if (
            arm_ctrl[ARM_LEFT].work_stage
            != ARM_WORK_HAS_BLOCK)
        {
            Arm_Reject(
                ARM_CMD_LEFT_WORK_2);

            break;
        }


        Arm_BeginAction(
            ARM_LEFT,
            ARM_STATE_WORK_2,
            ARM_CMD_LEFT_WORK_2);

        break;


        /*
         * ====================================================
         * 右 WORK_1
         * ====================================================
         */

    case ARM_CMD_RIGHT_WORK_1:

        if (
            arm_ctrl[ARM_RIGHT].work_stage
            != ARM_WORK_IDLE)
        {
            Arm_Reject(
                ARM_CMD_RIGHT_WORK_1);

            break;
        }


        Arm_BeginAction(
            ARM_RIGHT,
            ARM_STATE_WORK_1,
            ARM_CMD_RIGHT_WORK_1);

        break;


        /*
         * ====================================================
         * 右 WORK_2
         * ====================================================
         */

    case ARM_CMD_RIGHT_WORK_2:

        if (
            arm_ctrl[ARM_RIGHT].work_stage
            != ARM_WORK_HAS_BLOCK)
        {
            Arm_Reject(
                ARM_CMD_RIGHT_WORK_2);

            break;
        }


        Arm_BeginAction(
            ARM_RIGHT,
            ARM_STATE_WORK_2,
            ARM_CMD_RIGHT_WORK_2);

        break;


        /*
         * ====================================================
         * 左 PULLDOWN
         * ====================================================
         */

    case ARM_CMD_LEFT_PULLDOWN:

        /*
         * 这里直接释放气泵。
         *
         * 如果机械臂当前正在运动，
         * Busy 会拒绝。
         */
        if (arm_ctrl[ARM_LEFT].busy)
        {
            Arm_Reject(
                ARM_CMD_LEFT_PULLDOWN);

            break;
        }


        arm_ctrl[ARM_LEFT].state =
            ARM_STATE_PULLDOWN;

        arm_ctrl[ARM_LEFT].step =
            ARM_STEP_PUMP_OFF;

        arm_ctrl[ARM_LEFT].busy =
            true;

        arm_ctrl[ARM_LEFT].last_cmd =
            ARM_CMD_LEFT_PULLDOWN;

        Arm_FeedbackSend(
            ARM_CMD_LEFT_PULLDOWN,
            ARM_FB_STARTED,
            ARM_CMD_LEFT_PULLDOWN);

        break;


        /*
         * ====================================================
         * 右 PULLDOWN
         * ====================================================
         */

    case ARM_CMD_RIGHT_PULLDOWN:

        if (arm_ctrl[ARM_RIGHT].busy)
        {
            Arm_Reject(
                ARM_CMD_RIGHT_PULLDOWN);

            break;
        }


        arm_ctrl[ARM_RIGHT].state =
            ARM_STATE_PULLDOWN;

        arm_ctrl[ARM_RIGHT].step =
            ARM_STEP_PUMP_OFF;

        arm_ctrl[ARM_RIGHT].busy =
            true;

        arm_ctrl[ARM_RIGHT].last_cmd =
            ARM_CMD_RIGHT_PULLDOWN;

        Arm_FeedbackSend(
            ARM_CMD_RIGHT_PULLDOWN,
            ARM_FB_STARTED,
            ARM_CMD_RIGHT_PULLDOWN);

        break;


        /*
         * ====================================================
         * ERROR
         * ====================================================
         */

    case ARM_CMD_ERROR:

        Arm_FeedbackSend(
            ARM_CMD_ERROR,
            0U,
            ARM_DEVICE_ID);

        break;


    default:

        break;
    }
}


/*
 * ============================================================
 *                    左机械臂状态机
 * ============================================================
 */

static void Arm_LeftStateMachine(void)
{
    switch (arm_ctrl[ARM_LEFT].state)
    {
        /*
         * ====================================================
         * IDLE
         * ====================================================
         */

    case ARM_STATE_IDLE:

        arm_ctrl[ARM_LEFT].busy =
            false;

        break;


        /*
         * ====================================================
         * RESET
         * ====================================================
         */

    case ARM_STATE_RESET:

        switch (arm_ctrl[ARM_LEFT].step)
        {
        case ARM_STEP_SET_POSITION:

            /*
             * 回到上电位置。
             */
            Arm_StartReset(
                ARM_LEFT);


            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_WAIT_POSITION:

            /*
             * 每周期重申目标。
             */
            Arm_ReassertPosition(
                ARM_LEFT);


            if (Arm_IsAtTarget(ARM_LEFT))
            {
                arm_ctrl[ARM_LEFT].step =
                    ARM_STEP_FINISH;
            }

            break;


        case ARM_STEP_FINISH:

            /*
             * RESET 完成。
             *
             * !!! 注意 !!!
             *
             * 不修改：
             *
             *      work_stage
             *
             * 如果之前拿着块，
             * RESET 后仍然是 HAS_BLOCK。
             */
            Arm_Finish(
                ARM_LEFT);

            break;


        default:

            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_IDLE;

            break;
        }

        break;


        /*
         * ====================================================
         * PULLDOWN
         * ====================================================
         */

    case ARM_STATE_PULLDOWN:

        switch (arm_ctrl[ARM_LEFT].step)
        {
        case ARM_STEP_PUMP_OFF:

            /*
             * 关闭气泵。
             */
            Arm_PumpOff(
                ARM_LEFT);


            /*
             * 既然主动释放气泵，
             * 就认为方块已经释放。
             */
            arm_ctrl[ARM_LEFT].work_stage =
                ARM_WORK_IDLE;


            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_FINISH;

            break;


        case ARM_STEP_FINISH:

            Arm_Finish(
                ARM_LEFT);

            break;


        default:

            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_IDLE;

            break;
        }

        break;


        /*
         * ====================================================
         * WORK_1
         *
         *      Z=0.175
         *          ↓
         *      气泵 ON
         *          ↓
         *      Z=0.1
         *          ↓
         *      HAS_BLOCK
         * ====================================================
         */

    case ARM_STATE_WORK_1:

        switch (arm_ctrl[ARM_LEFT].step)
        {
        case ARM_STEP_SET_POSITION:

            /*
             * 第一步：
             *
             *      Z = 0.175m
             */
            Arm_SetPosition(
                ARM_LEFT,

                LEFT_Z_0175_DJI_ANGLE_DEG,

                LEFT_Z_0175_ZDRIVE_ANGLE_DEG);


            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_WAIT_POSITION:

            /*
             * 判断当前是哪一个目标。
             *
             * WORK_1 中：
             *
             * 第一次目标 = 0.175
             * 第二次目标 = 0.1
             *
             * 通过 step 切换。
             */
            Arm_ReassertPosition(
                ARM_LEFT);


            if (Arm_IsAtTarget(ARM_LEFT))
            {
                /*
                 * 如果当前还是第一阶段，
                 * 下一步开泵。
                 */
                if (
                    arm_ctrl[ARM_LEFT]
                    .work_stage
                    == ARM_WORK_IDLE)
                {
                    arm_ctrl[ARM_LEFT].step =
                        ARM_STEP_PUMP_ON;
                }
                else
                {
                    /*
                     * 已经开泵并完成第二次下降。
                     *
                     * 取块完成。
                     */
                    arm_ctrl[ARM_LEFT].step =
                        ARM_STEP_FINISH;
                }
            }

            break;


        case ARM_STEP_PUMP_ON:

            /*
             * 开气泵。
             */
            Arm_PumpOn(
                ARM_LEFT);


            /*
             * 立刻把目标改成：
             *
             *      Z = 0.1m
             */
            Arm_SetPosition(
                ARM_LEFT,

                LEFT_Z_0100_DJI_ANGLE_DEG,

                LEFT_Z_0100_ZDRIVE_ANGLE_DEG);


            /*
             * 用 work_stage 表示：
             *
             * 已经开泵，
             * 正在向 0.1m 运动。
             *
             * 这里暂时先设 HAS_BLOCK，
             * 因为气泵已经开启。
             *
             * 真正动作完成是在下一次
             * WAIT_POSITION 到位以后。
             */
            arm_ctrl[ARM_LEFT].work_stage =
                ARM_WORK_HAS_BLOCK;


            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_FINISH:

            /*
             * WORK_1 完成。
             *
             * 当前已经：
             *
             *      开泵
             *      到达 0.1m
             *      持有方块
             */
            arm_ctrl[ARM_LEFT].work_stage =
                ARM_WORK_HAS_BLOCK;


            Arm_Finish(
                ARM_LEFT);

            break;


        default:

            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_IDLE;

            break;
        }

        break;


        /*
         * ====================================================
         * WORK_2
         *
         *      Z=0.525
         *          ↓
         *      气泵 OFF
         *          ↓
         *      Z=0.8
         *          ↓
         *      放块完成
         * ====================================================
         */

    case ARM_STATE_WORK_2:

        switch (arm_ctrl[ARM_LEFT].step)
        {
        case ARM_STEP_SET_POSITION:

            /*
             * 第一步：
             *
             *      Z = 0.525m
             */
            Arm_SetPosition(
                ARM_LEFT,

                LEFT_Z_0525_DJI_ANGLE_DEG,

                LEFT_Z_0525_ZDRIVE_ANGLE_DEG);


            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_WAIT_POSITION:

            Arm_ReassertPosition(
                ARM_LEFT);


            if (Arm_IsAtTarget(ARM_LEFT))
            {
                /*
                 * 到达放块高度。
                 */
                arm_ctrl[ARM_LEFT].step =
                    ARM_STEP_PUMP_OFF;
            }

            break;


        case ARM_STEP_PUMP_OFF:

            /*
             * 释放方块。
             */
            Arm_PumpOff(
                ARM_LEFT);


            /*
             * 方块已经释放。
             */
            arm_ctrl[ARM_LEFT].work_stage =
                ARM_WORK_IDLE;


            /*
             * 第二阶段：
             *
             *      Z = 0.8m
             */
            Arm_SetPosition(
                ARM_LEFT,

                LEFT_Z_0800_DJI_ANGLE_DEG,

                LEFT_Z_0800_ZDRIVE_ANGLE_DEG);


            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_FINISH:

            /*
             * 到达 0.8m。
             *
             * WORK_2 完成。
             */
            Arm_Finish(
                ARM_LEFT);

            break;


        default:

            arm_ctrl[ARM_LEFT].step =
                ARM_STEP_IDLE;

            break;
        }

        break;


    default:

        arm_ctrl[ARM_LEFT].state =
            ARM_STATE_IDLE;

        arm_ctrl[ARM_LEFT].step =
            ARM_STEP_IDLE;

        arm_ctrl[ARM_LEFT].busy =
            false;

        break;
    }
}


/*
 * ============================================================
 *                    右机械臂状态机
 * ============================================================
 */

static void Arm_RightStateMachine(void)
{
    switch (arm_ctrl[ARM_RIGHT].state)
    {
        /*
         * ====================================================
         * IDLE
         * ====================================================
         */

    case ARM_STATE_IDLE:

        arm_ctrl[ARM_RIGHT].busy =
            false;

        break;


        /*
         * ====================================================
         * RESET
         * ====================================================
         */

    case ARM_STATE_RESET:

        switch (arm_ctrl[ARM_RIGHT].step)
        {
        case ARM_STEP_SET_POSITION:

            Arm_StartReset(
                ARM_RIGHT);


            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_WAIT_POSITION:

            Arm_ReassertPosition(
                ARM_RIGHT);


            if (Arm_IsAtTarget(ARM_RIGHT))
            {
                arm_ctrl[ARM_RIGHT].step =
                    ARM_STEP_FINISH;
            }

            break;


        case ARM_STEP_FINISH:

            /*
             * RESET 不清 work_stage。
             */
            Arm_Finish(
                ARM_RIGHT);

            break;


        default:

            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_IDLE;

            break;
        }

        break;


        /*
         * ====================================================
         * PULLDOWN
         * ====================================================
         */

    case ARM_STATE_PULLDOWN:

        switch (arm_ctrl[ARM_RIGHT].step)
        {
        case ARM_STEP_PUMP_OFF:

            Arm_PumpOff(
                ARM_RIGHT);


            /*
             * 释放以后认为没有方块。
             */
            arm_ctrl[ARM_RIGHT].work_stage =
                ARM_WORK_IDLE;


            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_FINISH;

            break;


        case ARM_STEP_FINISH:

            Arm_Finish(
                ARM_RIGHT);

            break;


        default:

            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_IDLE;

            break;
        }

        break;


        /*
         * ====================================================
         * WORK_1
         * ====================================================
         */

    case ARM_STATE_WORK_1:

        switch (arm_ctrl[ARM_RIGHT].step)
        {
        case ARM_STEP_SET_POSITION:

            /*
             *      Z = 0.175m
             */
            Arm_SetPosition(
                ARM_RIGHT,

                RIGHT_Z_0175_DJI_ANGLE_DEG,

                RIGHT_Z_0175_ZDRIVE_ANGLE_DEG);


            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_WAIT_POSITION:

            Arm_ReassertPosition(
                ARM_RIGHT);


            if (Arm_IsAtTarget(ARM_RIGHT))
            {
                if (
                    arm_ctrl[ARM_RIGHT]
                    .work_stage
                    == ARM_WORK_IDLE)
                {
                    /*
                     * 到 0.175m。
                     *
                     * 下一步开泵。
                     */
                    arm_ctrl[ARM_RIGHT].step =
                        ARM_STEP_PUMP_ON;
                }
                else
                {
                    /*
                     * 第二次下降完成。
                     */
                    arm_ctrl[ARM_RIGHT].step =
                        ARM_STEP_FINISH;
                }
            }

            break;


        case ARM_STEP_PUMP_ON:

            /*
             * 开气泵。
             */
            Arm_PumpOn(
                ARM_RIGHT);


            /*
             *      Z = 0.1m
             */
            Arm_SetPosition(
                ARM_RIGHT,

                RIGHT_Z_0100_DJI_ANGLE_DEG,

                RIGHT_Z_0100_ZDRIVE_ANGLE_DEG);


            /*
             * 表示已经开始取块。
             */
            arm_ctrl[ARM_RIGHT].work_stage =
                ARM_WORK_HAS_BLOCK;


            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_FINISH:

            /*
             * WORK_1 完成。
             */
            arm_ctrl[ARM_RIGHT].work_stage =
                ARM_WORK_HAS_BLOCK;


            Arm_Finish(
                ARM_RIGHT);

            break;


        default:

            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_IDLE;

            break;
        }

        break;


        /*
         * ====================================================
         * WORK_2
         * ====================================================
         */

    case ARM_STATE_WORK_2:

        switch (arm_ctrl[ARM_RIGHT].step)
        {
        case ARM_STEP_SET_POSITION:

            /*
             *      Z = 0.525m
             */
            Arm_SetPosition(
                ARM_RIGHT,

                RIGHT_Z_0525_DJI_ANGLE_DEG,

                RIGHT_Z_0525_ZDRIVE_ANGLE_DEG);


            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_WAIT_POSITION:

            Arm_ReassertPosition(
                ARM_RIGHT);


            if (Arm_IsAtTarget(ARM_RIGHT))
            {
                /*
                 * 到达放块位置。
                 */
                arm_ctrl[ARM_RIGHT].step =
                    ARM_STEP_PUMP_OFF;
            }

            break;


        case ARM_STEP_PUMP_OFF:

            /*
             * 释放方块。
             */
            Arm_PumpOff(
                ARM_RIGHT);


            /*
             * 已经没有方块。
             */
            arm_ctrl[ARM_RIGHT].work_stage =
                ARM_WORK_IDLE;


            /*
             *      Z = 0.8m
             */
            Arm_SetPosition(
                ARM_RIGHT,

                RIGHT_Z_0800_DJI_ANGLE_DEG,

                RIGHT_Z_0800_ZDRIVE_ANGLE_DEG);


            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_WAIT_POSITION;

            break;


        case ARM_STEP_FINISH:

            /*
             * WORK_2 完成。
             */
            Arm_Finish(
                ARM_RIGHT);

            break;


        default:

            arm_ctrl[ARM_RIGHT].step =
                ARM_STEP_IDLE;

            break;
        }

        break;


    default:

        arm_ctrl[ARM_RIGHT].state =
            ARM_STATE_IDLE;

        arm_ctrl[ARM_RIGHT].step =
            ARM_STEP_IDLE;

        arm_ctrl[ARM_RIGHT].busy =
            false;

        break;
    }
}


/*
 * ============================================================
 *                    Arm_Task
 * ============================================================
 *
 * 整个机械臂状态机运行在 FreeRTOS。
 *
 *
 * CAN 中断：
 *
 *      CAN RX
 *        ↓
 *      Arm_SendCommandFromISR()
 *        ↓
 *      Queue
 *
 *
 * Arm_Task：
 *
 *      Queue
 *        ↓
 *      Arm_ProcessCommand()
 *        ↓
 *      左状态机
 *      右状态机
 *
 *
 * main while 不参与机械臂控制。
 *
 * ============================================================
 */

void Arm_Task(void *argument)
{
    ArmCommandMsg_t command;


    (void)argument;


    /*
     * ========================================================
     * 初始化控制结构
     * ========================================================
     */

    memset(
        arm_ctrl,
        0,
        sizeof(arm_ctrl));


    /*
     * 初始化气泵 GPIO。
     *
     * 必须在这里做：
     * PA9/PA2 同时是 USART1_TX / USART2_TX 复用脚，
     * 任务启动时所有外设初始化已完成，
     * 这里配置成普通输出才不会被覆盖。
     */
    Arm_PumpGpioInit();


    arm_ctrl[ARM_LEFT].state =
        ARM_STATE_IDLE;

    arm_ctrl[ARM_LEFT].step =
        ARM_STEP_IDLE;

    arm_ctrl[ARM_LEFT].busy =
        false;

    arm_ctrl[ARM_LEFT].work_stage =
        ARM_WORK_IDLE;


    arm_ctrl[ARM_RIGHT].state =
        ARM_STATE_IDLE;

    arm_ctrl[ARM_RIGHT].step =
        ARM_STEP_IDLE;

    arm_ctrl[ARM_RIGHT].busy =
        false;

    arm_ctrl[ARM_RIGHT].work_stage =
        ARM_WORK_IDLE;


    /*
     * ========================================================
     * 记录上电位置
     * ========================================================
     *
     * 不再阻塞等待四电机反馈。
     *
     * 原因：
     *
     * 某个电机(比如 AK80)没上电/没接线时，
     * 反馈标志永远不置位，
     * 不能让它把整个任务卡死。
     *
     * 没收到反馈的电机:
     *   - 位置记录为 0(不会驱动它)；
     *   - 到位判断直接跳过(见 Arm_IsAtTarget)，
     *     等其他电机到位就继续动作；
     *   - 等它真正上线后 ready 置位，
     *     到位判断自动恢复严格检查。
     * ========================================================
     */

    Arm_RecordPowerOnPosition();


    /*
     * ========================================================
     * 正式运行
     * ========================================================
     */

    while (1)
    {
        /*
         * ----------------------------------------------------
         * 处理 Queue 命令
         * ----------------------------------------------------
         */

        while (
            xQueueReceive(
                arm_cmd_queue,
                &command,
                0U) == pdPASS)
        {
            Arm_ProcessCommand(
                &command);
        }


        /*
         * ----------------------------------------------------
         * 左机械臂
         * ----------------------------------------------------
         */

        if (arm_ctrl[ARM_LEFT].busy)
        {
            Arm_LeftStateMachine();
        }


        /*
         * ----------------------------------------------------
         * 右机械臂
         * ----------------------------------------------------
         */

        if (arm_ctrl[ARM_RIGHT].busy)
        {
            Arm_RightStateMachine();
        }


        /*
         * ----------------------------------------------------
         * 气泵状态变量 → 引脚
         * ----------------------------------------------------
         *
         * 每周期把 arm_left_pump_on / arm_right_pump_on
         * 应用到 PA9 / PA2。
         */
        Arm_PumpApply();


        /*
         * ----------------------------------------------------
         * 下一周期
         * ----------------------------------------------------
         */

        vTaskDelay(
            pdMS_TO_TICKS(
                ARM_TASK_PERIOD_MS));
    }
}


/*
 * ============================================================
 *                    初始化
 * ============================================================
 */

void Arm_StateMachine_Init(void)
{
    /*
     * 创建命令 Queue。
     */
    arm_cmd_queue =
        xQueueCreate(
            ARM_QUEUE_LENGTH,
            sizeof(ArmCommandMsg_t));


    if (arm_cmd_queue == NULL)
    {
        Error_Handler();
    }


    /*
     * 上电默认失能。
     */
    arm_enabled =
        false;
}


/*
 * ============================================================
 *                    普通任务发送命令
 * ============================================================
 */

bool Arm_SendCommand(
    ArmCommand_t command,
    uint8_t data0,
    uint8_t data1)
{
    ArmCommandMsg_t msg;


    if (arm_cmd_queue == NULL)
    {
        return false;
    }


    msg.cmd =
        command;

    msg.data0 =
        data0;

    msg.data1 =
        data1;


    return
        xQueueSend(
            arm_cmd_queue,
            &msg,
            0U) == pdPASS;
}


/*
 * ============================================================
 *                    ISR 发送命令
 * ============================================================
 *
 * 在 CAN 接收中断里调用。
 * ============================================================
 */

bool Arm_SendCommandFromISR(
    ArmCommand_t command,
    uint8_t data0,
    uint8_t data1)
{
    ArmCommandMsg_t msg;

    BaseType_t xHigherPriorityTaskWoken =
        pdFALSE;

    BaseType_t result;


    if (arm_cmd_queue == NULL)
    {
        return false;
    }


    msg.cmd =
        command;

    msg.data0 =
        data0;

    msg.data1 =
        data1;


    result =
        xQueueSendFromISR(
            arm_cmd_queue,
            &msg,
            &xHigherPriorityTaskWoken);


    portYIELD_FROM_ISR(
        xHigherPriorityTaskWoken);


    return
        result == pdPASS;
}


/*
 * ============================================================
 *                    DJI 反馈通知
 * ============================================================
 *
 * 在 CAN1 接收代码中调用。
 * ============================================================
 */

void Arm_NotifyDjiFeedback(
    uint8_t motor_index)
{
    /*
     * 防止数组越界。
     */
    if (motor_index >= USE_DJNUM)
    {
        return;
    }


    /*
     * 收到有效反馈。
     *
     * 允许该电机参与控制。
     */
    DJmotor[motor_index].Begin =
        true;


    /*
     * 左臂 DJI。
     */
    if (motor_index == LEFT_DJI_INDEX)
    {
        arm_left_dji_ready =
            true;
    }


    /*
     * 右臂 DJI。
     */
    else if (motor_index == RIGHT_DJI_INDEX)
    {
        arm_right_dji_ready =
            true;
    }
}


/*
 * ============================================================
 *                    Zdrive 反馈通知
 * ============================================================
 *
 * 在 CAN2 接收代码中调用。
 * ============================================================
 */

void Arm_NotifyZdriveFeedback(
    uint8_t motor_index)
{
    /*
     * 防止数组越界。
     */
    if (motor_index >= USE_ZDRIVE_NUM)
    {
        return;
    }


    /*
     * 收到有效反馈以后，
     * 允许 Zdrive 参与控制。
     */
    Zmotor[motor_index].Begin =
        true;


    /*
     * 左臂 AK80。
     */
    if (motor_index == LEFT_ZDRIVE_INDEX)
    {
        arm_left_zdrive_ready =
            true;
    }


    /*
     * 右臂 AK80。
     */
    else if (motor_index == RIGHT_ZDRIVE_INDEX)
    {
        arm_right_zdrive_ready =
            true;
    }
}


/*
 * ============================================================
 *                    Busy 查询
 * ============================================================
 */

bool Arm_IsBusy(uint8_t arm_id)
{
    if (arm_id >= ARM_NUM)
    {
        return false;
    }

    return
        arm_ctrl[arm_id].busy;
}


/*
 * ============================================================
 *                    状态查询
 * ============================================================
 */

ArmState_t Arm_GetState(uint8_t arm_id)
{
    if (arm_id >= ARM_NUM)
    {
        return ARM_STATE_IDLE;
    }

    return
        arm_ctrl[arm_id].state;
}