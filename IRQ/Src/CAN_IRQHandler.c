/*
 * @Author: Frt001 2067314783@qq.com
 * @Date: 2026-08-13 10:00:13
 * @LastEditors: Frt001 2067314783@qq.com
 * @LastEditTime: 2026-08-25 16:43:21
 * @FilePath: \f4_show\IRQ\Src\CAN_IRQHandler.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "CAN_IRQHandler.h"
#include "DJmotor.h"
#include "ZDrive.h"
#include "Arm_StateMachine.h"


/*
 * 主板动作命令解析(协议.md)。
 *
 * 主控 → 从板:
 *
 *     CAN1, 1Mbps, 扩展帧
 *     ExtId = 0x010105xx(xx = 操作码)
 *     DLC   = 2, data = [data0][data1]
 *
 * 操作码:
 *     0x00 主控自检广播
 *     0x01 使能/失能
 *     0x03 左臂 Work1 / 0x04 左臂 Work2
 *     0x05 右臂 Work1 / 0x06 右臂 Work2
 *     0x07 左臂释放 / 0x08 右臂释放(从板自定义)
 *     0xEE 错误警报(待定)
 *     0xFF 复位
 *
 * 收到后把 [命令][data0][data1] 扔进机械臂状态机命令队列。
 */
static void Arm_CAN_CommandParse(
    const CAN_RxHeaderTypeDef *RxHeader,
    const uint8_t *RxData)
{
    uint8_t opcode;

    /*
     * 只认扩展帧、数据帧、
     * ID 模板 0x010105xx。
     */
    if (RxHeader->IDE != CAN_ID_EXT)
    {
        return;
    }

    if (RxHeader->RTR != CAN_RTR_DATA)
    {
        return;
    }

    if ((uint32_t)(RxHeader->ExtId >> 8U) !=
        (uint32_t)(ARM_CMD_BASE_ID >> 8U))
    {
        return;
    }

    opcode = (uint8_t)(RxHeader->ExtId & 0xFFU);

    /*
     * 只接受协议里定义的操作码。
     */
    switch (opcode)
    {
    case ARM_CMD_SELF_CHECK:
    case ARM_CMD_ENABLE:
    case ARM_CMD_LEFT_WORK_1:
    case ARM_CMD_LEFT_WORK_2:
    case ARM_CMD_RIGHT_WORK_1:
    case ARM_CMD_RIGHT_WORK_2:
    case ARM_CMD_LEFT_PULLDOWN:
    case ARM_CMD_RIGHT_PULLDOWN:
    case ARM_CMD_ERROR:
    case ARM_CMD_RESET:
        break;

    default:
        return;
    }

    /*
     * DLC = 2; 不足 2 字节的按 0 补。
     */
    Arm_SendCommandFromISR(
        (ArmCommand_t)opcode,
        (RxHeader->DLC >= 1U) ? RxData[0] : 0U,
        (RxHeader->DLC >= 2U) ? RxData[1] : 0U);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (hcan->Instance == CAN1)
    {
        // 从 FIFO 0 把数据捞出来，存到 RxData 数组里
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
        #if USE_DJ && (MOTOR_DJI_CAN_BUS == 0U)
            DJmotor_Receive(RxHeader, RxData);

            /*
             * 3508 反馈 0x201..0x208：
             * 通知状态机对应电机已收到有效反馈。
             */
            if ((RxHeader.StdId >= 0x201U) &&
                (RxHeader.StdId <= 0x208U))
            {
                uint8_t dji_idx =
                    (uint8_t)(RxHeader.StdId - 0x201U);

                if (dji_idx < USE_DJNUM)
                {
                    Arm_NotifyDjiFeedback(dji_idx);
                }
            }
        #endif
        }
    }
    else if (hcan->Instance == CAN2)
    {

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
        #if USE_ZMDR
            /*
             * AK80 自动反馈帧 0x201..0x204 → FIFO0。
             *
             * 和 CAN1 FIFO0 收 DJI 反馈一样的链路：
             *
             *      帧 ID = 0x200 + 电机号
             *      DLC  = 8(4字节位置 + 2字节速度 + 2字节电流)
             *
             * ZdriveReceive() 的 DLC==8 分支正好解析这种帧。
             * 帧只在驱动进入运行模式后自动流出(1kHz)，
             * 失能状态没有。
             */
            if ((RxHeader.StdId >= 0x201U) &&
                (RxHeader.StdId <= 0x208U))
            {
                uint8_t z_idx =
                    (uint8_t)(RxHeader.StdId - 0x201U);

                if (z_idx < USE_ZDRIVE_NUM)
                {
                    ZdriveReceive(RxHeader, RxData, 1U);

                    Arm_NotifyZdriveFeedback(z_idx);
                }
            }
        #endif
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
        {
            /*
             * 主控动作命令:
             * 扩展帧 0x010105xx, DLC=2。
             */
            Arm_CAN_CommandParse(&RxHeader, RxData);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
        {
        #if USE_ZMDR
            /*
             * ZDrive 在 CAN2。
             *
             * 先判是不是我们的 ZDrive 帧，
             * 是才解析 + 通知反馈就绪。
             *
             * 不能用低 4 位直接判：
             * 主板命令帧 StdId 1..8 的低 4 位也是 1..8，
             * 会误当成 ZDrive。
             */
            if (Zdrive_IsOurs(&RxHeader, 1U))
            {
                uint8_t z_idx =
                    (uint8_t)(RxHeader.StdId & 0xFU) - 1U;

                ZdriveReceive(RxHeader, RxData, 1U);

                Arm_NotifyZdriveFeedback(z_idx);
            }
        #endif
        }
    }
}