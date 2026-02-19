#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H

#include <Arduino.h>
#include "TaskSharedData.h"

// 堆栈大小建议设置稍大，以防驱动层消耗
// #define CAN_COMM_TASK_STACK_SIZE 4096 //已调整位置


// ==========================================
// 【修改】CAN ID 定义 - 与 HalTWAI 发送端一致
// ==========================================

// #define ENCODER_TOTAL_NUM 21
#define CAN_ID_ENC_BASE   0x100  // 【修改】从 0x200 改为 0x100
#define CAN_ID_ENC_LAST   (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1) // 0x105
#define CAN_ID_ERR_STATUS 0x1F0  // 错误状态帧 ID (与 HalTWAI 一致)

void taskCanComm(void *parameter);

#endif
