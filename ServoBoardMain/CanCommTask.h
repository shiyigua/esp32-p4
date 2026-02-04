#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H

#include <Arduino.h>
#include "driver/twai.h"

// 堆栈大小建议设置稍大，以防驱动层消耗
#define CAN_COMM_TASK_STACK_SIZE 4096 

// 任务函数声明
void taskCanComm(void *parameter);

#endif