#ifndef UPPER_COMM_TASK_H
#define UPPER_COMM_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "TaskSharedData.h"

// #define UPPER_COMM_TASK_STACK_SIZE 8192//已调整位置
void taskUpperComm(void *parameter);

#endif