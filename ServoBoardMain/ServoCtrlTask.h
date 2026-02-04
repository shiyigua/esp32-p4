#ifndef SERVO_CTRL_TASK_H
#define SERVO_CTRL_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "TaskSharedData.h"

#define SERVO_CTRL_TASK_STACK_SIZE 4096
void taskServoCtrl(void *parameter);

#endif