#ifndef TASK_SHARED_DATA_H
#define TASK_SHARED_DATA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "ServoManager.h"
#include <freertos/semphr.h>  // 【新增】互斥锁头文件
#include "CanCommTask.h"



// ============ 常量定义 ============
#define ENCODER_TOTAL_NUM       21


// 舵机总线数量及引脚
// #define NUM_SERVO_BUSES 2 
// #define UART0_rxPin 22
// #define UART0_txPin 23
// #define UART1_rxPin 20
// #define UART1_txPin 21
// #define UART2_rxPin 22
// #define UART2_txPin 23
// #define UART3_rxPin 20
// #define UART3_txPin 21
// static const uint8_t BUS0_IDS[] = {0, 2, 5, 7, 9, 11, 13, 14, 17, 19};
// static const uint8_t BUS0_SERVO_NUM = sizeof(BUS0_IDS) / sizeof(BUS0_IDS[0]);
// static const uint8_t BUS1_IDS[] = {1, 3, 4, 6, 8, 10, 12, 15, 18}; // TODO: remove 16
// static const uint8_t BUS1_SERVO_NUM = sizeof(BUS1_IDS) / sizeof(BUS1_IDS[0]);

// // 舵机初始化位置结构体
// struct InitConfig {
//     uint8_t busIndex;   // 总线索引
//     uint8_t servoID;    // 舵机ID
//     float   percent;    // 目标位置百分比 (0.0 = 2048, 1.0 = 极限位置)
// };

// // 舵机初始化顺序和配置
// static const InitConfig SERVO_INIT_SEQUENCE[] = {
//     // {总线号, 舵机ID, 百分比位置}
//     {0, 0, 1.0}, 
//     {1, 1, 0.0}, 
//     {0, 2, 0.0}, 
//     {1, 3, 0.0}, 
//     {1, 4, 1.0}, 
//     {0, 5, 0.0}, 
//     {1, 6, 0.0}, 
//     {0, 7, 0.0}, 
//     {1, 8, 1.0}, 
//     {0, 9, 0.0}, 
//     {1, 10, 0.0}, 
//     {0, 11, 0.0}, 
//     {1, 12, 1.0}, 
//     {0, 13, 0.0}, 
//     {0, 14, 0.0}, 
//     {1, 15, 0.0}, 
//     // TODO: {1, 16, 1.0}, 
//     {0, 17, 0.0}, 
//     {1, 18, 0.0}, 
//     {0, 19, 0.0} 
// };

// --- 【新增】CAN / TWAI 配置 ---
#define TWAI_TX_PIN 47  // 请根据实际 P4 硬件连接修改
#define TWAI_RX_PIN 48  // 请根据实际 P4 硬件连接修改


// 任务优先级定义
#define TASK_UPPER_COMM_PRIORITY 1
// #define TASK_SERVO_CTRL_PRIORITY 2  //已弃用
#define TASK_CAN_COMM_PRIORITY   3  // 【新增】CAN通信优先级
#define TASK_SOLVER_PRIORITY      4   // 【新增】解算任务优先级（最高，保证实时性）

// ============ 任务堆栈 ============
#define UPPER_COMM_TASK_STACK_SIZE 8192
#define CAN_COMM_TASK_STACK_SIZE   4096
#define SOLVER_TASK_STACK_SIZE     8192  // 【新增】解算任务堆栈


// --- 【新增】来自 ESP32-S3 的传感器数据结构 ---
// typedef struct {
//     uint16_t encoderValues[21]; // 示例：接收到的编码器数据
//     uint8_t  sensorStatus;
//     uint32_t timestamp;
// } RemoteSensorData_t;
// typedef struct {
//     uint16_t encoderValues[21];             // 编码器角度值 (0~16383)
//     uint8_t  errorFlags[21];                // 【新增】单个编码器错误标志 (1=错误, 0=正常)
//     uint32_t errorBitmap;                   // 【新增】全局错误位图 (来自 CAN 0x1F0)
//     uint32_t timestamp;                     // 数据时间戳
//     bool     isValid;                       // 【新增】数据有效性标志
// } RemoteSensorData_t;

// --- 【新增】发送给 ESP32-S3 的指令结构 ---
typedef struct {
    uint8_t cmdID;
    uint8_t payload[8];
    uint8_t len;
} RemoteCommand_t;



// --- 【新增】发送给 ESP32-S3 的指令结构 ---
typedef struct {
    uint16_t encoderValues[ENCODER_TOTAL_NUM];
    uint8_t  errorFlags[ENCODER_TOTAL_NUM];
    uint32_t errorBitmap;
    uint32_t timestamp;
    bool     isValid;
} RemoteSensorData_t;

// 舵机指令
typedef struct {
    uint8_t cmdType;
    uint8_t servoId;
    int16_t position;
    uint16_t speed;
    uint8_t busIndex;
} ServoCommand_t;

// 舵机状态
typedef struct {
    uint8_t servoId;
    int16_t position;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
} ServoStatus_t;

// 任务间共享的数据结构
typedef struct {
    QueueHandle_t cmdQueue;
    QueueHandle_t statusQueue;

        // 【新增】CAN 通信队列
    QueueHandle_t canTxQueue;    // 存放要发给 S3 的指令
    QueueHandle_t canRxQueue;    // 存放从 S3 收到的解包数据
    
     // 【新增】目标角度数组 + 互斥锁
    // 由 UpperCommTask 写入，由 taskSolver 读取
    float targetAngles[ENCODER_TOTAL_NUM];
    SemaphoreHandle_t targetAnglesMutex;
} TaskSharedData_t;

#endif