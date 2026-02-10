#include "SystemTask.h"

// =============== 全局变量定义 ===============
volatile uint8_t g_calibrationUIStatus = 0;

// 共享数据实例
TaskSharedData_t sharedData;

// 任务句柄
TaskHandle_t taskUpperCommHandle = NULL;
TaskHandle_t taskCanCommHandle   = NULL;

// 注意：原 ino 中的 ServoManager 相关变量已被注释，因未启用 ServoCtrlTask，
// 若后续需启用，请在该文件中取消注释并添加相应头文件。
/*
ServoManager servoManager0(BUS0_SERVO_NUM);
ServoManager servoManager1(BUS1_SERVO_NUM);
ServoManager* servoManagers[NUM_SERVO_BUSES] = {
    &servoManager0,
    &servoManager1
};
*/

// =============== 系统初始化函数 ===============
void System_Init() {
    // 初始化串口
    Serial.begin(921600);
    while (!Serial) {
        delay(10);
    }
    delay(1000);

    //由于整理该部分被舍弃
    // // 创建命令队列
    // sharedData.cmdQueue = xQueueCreate(5, sizeof(ServoCommand_t));
    // if (sharedData.cmdQueue == NULL) {
    //     while (1); // 死循环，等待调试
    // }

    // // 创建状态队列
    // sharedData.statusQueue = xQueueCreate(3, sizeof(ServoStatus_t));
    // if (sharedData.statusQueue == NULL) {
    //     while (1);
    // }

    // // 【新增】创建 CAN 相关队列
    // sharedData.canRxQueue = xQueueCreate(1, sizeof(RemoteSensorData_t));
    // sharedData.canTxQueue = xQueueCreate(5, sizeof(RemoteCommand_t));

    // if (sharedData.canRxQueue == NULL || sharedData.canTxQueue == NULL) {
    //     while (1);
    // }

    
    // 1. 创建队列 (资源分配)
    sharedData.cmdQueue    = xQueueCreate(5, sizeof(ServoCommand_t));
    sharedData.statusQueue = xQueueCreate(3, sizeof(ServoStatus_t));
    sharedData.canRxQueue  = xQueueCreate(1, sizeof(RemoteSensorData_t));
    sharedData.canTxQueue  = xQueueCreate(5, sizeof(RemoteCommand_t));

    
    // 创建上位机通信任务
    xTaskCreate(
        taskUpperComm,
        "UpperComm",
        UPPER_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_UPPER_COMM_PRIORITY,
        &taskUpperCommHandle
    );

    // 4.3 【新增】CAN 通信任务
    xTaskCreate(
        taskCanComm,
        "CanComm",
        CAN_COMM_TASK_STACK_SIZE,
        &sharedData,
        TASK_CAN_COMM_PRIORITY,
        &taskCanCommHandle
    );

    Serial.println("✅ FreeRTOS tasks created successfully.");
    Serial.println("System ready. Commands: s(停止), r(恢复)");
}

// =============== 主循环函数 ===============
void System_Loop() {
    static uint32_t lastPrintTime = 0;

    // 每 200ms 打印一次（可选，建议注释以提升实时性）
    // if (millis() - lastPrintTime > 200) {
    //     lastPrintTime = millis();
    //     printCanMonitor(); // 若需此函数，请将其移到 UpperCommTask 或单独文件
    // }

    // 必须保留延时，防止触发看门狗！
    vTaskDelay(pdMS_TO_TICKS(10));
}