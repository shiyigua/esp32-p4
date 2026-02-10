#include <Arduino.h>
#include "SystemTask.h"

void setup() {
    System_Init();
}

void loop() {
    System_Loop();
}

// #include <Arduino.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include "ServoManager.h"
// #include "UpperCommTask.h"
// #include "ServoCtrlTask.h"
// #include "TaskSharedData.h"
// #include "CanCommTask.h" // 放在其他头文件后面

// #define ENCODER_TOTAL_NUM 21 // 请根据实际数量修改


// ServoManager servoManager0(BUS0_SERVO_NUM);
// ServoManager servoManager1(BUS1_SERVO_NUM);
// ServoManager* servoManagers[NUM_SERVO_BUSES] = {
//     &servoManager0,
//     &servoManager1
// };

// // 任务句柄
// TaskHandle_t taskUpperCommHandle = NULL;
// TaskHandle_t taskServoCtrlHandle = NULL;
// TaskHandle_t taskCanCommHandle   = NULL; // 【新增】CAN 任务句柄

// // 共享数据结构实例
// TaskSharedData_t sharedData;

// // --- [新增] 全局标志位，用于跨任务显示校准结果 ---
// // 0:无状态, 1:进行中, 2:成功, 3:失败
// volatile uint8_t g_calibrationUIStatus = 0; 

// void setup() {
//     // 初始化串口
//     Serial.begin(921600);
//     while (!Serial) {
//         delay(10);
//     }
//     delay(1000);
//     // Serial.println("FEETECH BUS Servo 控制系统启动");
    
//     // // 初始化舵机管理器
//     // servoManagers[0]->begin(0, UART1_rxPin, UART1_txPin, BUS0_IDS);
//     // servoManagers[1]->begin(1, UART2_rxPin, UART2_txPin, BUS1_IDS);
//     // uint8_t totalServoNum = 0;
//     // for (int i = 0; i < NUM_SERVO_BUSES; i++) {
//     //     totalServoNum += servoManagers[i]->numServos;
//     //     sharedData.servoManagers[i] = servoManagers[i];
//     // }
//     // sharedData.totalServoNum = totalServoNum;
//     // Serial.println("舵机管理器初始化完成");
    
//     // 创建命令队列
//     sharedData.cmdQueue = xQueueCreate(5, sizeof(ServoCommand_t));
//     if (sharedData.cmdQueue == NULL) {
//         // Serial.println("创建命令队列失败");
//         while (1);
//     }
    
//     // 创建状态队列
//     sharedData.statusQueue = xQueueCreate(3, sizeof(ServoStatus_t));
//     if (sharedData.statusQueue == NULL) {
//         // Serial.println("创建状态队列失败");
//         while (1);
//     }

//         // 【新增】创建 CAN 相关队列
//     // canRxQueue 长度设为1，因为通常我们只关心最新的传感器数据
//     sharedData.canRxQueue = xQueueCreate(1, sizeof(RemoteSensorData_t));
//     // canTxQueue 长度设为5，防止指令堆积
//     sharedData.canTxQueue = xQueueCreate(5, sizeof(RemoteCommand_t));

//     if (sharedData.canRxQueue == NULL || sharedData.canTxQueue == NULL) {
//         // Serial.println("创建 CAN 队列失败"); while (1);
//     }

//     // 创建上位机通信任务
//     xTaskCreate(
//         taskUpperComm,           // 任务函数
//         "UpperComm",            // 任务名称
//         UPPER_COMM_TASK_STACK_SIZE, // 堆栈大小
//         &sharedData,           // 传递共享数据指针
//         TASK_UPPER_COMM_PRIORITY, // 优先级
//         &taskUpperCommHandle    // 任务句柄
//     );
    
//     // // 创建舵机控制任务
//     // xTaskCreate(
//     //     taskServoCtrl,          // 任务函数
//     //     "ServoCtrl",           // 任务名称
//     //     SERVO_CTRL_TASK_STACK_SIZE, // 堆栈大小
//     //     &sharedData,           // 传递共享数据指针
//     //     TASK_SERVO_CTRL_PRIORITY, // 优先级
//     //     &taskServoCtrlHandle   // 任务句柄
//     // );

//         // 4.3 【新增】CAN 通信任务
//     xTaskCreate(
//         taskCanComm,            // 任务函数
//         "CanComm",              // 任务名称
//         CAN_COMM_TASK_STACK_SIZE, // 堆栈大小
//         &sharedData,            // 参数
//         TASK_CAN_COMM_PRIORITY, // 优先级 (建议较高)
//         &taskCanCommHandle      // 句柄
//     );
    
//     // 删除默认的Arduino loop任务
//     // vTaskDelete(NULL);

//     // Serial.println("FreeRTOS任务创建完成");
//     // Serial.println("系统准备就绪");
//     // Serial.println("可用命令: s(停止), r(恢复)");
//     // Serial.println("======================================");
// }

// void loop() {
//     static uint32_t lastPrintTime = 0;
    
//     // 每 200ms 打印一次（避免串口阻塞）
//     if (millis() - lastPrintTime > 200) {
//         lastPrintTime = millis();
//         // printCanMonitor();
//     }

//     // 必须保留延时，防止触发看门狗！
//     vTaskDelay(pdMS_TO_TICKS(10)); 
// }

// // ==========================================
// // [XSimple] CAN 数据监控函数 (P4端)
// // ==========================================
// void printCanMonitor() {
//     // 1. 从 CAN 队列安全读取数据（非阻塞）
//     RemoteSensorData_t sensorData;
//     if (xQueuePeek(sharedData.canRxQueue, &sensorData, 0) == pdFALSE) {
//         Serial.println("[WARN] No CAN data received yet");
//         return;
//     }

//     // 2. ANSI 清屏（可选，如果终端不支持会显示乱码）
//     // Serial.print("\033[2J\033[H"); 

//     // 3. 打印表头
//     Serial.println("\n========== [ CAN Sensor Monitor (S3 -> P4) ] ==========");
//     Serial.printf("Total Encoders: %d | Update Time: %lu ms\n", 
//                  ENCODER_TOTAL_NUM, millis());

//     // 4. 按网格布局打印数据（每行4个）
//     for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
//         uint16_t val = sensorData.encoderValues[i];
//         bool isError = (val == 0 || val == 0xFFFF || val == 16383);

//         // 格式: [ID] 原始值 | 角度 | 状态
//         Serial.printf("[%02d] 0x%04X (%6.1f°) %s  ", 
//                      i, 
//                      val, 
//                      (val * 360.0f) / 16384.0f,  // 转换为角度
//                      isError ? "⚠️" : "✅");

//         // 每4个换行
//         if ((i + 1) % 4 == 0) Serial.println();
//     }

//     // 5. 打印分隔线
//     if (ENCODER_TOTAL_NUM % 4 != 0) Serial.println(); // 补换行
//     Serial.println("======================================================");
// }