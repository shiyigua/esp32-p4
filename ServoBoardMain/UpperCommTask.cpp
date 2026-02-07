#include "UpperCommTask.h"
#include "CanCommTask.h"
extern volatile uint8_t g_calibrationUIStatus;

// --- 协议定义 ---
#define PROTOCOL_HEADER        0xFE
#define PROTOCOL_TAIL          0xFF
#define PACKET_TYPE_SENSOR     0x01
#define PACKET_TYPE_CALIB_ACK  0x02

// 内部辅助：发送数据包
void sendDataPacket(ServoStatus_t* pServo, RemoteSensorData_t* pSensor) {
    uint8_t buffer[64];
    size_t idx = 0;

    // 1. 帧头
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // 长度占位

    // 2. 负载内容
    if (g_calibrationUIStatus != 0) {
        // [高优先级] 校准状态反馈
        buffer[idx++] = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = g_calibrationUIStatus; 
        // g_calibrationUIStatus = 0; // 发送后清除状态，避免重复
    } 
    else if (pSensor) {
        // [常规] 传感器数据
        buffer[idx++] = PACKET_TYPE_SENSOR;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            uint16_t val = pSensor->encoderValues[i];
            buffer[idx++] = (val >> 8) & 0xFF;
            buffer[idx++] = val & 0xFF;
        }
    } else {
        return; // 无数据
    }

    // 3. 填充长度与帧尾
    // buffer[1] = idx - 2; 
    buffer[idx++] = PROTOCOL_TAIL;

    // 4. 正确计算 LEN: LEN = TYPE + PAYLOAD + TAIL
    // 注意：idx 已包含 TAIL，所以 LEN = idx - 2 (减去 FE 和 LEN 自身)
    buffer[1] = (uint8_t)(idx - 2);

    // 4. 写串口
    Serial.write(buffer, idx);

        // 【关键】仅在此处清除状态（防止重复发送）
    if (g_calibrationUIStatus != 0) {
        g_calibrationUIStatus = 0;
    }
}

// --- 主任务函数 ---
void taskUpperComm(void *parameter) {
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    RemoteSensorData_t sensorData;
    
    Serial.println("<<<SYS_READY>>>"); // 启动标志

    for (;;) {
        // ====================================================
        // [Part 1] 接收：处理来自 PC 的指令 (RX)
        // ====================================================
        if (Serial.available()) {
            uint8_t rxByte = Serial.read();
            
            // 简单指令解析: 'c' 或 0xCA 触发校准
            if (rxByte == 'c' || rxByte == 0xCA) {
                RemoteCommand_t cmd;
                cmd.cmdID = 0x200;    // 目标: S3
                cmd.len = 1;
                cmd.payload[0] = 0xCA; // 内容: Calibrate

                // 发送给 CAN 任务
                if (xQueueSend(sharedData->canTxQueue, &cmd, 0) == pdTRUE) {
                    g_calibrationUIStatus = 1; // 设置本地状态为 PENDING
                }
            }
            // 可以在此扩展其他指令，例如 's' 停止等
        }

        // ====================================================
        // [Part 2] 发送：上传状态与数据 (TX)
        // ====================================================
        
        // 尝试从队列获取最新的传感器数据
        // 使用 Receive 移除队列中旧数据，保证实时性
        if (xQueueReceive(sharedData->canRxQueue, &sensorData, 0) == pdTRUE) {
            sendDataPacket(NULL, &sensorData);
        }
        // 如果没有传感器数据，但有校准状态变化 (Success/Fail)，也需要立即发送
        else if (g_calibrationUIStatus != 0) {
            sendDataPacket(NULL, NULL);
        }

        // 任务调度延时
        // 既要保证 input 响应快，又要避免 output 发送太快堵塞串口
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}


// #include "UpperCommTask.h"
// #include "TaskSharedData.h"
// #include "CanCommTask.h"

// // ---------------------------------------------------------
// // 将 CAN 接收到的原始数据转为 JSON 格式并通过串口发送
// // ---------------------------------------------------------
// void sendDataPacket(uint32_t timestamp, ServoStatus_t* servoStatus, RemoteSensorData_t* sensorData) {
    
//     // 定义缓冲区 (足够容纳 21 个编码器数据)
//     char jsonBuffer[1024]; 
//     int offset = 0;

//     // JSON 开始
//     offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "{");

//     // 1. 处理来自 S3 的磁编数据 (如果有效)
//     if (sensorData && sensorData->isValid) {
//         // ts: P4接收时间戳
//         // emap: S3上传的全局错误位图 (Error Bitmap from CAN ID 0x1F0)
//         offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, 
//                            "\"ts\":%lu,\"emap\":%u,\"enc\":[", 
//                            sensorData->timestamp, 
//                            sensorData->errorBitmap);

//         // 填充编码器原始值数组 (Raw Values 0~16383)
//         for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
//             offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, 
//                                "%d%s", 
//                                sensorData->encoderValues[i], 
//                                (i < ENCODER_TOTAL_NUM - 1) ? "," : "");
//         }

//         // 填充单体错误标志数组 (Flags)
//         offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, 
//                            "],\"err\":[");
//         for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
//             offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, 
//                                "%d%s", 
//                                sensorData->errorFlags[i], 
//                                (i < ENCODER_TOTAL_NUM - 1) ? "," : "");
//         }
//         offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "]");
//     }

//     // 2. (可选) 可以在此添加 P4 本地的舵机状态数据上传逻辑
//     // ...

//     // JSON 结束并换行 (作为数据包分隔符)
//     offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "}\n");
    
//     // 直接发送至电脑
//     Serial.write(jsonBuffer, offset);
// }

// void taskUpperComm(void *parameter) {
//     TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    
//     ServoStatus_t servoStatus;
//     RemoteSensorData_t sensorData;
    
//     // 初始化
//     memset(&servoStatus, 0, sizeof(servoStatus));
//     memset(&sensorData, 0, sizeof(sensorData));

//     for (;;) {
//         // 非阻塞读取最新状态
//         bool hasSensor = (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE);
//         bool hasServo  = (xQueuePeek(sharedData->statusQueue, &servoStatus, 0) == pdTRUE);
        
//         // 只要有 CAN 磁编数据就上传
//         if (hasSensor) {
//             sendDataPacket(millis(), hasServo ? &servoStatus : NULL, &sensorData);
//         }

//         // 限制上传频率为 20Hz (50ms)，避免串口拥塞
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }


// #include "UpperCommTask.h"
// #include <Arduino.h>

// // 定义数据上报频率 (ms)，防止串口缓冲区在低波特率下溢出
// // 如果你的波特率是 115200，建议 >= 50ms。如果是 921600，可以设为 10ms。
// #define REPORT_INTERVAL_MS 50 

// /**
//  * @brief 手动构建 JSON 字符串并发送，避免引入额外的 ArduinoJson 库
//  * 格式示例: {"ts":1000,"s":[[1,2048,100],[2,1024,0]...],"e":[10,20,30...]}
//  * ts: 时间戳
//  * s: 舵机数组 [[ID, 位置, 负载], ...]
//  * e: 磁编数组 [值1, 值2, ...]
//  */
// void sendDataPacket(uint32_t timestamp, ServoStatus_t* servoStatus, RemoteSensorData_t* canData) {
//     // 静态缓冲区，1024字节通常足以容纳20个舵机+21个磁编的数据
//     static char buffer[1024];
//     int offset = 0;

//     // 1. 开始 JSON 构造
//     offset += snprintf(buffer + offset, sizeof(buffer) - offset, "{\"ts\":%lu", timestamp);

//     // 2. 添加舵机数据 (Key: "s")
//     if (servoStatus != NULL && servoStatus->totalServoNum > 0) {
//         offset += snprintf(buffer + offset, sizeof(buffer) - offset, ",\"s\":[");
//         for (int i = 0; i < servoStatus->totalServoNum; i++) {
//             offset += snprintf(buffer + offset, sizeof(buffer) - offset, 
//                 "[%d,%d,%d]", 
//                 servoStatus->states[i].id, 
//                 servoStatus->states[i].absolutePos, 
//                 servoStatus->states[i].currentLoad);
            
//             if (i < servoStatus->totalServoNum - 1) {
//                 // 如果缓冲区空间不足，提前截断防止崩溃
//                 if (sizeof(buffer) - offset < 20) break; 
//                 offset += snprintf(buffer + offset, sizeof(buffer) - offset, ",");
//             }
//         }
//         offset += snprintf(buffer + offset, sizeof(buffer) - offset, "]");
//     }

//     // 3. 添加磁编数据 (Key: "e")
//     // 假设 RemoteSensorData_t 中的 encoderValues 固定为 21 个长度
//     if (canData != NULL) {
//         offset += snprintf(buffer + offset, sizeof(buffer) - offset, ",\"e\":[");
//         for (int i = 0; i < 21; i++) {
//             offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%d", canData->encoderValues[i]);
            
//             if (i < 20) {
//                 offset += snprintf(buffer + offset, sizeof(buffer) - offset, ",");
//             }
//         }
//         offset += snprintf(buffer + offset, sizeof(buffer) - offset, "]");
//     }

//     // 4. 结束 JSON 并添加换行符
//     offset += snprintf(buffer + offset, sizeof(buffer) - offset, "}\n");

//     // 5. 通过串口发送
//     Serial.write(buffer, offset);
// }

// void taskUpperComm(void *parameter) {
//     TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    
//     // 发送一条非 JSON 的日志，Python 端在解析 JSON 出错时可以 print 出来看
//     Serial.println("# 上位机通信任务已就绪 (Protocol: JSON Lines)");

//     // 本地缓存数据
//     ServoStatus_t currentStatus;
//     RemoteSensorData_t currentCanData;
    
//     bool hasServoData = false;
//     bool hasCanData = false;
    
//     uint32_t lastReportTime = 0;
//     String inputBuffer = ""; // 用于缓存串口输入命令

//     while (true) {
//         // ---------------------------------------------------------
//         // 1. 处理串口接收 (非阻塞读取)
//         // ---------------------------------------------------------
//         while (Serial.available()) {
//             char c = Serial.read();
//             if (c == '\n' || c == '\r') {
//                 // 收到换行符，处理命令
//                 inputBuffer.trim(); // 去除可能的空白符
//                 if (inputBuffer.length() > 0) {
//                     ServoCommand_t cmd;
//                     cmd.timestamp = millis();

//                     if (inputBuffer.equalsIgnoreCase("s")) {
//                         cmd.command = 's'; // S: Stop
//                         xQueueSend(sharedData->cmdQueue, &cmd, portMAX_DELAY);
//                         // 可以选择是否回显，或者只发送 JSON 状态
//                         // Serial.println("{\"msg\":\"STOP_CMD_RECEIVED\"}");
//                     } 
//                     else if (inputBuffer.equalsIgnoreCase("r")) {
//                         cmd.command = 'r'; // R: Resume
//                         xQueueSend(sharedData->cmdQueue, &cmd, portMAX_DELAY);
//                         // Serial.println("{\"msg\":\"RESUME_CMD_RECEIVED\"}");
//                     }
//                     else {
//                         // 其它扩展命令处理
//                     }
//                 }
//                 inputBuffer = ""; // 清空缓冲区
//             } else {
//                 inputBuffer += c; // 累加字符
//             }
//         }

//         // ---------------------------------------------------------
//         // 2. 从队列获取最新数据 (非阻塞)
//         // ---------------------------------------------------------
//         if (xQueueReceive(sharedData->statusQueue, &currentStatus, 0) == pdTRUE) {
//             hasServoData = true;
//         }

//         if (xQueueReceive(sharedData->canRxQueue, &currentCanData, 0) == pdTRUE) {
//             hasCanData = true;
//         }

//         // ---------------------------------------------------------
//         // 3. 定时打包上报
//         // ---------------------------------------------------------
//         uint32_t now = millis();
//         if (now - lastReportTime >= REPORT_INTERVAL_MS) {
//             // 只有当至少有一种数据时才发送，避免发送空包
//             if (hasServoData || hasCanData) {
//                 sendDataPacket(now, 
//                                hasServoData ? &currentStatus : NULL, 
//                                hasCanData ? &currentCanData : NULL);
//             }
//             lastReportTime = now;
//         }

//         // 释放 CPU 给其他任务
//         vTaskDelay(pdMS_TO_TICKS(5));
//     }
// }



// #include "UpperCommTask.h"
// #include "TaskSharedData.h"

// void printServoStatus(ServoStatus_t status) {
//     Serial.println("=== 舵机状态 ===");
//     Serial.println("ID\t绝对位置\t位置上限\t负载\t最后通信时间");
//     for (uint8_t i = 0; i < status.totalServoNum; i++) {
//         Serial.print(status.states[i].id);
//         Serial.print("\t");
//         Serial.print(status.states[i].absolutePos);
//         Serial.print("\t\t");
//         Serial.print(status.states[i].limitPos);
//         Serial.print("\t\t");
//         Serial.print(status.states[i].currentLoad);
//         Serial.print("\t");
//         Serial.print(status.states[i].lastCommTime);
//         Serial.println();
//     }
// }

// // 上位机通信任务
// void taskUpperComm(void *parameter) {
//     TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    
//     Serial.println("上位机通信任务启动");
    
//     ServoStatus_t currentStatus;
//     uint32_t lastStatusPrintTime = 0;
//     const uint32_t STATUS_PRINT_INTERVAL = 100; // 0.1秒打印一次状态
    
//     while (true) {
//         // 1. 处理用户输入
//         if (Serial.available()) {
//             String input = Serial.readString();
//             input.trim();
            
//             ServoCommand_t cmd;
//             cmd.timestamp = millis();
            
//             if (input.equals("s") || input.equals("S")) {
//                 cmd.command = 's'; // 停止
//                 xQueueSend(sharedData->cmdQueue, &cmd, portMAX_DELAY);
//                 Serial.println("发送紧急停止命令");
//             } else if (input.equals("r") || input.equals("R")) {
//                 cmd.command = 'r'; // 恢复
//                 xQueueSend(sharedData->cmdQueue, &cmd, portMAX_DELAY);
//                 Serial.println("发送恢复运行命令");
//             } else {
//                 Serial.println("未知命令，可用命令: s(停止), r(恢复)");
//             }
//         }
        
//         // 2. 定期打印状态（非阻塞检查）
//         uint32_t currentTime = millis();
//         if (currentTime - lastStatusPrintTime > STATUS_PRINT_INTERVAL) {
//             if (xQueueReceive(sharedData->statusQueue, &currentStatus, 0) == pdTRUE) {
//                 printServoStatus(currentStatus);
//                 lastStatusPrintTime = currentTime;
//             }
//         }
        
//         vTaskDelay(10 / portTICK_PERIOD_MS); // 等待10ms，这段时间该进程不被调度器执行
//     }
// }