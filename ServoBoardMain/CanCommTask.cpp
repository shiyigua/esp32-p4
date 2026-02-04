#include "CanCommTask.h"
#include "TaskSharedData.h"

// TWAI 配置参数
void setupTwaiForP4() {
    // 1. 初始化配置
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_PIN, 
        (gpio_num_t)TWAI_RX_PIN, 
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = 20; // 增加接收缓存
    g_config.tx_queue_len = 20;

    // 2. 波特率配置 (例如 1Mbps，需与 S3 保持一致)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

    // 3. 过滤器配置 (接收所有 ID)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 安装驱动
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("[CAN] Driver installed");
    } else {
        Serial.println("[CAN] Driver install failed!");
        return;
    }

    // 启动驱动
    if (twai_start() == ESP_OK) {
        Serial.println("[CAN] Driver started");
    } else {
        Serial.println("[CAN] Driver start failed!");
    }
}

void taskCanComm(void *parameter) {
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    
    Serial.println("CAN (TWAI) 通信任务启动 (ESP32-P4)");
    
    // 初始化硬件
    setupTwaiForP4();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // 10ms 周期运行

    twai_message_t rxMsg;
    RemoteCommand_t txCmd;

    // 【关键】定义一个静态变量来缓存这21个数据
    // 因为 CAN 是分包来的，我们需要一个地方把包拼起来
    static RemoteSensorData_t accumulatedData; 
    
    // 用于标记是否收到新数据
    bool isDataUpdated = false;

    // while (true) {
    //     // -----------------------------
    //     // 1. 接收处理 (RX from S3)
    //     // -----------------------------
    //     // 非阻塞或短超时接收所有缓存中的消息
    //     while (twai_receive(&rxMsg, 0) == ESP_OK) {
    //         // 简单示例：假设 0x100 是传感器数据 ID
    //         if (rxMsg.identifier == 0x100 && rxMsg.data_length_code >= 2) {
    //             // 模拟解包：这里根据你的实际协议解析
    //             // 假设前2字节是某个数据
    //             parsedData.encoderValues[0] = (rxMsg.data[0] << 8) | rxMsg.data[1];
    //             parsedData.timestamp = millis();
                
    //             // 将解包后的数据的副本发送到队列供其他任务使用
    //             // xQueueOverwrite 适合只保留最新数据
    //             xQueueOverwrite(sharedData->canRxQueue, &parsedData);
                
    //             // Debug 打印 (生产环境可注释)
    //             //  Serial.printf("[CAN RX] ID:0x%X Val:%d\n", rxMsg.identifier, parsedData.encoderValues[0]);
    //         }
    //     }
    // //打印测试信息
    // // Serial.println(test_times); 
    // test_times++; 
    //     // -----------------------------
    //     // 2. 发送处理 (TX to S3)
    //     // -----------------------------
    //     // 检查是否有任务需要向 S3 发送指令
    //     if (xQueueReceive(sharedData->canTxQueue, &txCmd, 0) == pdTRUE) {
    //         twai_message_t txMsg;
    //         txMsg.identifier = 0x200; // 假设 0x200 是命令 ID
    //         txMsg.extd = 0;           // 标准帧
    //         txMsg.rtr = 0;
    //         txMsg.data_length_code = txCmd.len;
    //         memcpy(txMsg.data, txCmd.payload, txCmd.len);

    //         if (twai_transmit(&txMsg, pdMS_TO_TICKS(10)) != ESP_OK) {
    //             Serial.println("[CAN TX] 发送失败");
    //         } else {
    //              Serial.println("[CAN TX] 指令已发送");
    //         }
    //     }
        
    //     // 维持任务周期
    //     vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // }
     for (;;) {
        // ============================================================
        // 1. 接收处理 (RX): 核心重写部分
        // ============================================================
        while (twai_receive(&rxMsg, 0) == ESP_OK) {
            
            // 判断是否为传感器数据帧 (ID: 0x100 ~ 0x105)
            if (rxMsg.identifier >= 0x100 && rxMsg.identifier <= 0x105) {
                
                // 计算这是第几帧 (Offset)
                int frameIndex = rxMsg.identifier - 0x100;
                
                if (frameIndex < 5) {
                    // --- ID 0x100 ~ 0x104 (处理前20个数据) ---
                    // 每个包里有 4 个 uint16_t (4 * 2 = 8 bytes)
                    // 0x100 -> index 0~3
                    // 0x101 -> index 4~7
                    // ...
                    if (rxMsg.data_length_code == 8) {
                        memcpy(&accumulatedData.encoderValues[frameIndex * 4], rxMsg.data, 8);
                        isDataUpdated = true;
                    }
                } 
                else if (frameIndex == 5) {
                    // --- ID 0x105 (处理第21个数据) ---
                    // 只有 1 个 uint16_t (2 bytes)
                    memcpy(&accumulatedData.encoderValues[20], rxMsg.data, 2);
                    
                    // 收到最后一帧时，更新时间戳并推送队列
                    accumulatedData.timestamp = millis();
                    isDataUpdated = true;
                }
            }
        }

        // 只有当数据有更新时，才写入队列，避免无意义的覆盖
        if (isDataUpdated) {
            xQueueOverwrite(sharedData->canRxQueue, &accumulatedData);
            isDataUpdated = false;
        }

        // ============================================================
        // 2. 发送处理 (TX) (保持不变)
        // ============================================================
        if (xQueueReceive(sharedData->canTxQueue, &txCmd, 0) == pdTRUE) {
            twai_message_t txMsg;
            txMsg.identifier = 0x000; 
            txMsg.extd = 0;
            txMsg.rtr = 0;
            txMsg.data_length_code = txCmd.len;
            memcpy(txMsg.data, txCmd.payload, txCmd.len);
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
        }

        // 这里的延时决定了任务频率，根据需要调整
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}