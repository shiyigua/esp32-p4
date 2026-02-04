#include "ServoCtrlTask.h"
#include "TaskSharedData.h"

// 舵机控制任务
void taskServoCtrl(void *parameter) {
    Serial.println("舵机控制任务启动");
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 30 / portTICK_PERIOD_MS; // 30ms周期

    // 舵机通信自检
    Serial.println("### 系统自检: 检查所有舵机通信状态 (不更新位置) ###");
    bool allCheckPassed = true;
    int seqLen = sizeof(SERVO_INIT_SEQUENCE) / sizeof(SERVO_INIT_SEQUENCE[0]);

    // 1. 按总线进行同步读取测试 (SyncReadPosLoad)
    for (int bus = 0; bus < NUM_SERVO_BUSES; bus++) {
        if (sharedData->servoManagers[bus]) {
            Serial.printf(">>>总线 %d: 执行 SyncReadPosLoad 通信测试...\n", bus);
            if (!sharedData->servoManagers[bus]->SyncReadPosLoad(2, false)) {
                Serial.printf("错误: 总线 %d 同步读取测试失败！\n", bus);
                allCheckPassed = false;
            } else {
                Serial.printf("总线 %d 同步读取正常。\n", bus);
            }
        }
    }

    // 2. 对每个定义的舵机进行单机读取测试 (ReadSinglePosLoad)
    // 遍历定义好的初始化序列来检查每一个舵机
    Serial.println(">>> 执行单个舵机 ReadSinglePosLoad 通信测试...");
    for(int i = 0; i < seqLen; i++) {
        uint8_t bus = SERVO_INIT_SEQUENCE[i].busIndex;
        uint8_t servoID  = SERVO_INIT_SEQUENCE[i].servoID;

        if (bus < NUM_SERVO_BUSES && sharedData->servoManagers[bus]) {
            if (!sharedData->servoManagers[bus]->ReadSinglePosLoad(servoID, false)) {
                Serial.printf("错误: 舵机 [总线%d ID%d] 通信无响应！\n", bus, servoID);
                allCheckPassed = false;
            } else {
                Serial.printf("舵机 [总线%d ID%d] 通信OK\n", bus, servoID);
            }
        }
        delay(5); // 稍微给总线一点喘息时间
    }

    if (!allCheckPassed) {
        Serial.println("!!!!!!!!!! 严重错误: 通信自检失败，请检查接线或电源 !!!!!!!!!!");
        Serial.println("!!!!!!!!!! 为了安全，任务将暂停，不执行动作 !!!!!!!!!!");
        while(1) { delay(1000); } 
    }
    Serial.println("### 系统自检通过，开始执行初始化动作 ###");

    // 舵机初始化
    Serial.println("按照TaskSharedData.h定义的顺序初始化舵机中...");
    for(int i = 0; i < seqLen; i++) {
        // 从 TaskSharedData.h 中读取配置
        uint8_t bus = SERVO_INIT_SEQUENCE[i].busIndex;
        uint8_t servoID  = SERVO_INIT_SEQUENCE[i].servoID;
        float   pct = SERVO_INIT_SEQUENCE[i].percent;

        if (bus < NUM_SERVO_BUSES && sharedData->servoManagers[bus]) {
            Serial.printf("[%d/%d] 初始化 -> 总线:%d ID:%d 目标:%.1f%%\n", i+1, seqLen, bus, servoID, pct*100);
            if (!sharedData->servoManagers[bus]->InitializeSingleServo(servoID, pct)) {
                Serial.printf("错误: 总线 %d ID %d 初始化失败!\n", bus, servoID);
            }
            delay(500);
        } else {
            Serial.printf("跳过无效配置: 总线 %d 不存在\n", bus);
        }
    }
    Serial.println("所有指定舵机初始化完成");
    delay(100);

    // 开始日常运动控制
    ServoStatus_t statusToSend;
    statusToSend.states = new ServoState[sharedData->totalServoNum];
    uint8_t readServoNum = 0;
    while (true) {
        uint32_t currentTime = millis();

        // 1. 检查是否有新命令
        ServoCommand_t cmd;
        if (xQueueReceive(sharedData->cmdQueue, &cmd, 0) == pdTRUE) {
            for (int bus = 0; bus < NUM_SERVO_BUSES; bus++) {
                ServoManager* mgr = sharedData->servoManagers[bus];
                if (mgr) {
                    if (cmd.command == 's') {
                        mgr->EmergencyStop();
                    } else if (cmd.command == 'r') {
                        mgr->Resume();
                    }
                }
            }
            Serial.println(cmd.command == 's' ? "所有总线执行紧急停止" : "所有总线恢复运行");
        }

        // 2. 执行运动控制（如果不在紧急停止状态）
        readServoNum = 0;
        bool allBusesSuccess = true;
        for (int bus = 0; bus < NUM_SERVO_BUSES; bus++) {
            ServoManager* mgr = sharedData->servoManagers[bus];
            if (!mgr) continue;
            
            if (!mgr->emergencyStop) {
                // 生成随机目标并移动
                int16_t targets[mgr->numServos];
                mgr->GenerateReciprocatingTargets(targets);
                
                // 同步移动当前总线上的舵机
                if (mgr->SyncMoveServos(targets)) {
                    // 读取当前位置负载
                    if (!mgr->SyncReadPosLoad()) {
                        Serial.print("总线");
                        Serial.print(bus);
                        Serial.println("读取舵机状态失败");
                        allBusesSuccess = false;
                    }
                } else {
                    Serial.print("总线");
                    Serial.print(bus);
                    Serial.println("舵机移动失败");
                    allBusesSuccess = false;
                }
            }
            
            // 复制当前总线的舵机状态到汇总数组
            for (uint8_t i = 0; i < mgr->numServos; i++) {
                if (readServoNum < sharedData->totalServoNum) {
                    statusToSend.states[readServoNum] = mgr->servoStates[i];
                    readServoNum++;
                }
            }
        }

        // 3. 汇总所有状态并发送一次
        statusToSend.timestamp = currentTime;
        statusToSend.valid = allBusesSuccess;
        statusToSend.totalServoNum = sharedData->totalServoNum;
        xQueueSend(sharedData->statusQueue, &statusToSend, 0);

        // 4. 这段时间该进程不被调度器执行，直到下一个周期 
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}