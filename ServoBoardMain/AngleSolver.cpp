#include "AngleSolver.h"
#include <string.h> // for memset if needed
#include "pid.h"
// ============================================================
// 【新增】外部引用（定义在 SystemTask.cpp 中）
// ============================================================
extern ServoBusManager servoBus0;
extern ServoBusManager servoBus1;
extern ServoBusManager servoBus2;
extern ServoBusManager servoBus3;
extern AngleSolver angleSolver;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];

// ============================================================
// 原有 AngleSolver 类实现（保持不变）
// ============================================================

AngleSolver::AngleSolver() : _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_pids, 0, sizeof(_pids));
}

void AngleSolver::init(int16_t *zeroOffsets, float *gearRatios, int8_t *directions)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        _zeroOffsets[i] = zeroOffsets[i];
        _gearRatios[i] = gearRatios[i];
        _directions[i] = directions[i];
    }
    // resetAll();
    _initialized = true;
}

// void AngleSolver::resetAll()
// {
//     for (int i = 0; i < JOINT_COUNT; i++)
//     {
//         PID_Reset(&_pids[i][0]);
//         PID_Reset(&_pids[i][1]);
//     }
// }

void AngleSolver::setPIDParams(float pidParams[][PID_PARAMETER_NUM])
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);
        PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
    }
}

// 核心批量解算逻辑（保持不变）
bool AngleSolver::compute(float *targetDegs, float *magActualDegs,
                          float *servoActualDegs, int16_t *outServoPulses)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        // --- 第一环 (外环: 位置环) ---
        // 目标: 上位机规划角度
        // 实际: 磁编角度
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        // --- 第二环 (内环: 舵机环) ---
        float loop2_Target = _pids[i][0].Output + servoActualDegs[i];
        float loop2_Actual = servoActualDegs[i];
        f_PID_Calculate(&_pids[i][1], loop2_Target, loop2_Actual);

        // 输出脉冲
        outServoPulses[i] = (int16_t)_pids[i][1].Output;
    }
    return true;
}

// ============================================================
// 【新增】辅助函数：CAN 原始值转角度 (0-16383 -> 0-360°)
// ============================================================
static float convertCanToDeg(uint16_t rawValue)
{
    return (float)rawValue * 360.0f / 16384.0f;
}

// ============================================================
// 【新增】辅助函数：根据总线编号获取 ServoBusManager 指针
// ============================================================
static ServoBusManager *getBusByIndex(uint8_t busIndex)
{
    switch (busIndex)
    {
    case 0:
        return &servoBus0;
    case 1:
        return &servoBus1;
    case 2:
        return &servoBus2;
    case 3:
        return &servoBus3;
    default:
        return NULL;
    }
}

// ============================================================
// 【新增】taskSolver — 角度解算 + 电机控制任务
// ============================================================
// 数据流:
//   目标角度 ← sharedData.targetAngles[] (由 UpperCommTask 写入)
//   磁编角度 ← sharedData.canRxQueue     (由 CanCommTask 写入)
//   舵机反馈 ← ServoBusManager.readPosition()
//   输出脉冲 → ServoBusManager.writePosition()
// ============================================================

void taskSolver(void *parameter)
{

    // TaskSharedData_t *sharedData = (TaskSharedData_t *)parameter;
      TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;

    // 定义每条总线的舵机 ID 列表（根据你的配置：4+4+4+5）
    const uint8_t bus0_ids[] = {1, 2, 3, 4};    // 总线0: 4个舵机
    const uint8_t bus1_ids[] = {1, 2, 3, 4};    // 总线1: 4个舵机
    const uint8_t bus2_ids[] = {1, 2, 3, 4};    // 总线2: 4个舵机
    const uint8_t bus3_ids[] = {1, 2, 3, 4, 5}; // 总线3: 5个舵机

    // 本地数据缓冲区
    float localTargets[ENCODER_TOTAL_NUM];
    float canAngles[ENCODER_TOTAL_NUM];
    float servoAngles[ENCODER_TOTAL_NUM];
    int16_t outPulses[ENCODER_TOTAL_NUM];
    RemoteSensorData_t sensorData;
    while (1)
    {
        // ========================================
        // 步骤 1: 同步读取所有舵机位置（自动跨圈检测）
        // ========================================
        servoBus0.syncReadPositions(bus0_ids, 4);
        servoBus1.syncReadPositions(bus1_ids, 4);
        servoBus2.syncReadPositions(bus2_ids, 4);
        servoBus3.syncReadPositions(bus3_ids, 5);

        // ========================================
        // 步骤 2: 获取多圈绝对位置并转换为角度
        // ========================================
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager *pBus = getBusByIndex(bus);

            if (pBus && pBus->isOnline(id))
            {
                // 使用多圈绝对位置（-30719 到 30719）
                int32_t absPos = pBus->getAbsolutePosition(id);
                // 转换为角度（每圈 4096 步 = 360°）
                servoAngles[i] = (float)absPos * 360.0f / 4096.0f;
            }
            else
            {
                servoAngles[i] = 0.0f;
            }
        }

        // ========================================
        // 步骤 3: 读取 CAN 磁编角度
        // ========================================
        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE)
        {
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                canAngles[i] = convertCanToDeg(sensorData.encoderValues[i]);
            }
        }

        // ========================================
        // 步骤 4: 读取目标角度
        // ========================================
        if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localTargets, sharedData->targetAngles, sizeof(localTargets));
            xSemaphoreGive(sharedData->targetAnglesMutex);
        }

        // ========================================
        // 步骤 5: PID 解算
        // ========================================
        angleSolver.compute(localTargets, canAngles, servoAngles, outPulses);

        // ========================================
        // 步骤 6: 同步写入所有舵机
        // ========================================
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            uint8_t bus = jointMap[i].busIndex;
            uint8_t id = jointMap[i].servoID;
            ServoBusManager *pBus = getBusByIndex(bus);

            if (pBus)
            {
                // outPulses[i] 范围：-30719 到 30719
                int16_t targetPos = constrain(outPulses[i], -30719, 30719);
                pBus->setTarget(id, targetPos, 1000, 50);
            }
        }

        // 统一发送
        servoBus0.syncWriteAll();
        servoBus1.syncWriteAll();
        servoBus2.syncWriteAll();
        servoBus3.syncWriteAll();

        // ========================================
        // 步骤 7: 控制周期延时（100Hz）
        // ========================================
        vTaskDelay(pdMS_TO_TICKS(10));

        // // ========================================
        // // 步骤 1: 同步读取所有舵机位置（自动跨圈检测）
        // // ========================================
        // servoBus0.syncReadPositions(bus0_ids, 4);
        // servoBus1.syncReadPositions(bus1_ids, 4);
        // servoBus2.syncReadPositions(bus2_ids, 4);
        // servoBus3.syncReadPositions(bus3_ids, 5);

        // // ========================================
        // // 步骤 2: 获取多圈绝对位置并转换为角度
        // // ========================================
        // for (int i = 0; i < JOINT_COUNT; i++) {
        //     uint8_t bus = jointMap[i].busIndex;
        //     uint8_t id  = jointMap[i].servoID;
        //     ServoBusManager* pBus = getBusByIndex(bus);

        //     if (pBus && pBus->isOnline(id)) {
        //         // 使用多圈绝对位置（-30719 到 30719）
        //         int32_t absPos = pBus->getAbsolutePosition(id);

        //         // 转换为角度（每圈 4096 步 = 360°）
        //         servoAngles[i] = (float)absPos * 360.0f / 4096.0f;
        //     } else {
        //         servoAngles[i] = 0.0f;
        //     }
        // }

        // // ========================================
        // // 步骤 3: 读取 CAN 磁编角度
        // // ========================================
        // if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE) {
        //     for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        //         canAngles[i] = convertCanToDeg(sensorData.encoderValues[i]);
        //     }
        // }

        // // ========================================
        // // 步骤 4: PID 解算
        // // ========================================
        // angleSolver.compute(localTargets, canAngles, servoAngles, outPulses);

        // // ========================================
        // // 步骤 5: 同步写入（支持多圈目标位置）
        // // ========================================
        // for (int i = 0; i < JOINT_COUNT; i++) {
        //     uint8_t bus = jointMap[i].busIndex;
        //     uint8_t id  = jointMap[i].servoID;
        //     ServoBusManager* pBus = getBusByIndex(bus);

        //     if (pBus) {
        //         // outPulses[i] 范围：-30719 到 30719
        //         int16_t targetPos = constrain(outPulses[i], -30719, 30719);
        //         pBus->setTarget(id, targetPos, 1000, 50);
        //     }
        // }

        // servoBus0.syncWriteAll();
        // servoBus1.syncWriteAll();
        // servoBus2.syncWriteAll();
        // servoBus3.syncWriteAll();

        // vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz 控制频率
    }
}

// void taskSolver(void* parameter) {
//     TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;

//     // 本地数据缓冲区
//     float localTargets[ENCODER_TOTAL_NUM];
//     float canAngles[ENCODER_TOTAL_NUM];
//     float servoAngles[ENCODER_TOTAL_NUM];
//     int16_t outPulses[ENCODER_TOTAL_NUM];

//     // 初始化为 0
//     memset(localTargets, 0, sizeof(localTargets));
//     memset(canAngles, 0, sizeof(canAngles));
//     memset(servoAngles, 0, sizeof(servoAngles));
//     memset(outPulses, 0, sizeof(outPulses));

//     RemoteSensorData_t sensorData;

//     // 控制周期 10ms (100Hz)
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xPeriod = pdMS_TO_TICKS(10);

//     while (1) {
//         vTaskDelayUntil(&xLastWakeTime, xPeriod);

//         // ========================================
//         // 步骤 1: 获取目标角度 (来自上位机)
//         // ========================================
//         if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
//             memcpy(localTargets, sharedData->targetAngles, sizeof(localTargets));
//             xSemaphoreGive(sharedData->targetAnglesMutex);
//         }

//         // ========================================
//         // 步骤 2: 获取磁编实际角度 (来自 CAN)
//         // ========================================
//         if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE) {
//             for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
//                 canAngles[i] = convertCanToDeg(sensorData.encoderValues[i]);
//             }
//         }

//         // ========================================
//         // 步骤 3: 获取舵机当前位置反馈
//         // ========================================
//         for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
//             uint8_t bus = jointMap[i].busIndex;
//             uint8_t id  = jointMap[i].servoID;
//             ServoBusManager* pBus = getBusByIndex(bus);

//             if (pBus) {
//                 int16_t pos = pBus->readPosition(id);
//                 if (pos != -1) {
//                     // 舵机原始位置(0-4095) 转角度(0-360°)
//                     servoAngles[i] = (float)pos * 360.0f / 4096.0f;
//                 }
//             }
//         }

//         // ========================================
//         // 步骤 4: 双环 PID 解算
//         // ========================================
//         angleSolver.compute(localTargets, canAngles, servoAngles, outPulses);

//         // ========================================
//         // 步骤 5: 将解算结果写入舵机
//         // ========================================
//         for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
//             uint8_t bus = jointMap[i].busIndex;
//             uint8_t id  = jointMap[i].servoID;
//             ServoBusManager* pBus = getBusByIndex(bus);

//             if (pBus) {
//                 // outPulses[i] 是 PID 输出的脉冲值，直接写入舵机位置
//                 int16_t targetPos = constrain(outPulses[i], 0, 4095);
//                 pBus->writePosition(id, targetPos);
//             }
//         }
//     }
// }

// #include "AngleSolver.h"
// #include <string.h> // for memset if needed

// AngleSolver::AngleSolver() {
//     // 默认初始化，防止指针访问野数据
//     for(int i=0; i<JOINT_COUNT; i++) {
//         _zeroOffsets[i] = 2048;
//         _gearRatios[i] = 1.0f;
//         _dirs[i] = 1;
//     }
// }

// void AngleSolver::init(int16_t* zeroOffsets, float* gearRatios, int8_t* dirs) {
//     if(zeroOffsets) memcpy(_zeroOffsets, zeroOffsets, sizeof(_zeroOffsets));
//     if(gearRatios)  memcpy(_gearRatios, gearRatios, sizeof(_gearRatios));
//     if(dirs)        memcpy(_dirs, dirs, sizeof(_dirs));
// }

// void AngleSolver::setPIDParams(float pidParams[2][PID_PARAMETER_NUM]) {

//     // 遍历所有 21 个关节进行初始化
//     for (int i = 0; i < JOINT_COUNT; i++) {
//         // [修改] 使用类成员 _pids，而不是局部变量

//         // 初始化第一环 (位置环/磁编环)
//         // 使用传入的 pidParams[0]
//         PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);

//         // 初始化第二环 (舵机环)
//         // 使用传入的 pidParams[1]
//         PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
//     }
// }
// // [修改] 核心批量解算逻辑
// bool AngleSolver::compute(float* targetDegs, float* magActualDegs, float* servoActualDegs, int16_t* outServoPulses) {

//     for (int i = 0; i < JOINT_COUNT; i++) {
//         // --- 第一环 (Outer Loop) ---
//         // 目标: 外部规划角度
//         // 实际: 磁编角度
//         // 输出: 角度误差补偿量 (Delta Angle)
//         f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

//         // --- 第二环 (Inner Loop / Servo Loop) ---
//         // 根据您的要求：
//         // 目标值 = 第一环输出 + 舵机当前角度
//         // // 实际值 = 舵机当前角度
//         // float loop2_Target = out1_Correction + servoActualDegs[i];
//         // float loop2_Actual = servoActualDegs[i];

//         // 计算第二环
//         // 注意：这里输出的是 PID 计算结果。
//         // 如果舵机控制模式是位置模式，PID输出通常还要处理成绝对位置及增加前馈，
//         // 但此处严格遵循您的逻辑：PID计算结果直接存入输出。
//         float out2_Result = f_PID_Calculate(&_pids[i][1], _pids[i][0].Output+servoActualDegs[i], servoActualDegs[i]);

//         // --- 输出转换 ---
//         // 假设第二环输出的是“角度域”的结果，需要转换为“脉冲域”发送给电机
//         // 加上零位偏置并进行方向/减速比处理

//         // 逻辑：目标脉冲 = 零位 + (PID计算结果 * 步进系数 * 减速比 * 方向)
//         int32_t finalPulse = /*_zeroOffsets[i] +*/ (int32_t)(out2_Result * STS_STEPS_PER_DEG * _gearRatios[i] * _dirs[i]);

//         // 限幅保护 (根据之前的修改，范围为 -30719 到 30719)
//         if (finalPulse > 30719) finalPulse = 30719;
//         if (finalPulse < -30719) finalPulse = -30719;

//         outServoPulses[i] = (int16_t)finalPulse;
//     }

//     return true;
// }

// // AngleSolver.cpp
// #include "AngleSolver.h"
// #include <math.h>

// // 舵机常数：STS3215等通常 4096步 = 360度 -> 1度 ≈ 11.377步
// #define STEPS_PER_DEG 11.377f

// AngleSolver::AngleSolver() :
//     _zeroOffset(2048), _gearRatio(1.0f), _dir(1)
// {
//     // 构造函数中可以给PID结构体清零，但主要初始化由 setPID 完成
// }

// void AngleSolver::init(int16_t zeroOffset, float gearRatio, int8_t dir) {
//     _zeroOffset = zeroOffset;
//     _gearRatio = gearRatio;
//     _dir = dir;
// }

// void AngleSolver::setPID(float kp, float ki, float kd, float maxIntegral, float maxOutput, float deadband) {
//     // 准备参数数组，对应 pid.c 中的 float para[6]
//     // Order: Kp, Ki, Kd, Deadband, limitIntegral, limitOutput
// PID_Info_TypeDef Finger_Pid[21][2];
// //P I D DeadBand IntegeralMAX OutputMAX
// float Finger_Pid_Para[4][PID_PARAMETER_NUM] = {
//     [0] = {20, 0, 0, 0, 0, 30000},       // motor angle
//     [1] = {5, 0, 0, 0, 0, 30719},       // motor  speed
//     };
//     // PID_POSITION 代表位置式PID (根据您的需求选择)
//     for (int i = 0; i < 21; i++) {
//         PID_Init(&Finger_Pid[i][0], PID_POSITION, Finger_Pid_Para[0]);
//         PID_Init(&Finger_Pid[i][1], PID_POSITION, Finger_Pid_Para[1]);
//     }

// }

// bool AngleSolver::compute(float targetDeg, float actualDeg, int16_t &outputServoPos) {

//     // 1. 调用 pid.c 的核心计算函数
//     // f_PID_Calculate(结构体指针, 目标值, 测量值)
//     // 这里的返回值就是 PID 输出的补偿值 (Output)
//     for(int i = 0; i < 21; i++) {
//         f_PID_Calculate(&Finger_Pid[i][0], targetDeg, actualDeg);
//         f_PID_Calculate(&Finger_Pid[i][1], Finger_Pid[i][0].Output+_currentMotorPulse, _currentMotorPulse);
//     }

//     // 注意：pid.c 的实现中如果发生错误(如配置错误)会返回0或清除计算
//     // 可以检查 _pid.ERRORHandler.Status，这里简化处理

//     // 2. 将 PID 输出叠加到前馈控制上
//     // 理论上舵机应该在该角度：targetDeg
//     // 实际上可能有负载误差，pidOutput 用于修正这个误差
//     // newTarget = mechanical_target + pid_correction
//     // float finalDeg = targetDeg + pidOutput;

//     // 3. 映射到舵机脉冲空间
//     // 逻辑：舵机位置 = 零位 + (修正后的角度 * 传动比 * 步进系数 * 方向)
//     int32_t calcPos = _zeroOffset + (int32_t)(targetDeg * _gearRatio * STEPS_PER_DEG * _dir);

//     // 4. 舵机硬件限位保护 (0-4095)
//     // if (calcPos < 0) calcPos = 0;
//     // if (calcPos > 4095) calcPos = 4095;

//     // outputServoPos = (int16_t)calcPos;
//     return true;
// }