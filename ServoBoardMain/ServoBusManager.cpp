#include "ServoBusManager.h"

/* ==================== 构造函数 ==================== */

ServoBusManager::ServoBusManager() {
    _serial = nullptr;
    _writeCount = 0;

    // 初始化反馈缓存
    for (int i = 0; i <= MAX_SERVO_ID; i++) {
        _feedback[i].online = false;
        _feedback[i].rawPosition = 0;
        _feedback[i].absolutePosition = 0;
        _feedback[i].turnCount = 0;
        _feedback[i].lastRawPosition = 0;
        _feedback[i].initialized = false;
        _feedback[i].lastUpdate = 0;
    }
}

/* ==================== 初始化 ==================== */

void ServoBusManager::begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud) {
    // ESP32-P4 串口映射
    HardwareSerial* s = nullptr;
    switch (busIndex) {
        case 0: s = &Serial1; break;
        case 1: s = &Serial2; break;
        case 2: s = &Serial3; break;
        case 3: s = &Serial4; break;
        default: return;
    }

    _serial = s;
    s->begin(baud, SERIAL_8N1, rxPin, txPin);
    
    // 绑定串口到飞特库
    _sms.pSerial = s;
}

/* ==================== 同步写入 ==================== */

void ServoBusManager::setTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc) {
    if (_writeCount >= MAX_SERVOS_PER_BUS) return;

    // 限制目标位置范围 (-30719 到 30719)
    if (position < -30719) position = -30719;
    if (position > 30719) position = 30719;

    _writeIDs[_writeCount] = id;
    _writePos[_writeCount] = position;
    _writeSpd[_writeCount] = speed;
    _writeAcc[_writeCount] = acc;
    _writeCount++;
}

void ServoBusManager::syncWriteAll() {
    if (_writeCount == 0 || !_serial) return;

    // 调用飞特库的同步写函数
    _sms.SyncWritePosEx(_writeIDs, _writeCount, _writePos, _writeSpd, _writeAcc);
    
    // 清空缓存
    _writeCount = 0;
}

/* ==================== 同步读取（带跨圈检测） ==================== */

int ServoBusManager::syncReadPositions(const uint8_t* ids, uint8_t count) {
    if (!_serial || count == 0) return 0;

    // 飞特舵机内存表地址
    const uint8_t ADDR_PRESENT_POSITION = 56;  // 位置地址
    const uint8_t LEN_POSITION = 2;            // 位置长度（2 字节）

    int successCount = 0;

    // 使用飞特库的同步读功能
    // 1. 初始化同步读
    _sms.syncReadBegin(count, LEN_POSITION, 100);  // 100ms 超时

    // 2. 发送同步读请求
    int ret = _sms.syncReadPacketTx((uint8_t*)ids, count, ADDR_PRESENT_POSITION, LEN_POSITION);
    if (ret != count) {
        _sms.syncReadEnd();
        return 0;
    }

    // 3. 接收并解析每个舵机的返回包
    for (uint8_t i = 0; i < count; i++) {
        uint8_t id = ids[i];
        uint8_t rxBuf[LEN_POSITION];
        
        int rxLen = _sms.syncReadPacketRx(id, rxBuf);
        
        if (rxLen == LEN_POSITION) {
            // 解析位置数据（小端序）
            int16_t rawPos = (rxBuf[1] << 8) | rxBuf[0];
            
            // 更新多圈位置（自动跨圈检测）
            _updateMultiTurnPosition(id, rawPos);
            
            // 更新状态
            _feedback[id].online = true;
            _feedback[id].lastUpdate = millis();
            successCount++;
        } else {
            _feedback[id].online = false;
        }
    }

    // 4. 结束同步读
    _sms.syncReadEnd();

    return successCount;
}

/* ==================== 跨圈检测核心算法 ==================== */

void ServoBusManager::_updateMultiTurnPosition(uint8_t id, int16_t newRawPos) {
    if (id > MAX_SERVO_ID) return;

    ServoFeedback& fb = _feedback[id];
    
    // 首次初始化
    if (!fb.initialized) {
        fb.rawPosition = newRawPos;
        fb.lastRawPosition = newRawPos;
        fb.absolutePosition = newRawPos;
        fb.turnCount = 0;
        fb.initialized = true;
        return;
    }

    // 计算位置变化量
    int16_t delta = newRawPos - fb.lastRawPosition;

    // 跨圈检测阈值（当变化量超过半圈时认为发生了跨圈）
    const int16_t HALF_TURN = 2048;  // 4096 / 2

    if (delta > HALF_TURN) {
        // 反向跨圈（从 4095 -> 0）
        fb.turnCount--;
    } else if (delta < -HALF_TURN) {
        // 正向跨圈（从 0 -> 4095）
        fb.turnCount++;
    }

    // 更新数据
    fb.rawPosition = newRawPos;
    fb.lastRawPosition = newRawPos;
    
    // 计算绝对位置 = 圈数 * 4096 + 当前位置
    fb.absolutePosition = (int32_t)fb.turnCount * 4096 + newRawPos;

    // 限制在有效范围内 (-30719 到 30719)
    if (fb.absolutePosition > 30719) {
        fb.absolutePosition = 30719;
    } else if (fb.absolutePosition < -30719) {
        fb.absolutePosition = -30719;
    }
}

/* ==================== 数据访问 ==================== */

int16_t ServoBusManager::getRawPosition(uint8_t id) const {
    if (id > MAX_SERVO_ID) return -1;
    return _feedback[id].rawPosition;
}

int32_t ServoBusManager::getAbsolutePosition(uint8_t id) const {
    if (id > MAX_SERVO_ID) return 0;
    return _feedback[id].absolutePosition;
}

void ServoBusManager::resetTurnCounter(uint8_t id) {
    if (id > MAX_SERVO_ID) return;
    
    ServoFeedback& fb = _feedback[id];
    fb.turnCount = 0;
    fb.absolutePosition = fb.rawPosition;
}

const ServoFeedback& ServoBusManager::getFeedback(uint8_t id) const {
    static ServoFeedback dummy;
    if (id > MAX_SERVO_ID) return dummy;
    return _feedback[id];
}

bool ServoBusManager::isOnline(uint8_t id) const {
    if (id > MAX_SERVO_ID) return false;
    return _feedback[id].online;
}



// /*
//  * ServoBusManager.cpp
//  * 修改：XSimple
//  */

// #include "ServoBusManager.h"

// // 构造函数
// ServoBusManager::ServoBusManager() {
//     for (int i = 0; i < MAX_BUSES; i++) {
//         _counts[i] = 0;
//         serials[i] = nullptr;
//     }
// }

// // 初始化串口并绑定
// void ServoBusManager::begin(uint8_t busIndex, int8_t rxPin, int8_t txPin, unsigned long baudRate) {
//     if (busIndex >= MAX_BUSES) return;

//     // ESP32-P4 硬件串口映射
//     HardwareSerial* pSer = nullptr;
//     switch (busIndex) {
//         case 0: pSer = &Serial1; break;
//         case 1: pSer = &Serial2; break;
//         case 2: pSer = &Serial3; break;
//         case 3: pSer = &Serial4; break;
//         default: return;
//     }

//     if (pSer) {
//         serials[busIndex] = pSer;
//         // 动态引脚初始化
//         pSer->begin(baudRate, SERIAL_8N1, rxPin, txPin); // ESP32 core API
        
//         // 关键：通过适配器方法注入串口指针
//         sms[busIndex].attachSerial(pSer);
//     }
// }

// // 设置目标位置 (缓存)
// void ServoBusManager::setServoTarget(uint8_t busIndex, uint8_t servoID, int16_t position, uint16_t speed, uint8_t acc) {
//     if (busIndex >= MAX_BUSES) return;
    
//     // 防止数组越界
//     if (_counts[busIndex] < MAX_SERVOS_PER_BUS) {
//         uint8_t idx = _counts[busIndex];
//         _ids[busIndex][idx] = servoID;
//         _pos[busIndex][idx] = position;
//         _spd[busIndex][idx] = speed;
//         _acc[busIndex][idx] = acc;
//         _counts[busIndex]++;
//     }
// }

// // 轮模式控制
// void ServoBusManager::setWheelMode(uint8_t busIndex, uint8_t servoID, int16_t speed, uint8_t acc) {
//     if (busIndex >= MAX_BUSES || serials[busIndex] == nullptr) return;
//     sms[busIndex].WriteSpe(servoID, speed, acc);
// }

// // 同步发送
// void ServoBusManager::syncWriteAll() {
//     for (int i = 0; i < MAX_BUSES; i++) {
//         // 仅当总线有数据且已初始化时发送
//         if (_counts[i] > 0 && serials[i] != nullptr) {
//             sms[i].SyncWritePosEx(_ids[i], _counts[i], _pos[i], _spd[i], _acc[i]);
//             _counts[i] = 0; // 清空缓存
//         }
//     }
// }

// // 高效读取状态
// ServoState ServoBusManager::readServoState(uint8_t busIndex, uint8_t servoID) {
//     ServoState state;
//     // 默认离线状态
//     state.online = false;
//     state.position = -1;
//     state.speed = 0;
//     state.load = 0;
//     state.voltage = 0;
//     state.temperature = 0;
//     state.current = 0;
//     state.isMoving = false;

//     if (busIndex >= MAX_BUSES || serials[busIndex] == nullptr) return state;

//     // FEETECH STS 内存表地址 (参考通用文档)
//     // 56: Present Position (2 byte)
//     // 58: Present Speed (2 byte)
//     // 60: Present Load (2 byte)
//     // 62: Present Voltage (1 byte)
//     // 63: Present Temperature (1 byte)
//     // 64: Reg Write Flag
//     // 65: Limit Flag
//     // 66: Moving Flag (1 byte)
//     // ...
//     // 69: Present Current (2 byte, ST/SMS系列部分支持)
    
//     // 我们从地址 56 开始读取 15 个字节 (覆盖到 70)
//     // 这样比调用 ReadPos, ReadSpeed... 快得多
//     uint8_t buf[16];
//     int count = sms[busIndex].Read(servoID, 56, buf, 15);

//     if (count == 15) {
//         state.online = true;
        
//         // 解算数据 (注意大小端，STS默认小端)
//         // SCS库内部 SCS2Host 处理了大小端，但这里我们直接读的原生字节流，通常是小端 (Low Byte first)
//         // 若使用 TypeSTS (继承自 SCS), 该类 End 默认为 0 (大端)，但 Feetech 硬件通常是小端
//         // 此处手动依小端合成，最稳妥
        
//         state.position = (int16_t)(buf[1] << 8 | buf[0]);
//         // 修正最高位符号 (STS协议：大于32768为负数)
        
//         state.speed    = (int16_t)(buf[3] << 8 | buf[2]);
//         if(state.speed & 0x8000) state.speed = -(state.speed & 0x7FFF);
        
//         state.load     = (int16_t)(buf[5] << 8 | buf[4]);
//         if(state.load & 0x8000) state.load = -(state.load & 0x7FFF);
        
//         state.voltage  = buf[6];
//         state.temperature = buf[7];
        
//         state.isMoving = buf[10]; // Address 66
        
//         state.current  = (int16_t)(buf[14] << 8 | buf[13]); // Address 69
//         if(state.current & 0x8000) state.current = -(state.current & 0x7FFF);
//     }

//     return state;
// }

// int ServoBusManager::ping(uint8_t busIndex, uint8_t servoID) {
//     if (busIndex >= MAX_BUSES || serials[busIndex] == nullptr) return -1;
//     return sms[busIndex].Ping(servoID);
// }

// // ServoBusManager.cpp
// #include "ServoBusManager.h"
// #include "TaskSharedData.h"

// // [新增] 定义半圈阈值，用于判断是否过零
// #define HALF_TURN_THRESHOLD 2048
// #define FULL_TURN_STEPS     4096


// ServoBusManager::ServoBusManager() {
//     serials[0] = &Serial1; // 根据P4定义映射
//     serials[1] = &Serial2;
//     serials[2] = &Serial3;
//     serials[3] = &Serial4;
    
//     for(int i=0; i<NUM_BUSES; i++) _counts[i] = 0;

//        // [新增] 初始化状态数组
//     for(int b=0; b<NUM_BUSES; b++) {
//         for(int s=0; s<32; s++) {
//             _servoStates[b][s].isInitialized = false;
//             _servoStates[b][s].totalTurns = 0;
//             _servoStates[b][s].absolutePulse = 0;
//             _servoStates[b][s].lastRawPos = -1;
//         }
//     }
// }

// void ServoBusManager::begin() {
//     serials[0]->begin(SERVO_BAUDRATE, SERIAL_8N1, BUS0_RX_PIN, BUS0_TX_PIN);
//     serials[1]->begin(SERVO_BAUDRATE, SERIAL_8N1, BUS1_RX_PIN, BUS1_TX_PIN);
//     serials[2]->begin(SERVO_BAUDRATE, SERIAL_8N1, BUS2_RX_PIN, BUS2_TX_PIN);
//     serials[3]->begin(SERVO_BAUDRATE, SERIAL_8N1, BUS3_RX_PIN, BUS3_TX_PIN);

//     for (int i = 0; i < NUM_BUSES; i++) {
//         sms[i].pSerial = serials[i];
//     }
// }

// void ServoBusManager::setServoTarget(uint8_t busIndex, uint8_t servoID, int16_t position, uint16_t speed, uint8_t acc) {
//     if (busIndex >= NUM_BUSES) return;
    
//     uint8_t idx = _counts[busIndex];
//     if (idx >= 10) return; // 防止溢出

//     _ids[busIndex][idx] = servoID;
//     _pos[busIndex][idx] = position;
//     _spd[busIndex][idx] = speed;
//     _acc[busIndex][idx] = acc;
//     _counts[busIndex]++;
// }

// void ServoBusManager::syncWriteAll() {
//     for (int i = 0; i < NUM_BUSES; i++) {
//         if (_counts[i] > 0) {
//             // 调用 SMS_STS 库的同步写
//             sms[i].SyncWritePosEx(_ids[i], _counts[i], _pos[i], _spd[i], _acc[i]);
//             // 清空计数器，准备下一帧
//             _counts[i] = 0;
//         }
//     }
// }


// // [新增] 核心多圈解算逻辑
// int32_t ServoBusManager::updateAndGetAbsolutePulse(uint8_t busIndex, uint8_t servoID) {
//     if (busIndex >= NUM_BUSES || servoID >= 32) return 0;

//     // 1. 读取当前原始位置 (0-4095)
//     int rawPos = sms[busIndex].ReadPos(servoID);
    
//     if (rawPos == -1) {
//         return _servoStates[busIndex][servoID].absolutePulse; // 读取失败返回旧值
//     }

//     ServoState &state = _servoStates[busIndex][servoID];

//     // 2. 如果是首次运行，直接对齐
//     if (!state.isInitialized) {
//         state.lastRawPos = rawPos;
//         state.absolutePulse = rawPos; // 初始时刻默认为0圈，位置即为当前值
//         // 如果电机上电时不在0位，这里可能需要Offset逻辑，暂按原始值处理
//         state.isInitialized = true;
//         return state.absolutePulse;
//     }

//     // 3. 计算差值 (当前 - 上次)
//     int diff = rawPos - state.lastRawPos;

//     // 4. 过零检测逻辑
//     // 如果差值突变很大 (例如从 4090 变到 10，差值 -4080)，说明正向过零
//     if (diff < -HALF_TURN_THRESHOLD) {
//         state.totalTurns++;  // 正向增加一圈
//         diff += FULL_TURN_STEPS; // 修正差值用于平滑计算（可选）
//     } 
//     // 如果差值突变很大 (例如从 10 变到 4090，差值 +4080)，说明反向过零
//     else if (diff > HALF_TURN_THRESHOLD) {
//         state.totalTurns--;  // 反向减少一圈
//         diff -= FULL_TURN_STEPS;
//     }

//     // 5. 更新状态
//     state.lastRawPos = rawPos;
    
//     // 计算绝对位置 = 圈数 * 4096 + 当前原始位置
//     state.absolutePulse = (state.totalTurns * FULL_TURN_STEPS) + rawPos;

//     return state.absolutePulse;
// }

// // [新增] 获取缓存值
// int32_t ServoBusManager::getStoredAbsolutePulse(uint8_t busIndex, uint8_t servoID) {
//     if (busIndex >= NUM_BUSES || servoID >= 32) return 0;
//     return _servoStates[busIndex][servoID].absolutePulse;
// }