#ifndef SERVO_BUS_MANAGER_H
#define SERVO_BUS_MANAGER_H

#include <Arduino.h>
#include "SMS_STS.h"

/* ==================== 配置参数 ==================== */

#define NUM_BUSES              4      // 4 条串口总线
#define MAX_SERVOS_PER_BUS     8      // 单总线最大舵机数
#define MAX_SERVO_ID           32     // 支持的最大舵机 ID

/* ==================== 舵机反馈数据（支持多圈） ==================== */

struct ServoFeedback {
    // 原始反馈数据（0-4096）
    int16_t rawPosition;       // 单圈位置 (0-4096)
    int16_t speed;             // 速度（带符号）
    int16_t load;              // 负载（带符号）
    uint8_t voltage;           // 电压 (0.1V)
    uint8_t temperature;       // 温度 (℃)
    
    // 多圈跟踪数据
    int32_t absolutePosition;  // 绝对位置（多圈累计，范围 -30719 到 30719）
    int16_t turnCount;         // 圈数计数器
    int16_t lastRawPosition;   // 上一次的原始位置（用于跨圈检测）
    bool    initialized;       // 是否已初始化
    
    // 状态标志
    bool    online;            // 是否在线
    uint32_t lastUpdate;       // 最后更新时间戳
};

/* ==================== 舵机总线管理器 ==================== */

class ServoBusManager {
public:
    ServoBusManager();

    /* 初始化串口总线 */
    void begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud = 1000000);

    /* ========== 同步写入（控制） ========== */
    
    /**
     * @brief 设置舵机目标位置（加入缓存）
     * @param id 舵机 ID
     * @param position 目标位置 (-30719 到 30719，支持多圈)
     * @param speed 速度
     * @param acc 加速度
     */
    void setTarget(uint8_t id, int16_t position, uint16_t speed = 1000, uint8_t acc = 50);

    /**
     * @brief 同步写入所有缓存的目标位置
     * 调用后会清空缓存
     */
    void syncWriteAll();

    /* ========== 同步读取（反馈） ========== */
    
    /**
     * @brief 同步读取指定舵机列表的位置（自动进行跨圈检测）
     * @param ids 舵机 ID 数组
     * @param count 舵机数量
     * @return 成功读取的舵机数量
     */
    int syncReadPositions(const uint8_t* ids, uint8_t count);

    /**
     * @brief 获取舵机原始位置（0-4096）
     * @param id 舵机 ID
     * @return 原始位置值，失败返回 -1
     */
    int16_t getRawPosition(uint8_t id) const;

    /**
     * @brief 获取舵机绝对位置（多圈累计，-30719 到 30719）
     * @param id 舵机 ID
     * @return 绝对位置值，失败返回 0
     */
    int32_t getAbsolutePosition(uint8_t id) const;

    /**
     * @brief 重置舵机的多圈计数器（将当前位置设为零点）
     * @param id 舵机 ID
     */
    void resetTurnCounter(uint8_t id);

    /**
     * @brief 获取完整的舵机反馈数据
     */
    const ServoFeedback& getFeedback(uint8_t id) const;

    /**
     * @brief 检查舵机是否在线
     */
    bool isOnline(uint8_t id) const;

private:
    SMS_STS _sms;                    // 飞特舵机协议对象
    HardwareSerial* _serial;         // 串口指针

    /* 同步写缓存 */
    uint8_t  _writeIDs[MAX_SERVOS_PER_BUS];
    int16_t  _writePos[MAX_SERVOS_PER_BUS];
    uint16_t _writeSpd[MAX_SERVOS_PER_BUS];
    uint8_t  _writeAcc[MAX_SERVOS_PER_BUS];
    uint8_t  _writeCount;

    /* 反馈数据缓存 */
    ServoFeedback _feedback[MAX_SERVO_ID + 1];

    /* 内部辅助函数 */
    void _updateMultiTurnPosition(uint8_t id, int16_t newRawPos);
};

#endif



// #ifndef SERVO_BUS_MANAGER_H
// #define SERVO_BUS_MANAGER_H

// #include <Arduino.h>
// #include "SMS_STS.h"

// /* ================= 系统配置 ================= */

// #define NUM_BUSES              4
// #define MAX_SERVOS_PER_BUS     8      // 单总线最大舵机数（5 已足够，留余量）
// #define MAX_SERVO_ID           32

// /* ================= 舵机状态 ================= */

// struct ServoFeedback {
//     int16_t position;      // 0~4095
//     int16_t speed;         // 带符号
//     int16_t load;          // 带符号
//     uint8_t voltage;       // 0.1V
//     uint8_t temperature;  // ℃
//     bool    online;
// };

// /* ================= 核心管理类 ================= */

// class ServoBusManager {
// public:
//     ServoBusManager();

//     /* 初始化串口总线 */
//     void begin(uint8_t bus,
//                int rxPin,
//                int txPin,
//                uint32_t baud = 1000000);

//     /* -------- 控制 -------- */
//     void setServoTarget(uint8_t bus,
//                         uint8_t id,
//                         int16_t pos,
//                         uint16_t speed,
//                         uint8_t acc);

//     void syncWriteAll();

//     /* -------- 高效同步读取 -------- */
//     /**
//      * @brief 同步读取一整条总线的舵机反馈
//      * @param bus 总线索引
//      * @param ids 舵机 ID 列表
//      * @param count 舵机数量
//      */
//     void syncReadFeedback(uint8_t bus,
//                           const uint8_t* ids,
//                           uint8_t count);

//     /* 获取缓存的反馈数据 */
//     const ServoFeedback& getFeedback(uint8_t bus, uint8_t id) const;

// private:
//     /* 飞特舵机对象 */
//     SMS_STS sms[NUM_BUSES];
//     HardwareSerial* serials[NUM_BUSES];

//     /* Sync Write 缓冲 */
//     uint8_t  _ids[NUM_BUSES][MAX_SERVOS_PER_BUS];
//     int16_t  _pos[NUM_BUSES][MAX_SERVOS_PER_BUS];
//     uint16_t _spd[NUM_BUSES][MAX_SERVOS_PER_BUS];
//     uint8_t  _acc[NUM_BUSES][MAX_SERVOS_PER_BUS];
//     uint8_t  _count[NUM_BUSES];

//     /* 反馈缓存 */
//     ServoFeedback _feedback[NUM_BUSES][MAX_SERVO_ID + 1];

//     /* 内部函数 */
//     void _parseStatusPacket(uint8_t bus);
// };

// #endif


// /*
//  * ServoBusManager.h
//  * 适配：ESP32-P4 / 多总线 / Feetech STS/SMS
//  * 特性：不修改原库，支持批量读取反馈
//  * 修改：XSimple
//  */

// #ifndef SERVO_BUS_MANAGER_H
// #define SERVO_BUS_MANAGER_H

// #include <Arduino.h>
// #include "SMS_STS.h" // 保持引用原库

// // --- 系统配置 ---
// #define MAX_BUSES            4   // ESP32-P4 支持 Serial1 - Serial4
// #define MAX_SERVOS_PER_BUS   20  // 缓存池大小 (满足每组5个甚至更多)
// #define MAX_SERVO_ID         32  // 支持的最大ID号

// // --- 舵机实时反馈数据结构 ---
// // 用于上层算法检测
// struct ServoState {
//     int16_t  position;    // 当前位置 (0-4095)
//     int16_t  speed;       // 当前速度 (steps/s)
//     int16_t  load;        // 当前负载 (0-1000)
//     uint8_t  voltage;     // 电压 (单位 0.1V)
//     uint8_t  temperature; // 温度 (摄氏度)
//     int16_t  current;     // 电流 (mA, 部分型号支持)
//     bool     isMoving;    // 是否正在移动
//     bool     online;      // 通信是否成功
// };

// // --- 适配器类 (关键) ---
// // 用于在不修改库文件的前提下，访问 protected 的 pSerial
// class TypeSTS : public SMS_STS {
// public:
//     void attachSerial(HardwareSerial* s) {
//         this->pSerial = s; // 访问父类 SCS 的 protected 成员
//     }
// };

// class ServoBusManager {
// public:
//     ServoBusManager();

//     /**
//      * @brief 初始化指定总线
//      * @param busIndex 总线索引 (0-3)
//      * @param rxPin RX引脚
//      * @param txPin TX引脚
//      * @param baudRate 波特率 (默认 1000000)
//      */
//     void begin(uint8_t busIndex, int8_t rxPin, int8_t txPin, unsigned long baudRate = 1000000);

//     // ================= 控制指令 (写入) =================

//     /**
//      * @brief 设置舵机目标 (仅加入缓存，不发送)
//      * @param busIndex 总线索引
//      * @param servoID  舵机ID
//      * @param position 目标位置 (0-4095)
//      * @param speed    运动速度
//      * @param acc      加速度
//      */
//     void setServoTarget(uint8_t busIndex, uint8_t servoID, int16_t position, uint16_t speed, uint8_t acc);

//     /**
//      * @brief 设置轮式/恒速模式 (直接发送)
//      */
//     void setWheelMode(uint8_t busIndex, uint8_t servoID, int16_t speed, uint8_t acc = 0);

//     /**
//      * @brief 同步发送所有缓存指令 (Sync Write)
//      * 极高效率，建议在 loop 结尾调用
//      */
//     void syncWriteAll();

//     // ================= 状态反馈 (读取) =================

//     /**
//      * @brief 读取舵机完整状态 (高效率批量读取)
//      * 用于实时检测，一次通信获取 位置/速度/负载/电压/温度/电流
//      * @return ServoState 结构体
//      */
//     ServoState readServoState(uint8_t busIndex, uint8_t servoID);

//     /**
//      * @brief 简单的 Ping 测试
//      */
//     int ping(uint8_t busIndex, uint8_t servoID);

// private:
//     TypeSTS sms[MAX_BUSES];           // 使用适配器类实例
//     HardwareSerial* serials[MAX_BUSES];

//     // 同步写缓存数组
//     uint8_t  _ids[MAX_BUSES][MAX_SERVOS_PER_BUS]; 
//     int16_t  _pos[MAX_BUSES][MAX_SERVOS_PER_BUS];
//     uint16_t _spd[MAX_BUSES][MAX_SERVOS_PER_BUS];
//     uint8_t  _acc[MAX_BUSES][MAX_SERVOS_PER_BUS];
//     uint8_t  _counts[MAX_BUSES]; 
// };

// #endif

// #ifndef SERVO_BUS_MANAGER_H
// #define SERVO_BUS_MANAGER_H

// #include "SMS_STS.h"
// // #include "AppConfig.h" // 假设包含 NUM_BUSES 等定义
// #include <Arduino.h>
// #include "TaskSharedData.h"

// // [新增] 舵机状态结构体，用于多圈计数
// struct ServoState {
//     int16_t  lastRawPos;     // 上一次读取的原始位置 (0-4095)
//     int32_t  totalTurns;     // 累计圈数
//     int32_t  absolutePulse;  // 累计后的绝对脉冲值
//     bool     isInitialized;  // 是否已初始化
// };

// class ServoBusManager {
// public:
//     ServoBusManager();

//     // [修改] begin函数增加引脚参数，赋默认值
//     // 如果不传参，则需要在 .cpp 中根据 busIndex 使用宏定义
//     void begin(int rxPin , int txPin );


//     // ... 原有的 setServoTarget 等函数 ...
//     void setServoTarget(uint8_t busIndex, uint8_t servoID, int16_t position, uint16_t speed, uint8_t acc);
//     void syncWriteAll();

//     /**
//      * [新增] 读取并更新指定舵机的多圈绝对位置
//      * @param busIndex 总线索引
//      * @param servoID 舵机ID
//      * @return 累加后的绝对脉冲值 (例如 > 4096 或 < 0)
//      */
//     int32_t updateAndGetAbsolutePulse(uint8_t busIndex, uint8_t servoID);

//     // [新增] 获取某个舵机的当前绝对位置（不读取硬件，仅返回缓存）
//     int32_t getStoredAbsolutePulse(uint8_t busIndex, uint8_t servoID);

// private:
//     SMS_STS sms[NUM_BUSES];
//     HardwareSerial* serials[NUM_BUSES];
    
//     // 发送缓冲区
//     uint8_t  _ids[NUM_BUSES][10]; 
//     int16_t  _pos[NUM_BUSES][10];
//     uint16_t _spd[NUM_BUSES][10];
//     uint8_t  _acc[NUM_BUSES][10];
//     uint8_t  _counts[NUM_BUSES];

//     // [新增] 状态存储数组 [总线][舵机ID(假设最大32)]
//     // 注意：如果ID不连续，建议使用 Hash map 或 只是简单的大数组
//     ServoState _servoStates[NUM_BUSES][32]; 
// };

// #endif

// // ServoBusManager.h
// #ifndef SERVO_BUS_MANAGER_H
// #define SERVO_BUS_MANAGER_H

// #include "SMS_STS.h" // 引用您上传的库
// #include "AppConfig.h"
// #include <Arduino.h>

// class ServoBusManager {
// public:
//     ServoBusManager();
//     void begin(); // 初始化串口

//     /**
//      * @brief 发送单个舵机指令（会放入内部缓冲，调用 syncWriteAll 统一发送）
//      */
//     void setServoTarget(uint8_t busIndex, uint8_t servoID, int16_t position, uint16_t speed, uint8_t acc);

//     /**
//      * @brief 立即执行所有总线的同步写操作
//      */
//     void syncWriteAll();

// private:
//     SMS_STS sms[NUM_BUSES];
//     HardwareSerial* serials[NUM_BUSES];
    
//     // 简单的本地缓存，用于组包
//     // [BusIndex][BufferIndex]
//     uint8_t  _ids[NUM_BUSES][10]; 
//     int16_t  _pos[NUM_BUSES][10];
//     uint16_t _spd[NUM_BUSES][10];
//     uint8_t  _acc[NUM_BUSES][10];
//     uint8_t  _counts[NUM_BUSES];
// };

// #endif