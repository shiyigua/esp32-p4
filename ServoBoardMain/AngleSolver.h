#ifndef ANGLE_SOLVER_H
#define ANGLE_SOLVER_H

#include <stdint.h>
#include <math.h>

// 【关键】必须在 pid.h 之前，因为 pid.h 是纯 C 头文件
#ifdef __cplusplus
extern "C" {
#endif
#include "pid.h"
#ifdef __cplusplus
}
#endif

#include "TaskSharedData.h"       // 【新增】引入共享数据定义
#include "ServoBusManager.h"      // 【新增】引入舵机总线管理器

// [新增] 定义关节数量
#define JOINT_COUNT 21
// [新增] 舵机脉冲/角度转换系数 (根据之前文件 STS3215: 4096=360度)
#define STS_STEPS_PER_DEG 11.3777f 


// ============ 关节映射结构 ============
// 【新增】将关节索引(0-20)映射到物理总线和舵机ID
struct JointMapItem {
    uint8_t busIndex;   // 总线编号 0-3
    uint8_t servoID;    // 舵机ID
};


class AngleSolver {
public:
    AngleSolver();

    /**
     * @brief 初始化所有关节的参数
     * @param zeroOffsets 21个关节的零位脉冲数组
     * @param gearRatios 21个关节的减速比数组
     * @param dirs 21个关节的方向数组
     */
    void init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions);

    // [修改] 参数改为接收二维数组，对应两个环的PID参数
    // pidParams[0] 为第一环参数, pidParams[1] 为第二环参数
    // 每个数组包含 6 个元素: Kp, Ki, Kd, Deadband, LimitIntegral, LimitOutput
    void setPIDParams(float pidParams[][PID_PARAMETER_NUM]);
    
    /**
     * @brief [修改] 21轴批量核心解算函数
     * 
     * @param targetDegs     [输入] 21个电机的目标角度 (来自外部规划)
     * @param magActualDegs  [输入] 21个磁编的实际角度 (来自CAN)
     * @param servoActualDegs[输入] 21个舵机的当前反馈角度 (需先转换为度)
     * @param outServoPulses [输出] 21个电机的目标控制值 (脉冲)
     * @return true 计算成功
     */
    bool compute(float* targetDegs, float* magActualDegs, float* servoActualDegs, int16_t* outServoPulses);

    // 重置所有PID
    void resetAll();

private:
    int16_t _zeroOffsets[JOINT_COUNT];
    float   _gearRatios[JOINT_COUNT];
    int8_t  _directions[JOINT_COUNT];

    // [修改] PID 实例数组: [关节ID 0-20][环ID 0=外环, 1=内环]
    PID_Info_TypeDef _pids[JOINT_COUNT][2];

    bool _initialized;
};

// 【新增】全局声明（定义在 SystemTask.cpp 中）
extern AngleSolver  angleSolver;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];

// ============================================================
// 【新增】Solver 任务函数声明
// ============================================================
void taskSolver(void* parameter);

#endif
// // AngleSolver.h
// #ifndef ANGLE_SOLVER_H
// #define ANGLE_SOLVER_H

// #include <stdint.h>
// // 引入上传的 pid 头文件 (注意路径问题，如果放在同级目录直接引用)
// #include "pid.h" 

// // [修改] 脉冲限幅定义 (根据您的增补信息)
// #define MIN_SERVO_PULSE -30719
// #define MAX_SERVO_PULSE  30719

// class AngleSolver {
// public:
//     AngleSolver();

//     /**
//      * @brief 初始化解算参数
//      * @param zeroOffset 机械零位对应的舵机脉冲值 (如 2048)
//      * @param gearRatio 传动比 (舵机转动多少度对应关节转动1度)
//      * @param dir 方向修正 (1 或 -1)
//      */
//     void init(int16_t zeroOffset, float gearRatio, int8_t dir);

//     /**
//      * @brief 设置 PID 参数 (适配 pid.c 的参数结构)
//      * @param kp, ki, kd PID系数
//      * @param maxIntegral 积分限幅
//      * @param maxOutput 输出限幅
//      * @param deadband 死区
//      */
//     void setPID(float kp, float ki, float kd, float maxIntegral, float maxOutput, float deadband = 0.0f);

//     /**
//      * @brief 计算舵机控制量
//      * @param targetDeg 外部输入的目标角度
//      * @param actualDeg CAN反馈的实际角度
//      * @param outputServoPos 输出参数：计算出的舵机目标位置(0-4095)
//      * @return 计算是否有效
//      */
//     bool compute(float targetDeg, float actualDeg, int16_t &outputServoPos);

//     // [修改] compute 函数签名保持不变，但内部逻辑将改变
//     bool compute(float targetDeg_Ext, float actualDeg_Can, int16_t &outServoPulse);

//     // [新增] 接收来自 ServoBusManager 的多圈反馈 (作为辅助或主反馈)
//     // 如果不需要在解算中使用电机反馈而只使用CAN，此函数可主要用于记录日志
//     void setMotorFeedback(int32_t multiTurnPulse);

// private:
//     int16_t _zeroOffset;
//     float   _gearRatio;
//     int8_t  _dir;

//     // 使用 pid.h 中定义的结构体
//     PID_Info_TypeDef _pid;

//     int32_t _currentMotorPulse; // [新增] 存储电机当前的绝对脉冲
// };

// #endif