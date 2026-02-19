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
