// step_sim.h — 步数/步频真值合成（融合进 Portal libportal.so 的 native 侧）
// 算法移植自 LocationSpoofer：
//   - SensorStepHooker.calculateAutoCadence / calculateCurrentStepsFloat
//   - MotionRealism.Session.steps / accelerometer（proceduralGait 查表波形）
//   - GaitTemplate（可选，用户录制模板）
// 设计原则（与 LocationSpoofer 一致）：步数/步频是「时间的确定性闭式函数」，
// 由 (startTime, stepBase, speed) 即可算出，无需在 system_server 内开定时器，
// 各进程/各连接在同一时刻算出完全相同的值，天然与伪造定位速度匹配。
#pragma once

#include <cstdint>

namespace portal_step {

// 步态真实度等级（对应 MotionRealism.Level）
struct Level {
    int    id;
    double cadenceVariation;        // 步频相对起伏幅度
    double altitudeAmplitudeM;      // 海拔漂移幅度（本模块不用，保留）
    double stepAmplitudeVariation;  // 每步冲击力度差异
    double sensorNoise;             // 加速度计白噪声标准差 m/s^2
};

inline const Level kLevels[] = {
    {0, 0.00, 0.0, 0.00, 0.00},  // OFF
    {1, 0.015, 1.5, 0.05, 0.03}, // LOW
    {2, 0.03, 3.0, 0.10, 0.06},  // MEDIUM
    {3, 0.05, 5.0, 0.18, 0.10},  // HIGH
};

// 步数合成状态（由 JNI 侧写入，hook 侧只读）
struct StepSimState {
    bool     enableStepSim = false;  // 总开关
    bool     autoCadence   = true;   // 是否按速度自动匹配步频
    int      cadenceSpm    = 165;    // 固定步频（autoCadence=false 时生效），60..240
    double   speedMs       = 0.0;    // 当前伪造速度 m/s（与定位同源）
    int64_t  stepBase      = 0;      // 本会话起始累计步数（由 Java 侧 setMotion 下发）
    int64_t  startTimeMs   = 0;      // 本会话开始时刻（0 = 尚未开始）
    double   lastSteps     = 0.0;    // 单调递增地板：上次输出的累计步数（冻结/未开启时返回它，保证永不回退）
    int      speedFluctuationPct = 0; // 速度波动百分比 0..30（完整方案真实度）
    int      realismLevel  = 0;      // MotionRealism.Level.id，默认 OFF
    // 步态模板（可选，64×3 查表，解码自配置 gait_template；空表示用内置波形）
    float    gaitX[64] = {};
    float    gaitY[64] = {};
    float    gaitZ[64] = {};
    int      gaitCadenceSpm = 0;
    bool     hasGaitTemplate = false;
};

// 全局唯一状态实例（定义在 step_sim.cpp）
extern StepSimState gStep;

// 速度(m/s) → 步频(步/分钟) 分段线性映射（SensorStepHooker.calculateAutoCadence）
int cadenceForSpeed(double speedMs);

// 当前累计步数（连续值：整数部分是步数，小数部分是这一步相位）
// MVP 不含速度/步频波动；完整版可在此叠加 SmoothNoise 解析积分。
double stepsAtTime(int64_t nowMs);

// 解析步态模板编码 "v1;cadenceSpm;strideCount;x0,x1,…;y0,…;z0,…" 写入 gStep
// 成功返回 true 并置 hasGaitTemplate；失败/格式不符返回 false（回退内置波形）。
bool decodeGaitTemplate(const char* encoded);

// 合成加速度计三轴读数（含重力，单位 m/s^2）：Z 竖直、Y 前后、X 左右
// stepsFloat: 当前累计步数（连续）；speed: 当前速度；noiseSeed: 供白噪声（可传 0 关闭）
void synthAccel(double stepsFloat, double speed, float out[3], unsigned int noiseSeed);

} // namespace portal_step
