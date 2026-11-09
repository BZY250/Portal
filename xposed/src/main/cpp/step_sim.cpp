// step_sim.cpp — 步数/步频真值合成实现（移植 LocationSpoofer 的 MotionRealism/SensorStepHooker）
#include "step_sim.h"
#include <cmath>
#include <cstdint>
#include <cstdio>    // sscanf
#include <cstdlib>   // strtof

namespace portal_step {

StepSimState gStep;

namespace {

constexpr double kPi        = 3.14159265358979323846;
constexpr float  kGravity   = 9.80665f;
constexpr double kRunThresh = 2.2;    // 速度 >= 2.2 m/s 视为跑步
constexpr int    kTableSize = 256;

// ---- 分段线性步频映射（与 SensorStepHooker.calculateAutoCadence 完全一致）----
int cadenceForSpeedImpl(double v) {
    if (v <= 0.8) return 95;                                                     // 慢走
    if (v <= 1.5) return (int)(100 + (v - 0.8) / 0.7 * 25);                      // 快走 100~125
    if (v <= 2.8) return (int)(135 + (v - 1.5) / 1.3 * 25);                      // 慢跑 135~160
    if (v <= 4.0) return (int)(160 + (v - 2.8) / 1.2 * 20);                      // 匀速跑 160~180
    if (v <= 5.5) return (int)(180 + (v - 4.0) / 1.5 * 15);                      // 快跑 180~195
    return 200;                                                                  // 冲刺
}

// ---- 步态波形：bump / 零均值查表 / 线性插值（MotionRealism 同款）----
double bump(double p, double center, double width) {
    double d = std::fabs(p - center);
    if (d > 0.5) d = 1.0 - d;                 // 周期回绕（相位差）
    return std::exp(-0.5 * (d / width) * (d / width));
}

// 采样 256 点并去均值：真实步态周期内加速度平均值必须等于重力，否则长时间积分暴露
void zeroMeanTable(double (*shape)(double), float out[kTableSize]) {
    double raw[kTableSize];
    double mean = 0.0;
    for (int i = 0; i < kTableSize; ++i) {
        raw[i] = shape((double)i / kTableSize);
        mean  += raw[i];
    }
    mean /= kTableSize;
    for (int i = 0; i < kTableSize; ++i) out[i] = (float)(raw[i] - mean);
}

// 步行：脚跟着地冲击 → 支撑期回落 → 蹬地第二峰 → 摆动期低谷
double walkVertical(double p) {
    return 1.0 * bump(p, 0.00, 0.045) - 0.45 * bump(p, 0.13, 0.07)
         + 0.55 * bump(p, 0.42, 0.09) - 0.65 * bump(p, 0.72, 0.13);
}
double walkForward(double p) {
    return -0.5 * bump(p, 0.03, 0.05) + 0.45 * bump(p, 0.40, 0.09) - 0.15 * bump(p, 0.75, 0.15);
}
// 跑步：尖锐落地冲击 + 支撑期主峰 + 腾空期接近失重
double runVertical(double p) {
    return 1.0 * bump(p, 0.00, 0.035) + 0.55 * bump(p, 0.16, 0.09) - 0.85 * bump(p, 0.62, 0.18);
}
double runForward(double p) {
    return -0.6 * bump(p, 0.02, 0.04) + 0.5 * bump(p, 0.25, 0.08);
}

// 惰性构建的查表（首次使用时填充）
struct GaitTables {
    float walkV[kTableSize], walkF[kTableSize], runV[kTableSize], runF[kTableSize];
    GaitTables() {
        zeroMeanTable(walkVertical, walkV);
        zeroMeanTable(walkForward, walkF);
        zeroMeanTable(runVertical,  runV);
        zeroMeanTable(runForward,  runF);
    }
};
const GaitTables& tables() { static GaitTables t; return t; }

float lookup(const float table[kTableSize], double phase) {
    double pos = (phase - std::floor(phase)) * kTableSize;
    int i = ((int)pos) % kTableSize;
    double frac = pos - std::floor(pos);
    int j = (i + 1) % kTableSize;
    return (float)(table[i] * (1.0 - frac) + table[j] * frac);
}

// 由 (seed, n) 确定的 [-1,1] 伪随机数：同一步在不同进程里算出同样的力度（hashUnit）
double hashUnit(int64_t seed, int64_t n) {
    uint64_t x = (uint64_t)seed ^ ((uint64_t)n * 0x9E3779B97F4A7C15ULL);
    x = (x ^ (x >> 33)) * 0xC2B2AE3D27D4EB4FULL;
    x = (x ^ (x >> 33)) * 0x165667B19E3779F9ULL;
    x = x ^ (x >> 33);
    return (double)(x >> 11) / (double)(1ULL << 53) * 2.0 - 1.0;
}

// ---- 完整方案：速度/步频波动（MotionRealism.SmoothNoise 同款，闭式可积）----
// java.util.Random 的 48 位 LCG，保证与 LocationSpoofer 的 Kotlin 端用同一套种子序列
class JavaRandom {
    uint64_t seed;
public:
    explicit JavaRandom(int64_t s) : seed((uint64_t)s ^ 0x5DEECE66DULL) {}   // Kotlin Random(seed) 的初值
    int next(int bits) {
        seed = (seed * 0x5DEECE66DULL + 0xBULL) & 0xFFFFFFFFFFFFULL;
        return (int)(seed >> (48 - bits));
    }
    double nextDouble() {
        return ((((uint64_t)next(26)) << 27) + (uint64_t)next(27)) / (double)(1ULL << 53);
    }
};

// 相位随机的多正弦叠加：值 ∈ [-1,1]，积分有解析解 → 各进程对任意 t 独立算出完全一致
class SmoothNoise {
    double periods[4], phases[4], weights[4];
public:
    SmoothNoise(int64_t seed, double minPeriodSec, double maxPeriodSec, int components = 4) {
        JavaRandom rnd(seed);
        if (components > 4) components = 4;
        double raw[4], sum = 0.0;
        for (int i = 0; i < components; ++i) {
            periods[i] = minPeriodSec * std::pow(maxPeriodSec / minPeriodSec, rnd.nextDouble());
            phases[i]  = rnd.nextDouble() * 2.0 * kPi;
            raw[i]     = 0.5 + rnd.nextDouble();
            sum       += raw[i];
        }
        for (int i = 0; i < components; ++i) weights[i] = raw[i] / sum;
        for (int i = components; i < 4; ++i) { periods[i] = 1.0; phases[i] = 0.0; weights[i] = 0.0; }
    }
    double value(double tSec) const {
        double v = 0.0;
        for (int i = 0; i < 4; ++i) if (weights[i] != 0.0)
            v += weights[i] * std::sin(2.0 * kPi * tSec / periods[i] + phases[i]);
        return v;
    }
    double integral(double tSec) const {   // ∫₀ᵗ value(τ) dτ
        double v = 0.0;
        for (int i = 0; i < 4; ++i) if (weights[i] != 0.0) {
            double w = 2.0 * kPi / periods[i];
            v += weights[i] / w * (std::cos(phases[i]) - std::cos(w * tSec + phases[i]));
        }
        return v;
    }
};

// 会话噪声缓存（按 startTimeMs/level/pct 变化时重建，避免高频 hook 反复初始化）
static constexpr int64_t kSpeedNoiseSalt = 0x5EED5EEDL;
static constexpr int64_t kCadenceNoiseSalt = 0x0CADE0CEL;
static int64_t  g_cachedStartMs = -1;
static int      g_cachedLevel = -1;
static int      g_cachedPct = -1;
static SmoothNoise g_speedNoise(0, 20.0, 600.0);
static SmoothNoise g_cadenceNoise(0, 3.0, 60.0);

static void ensureSession(const StepSimState& s) {
    if (s.startTimeMs == g_cachedStartMs && s.realismLevel == g_cachedLevel
        && s.speedFluctuationPct == g_cachedPct) return;
    g_cachedStartMs = s.startTimeMs;
    g_cachedLevel   = s.realismLevel;
    g_cachedPct     = s.speedFluctuationPct;
    g_speedNoise   = SmoothNoise(s.startTimeMs ^ kSpeedNoiseSalt, 20.0, 600.0);
    g_cadenceNoise = SmoothNoise(s.startTimeMs ^ kCadenceNoiseSalt, 3.0, 60.0);
}

} // namespace

int cadenceForSpeed(double speedMs) { return cadenceForSpeedImpl(speedMs); }

bool decodeGaitTemplate(const char* encoded) {
    if (encoded == nullptr) return false;
    StepSimState& s = gStep;

    // 拆分为 "v1" / cadenceSpm / strideCount / x / y / z 六段
    const char* parts[6];
    int nparts = 0;
    const char* p = encoded;
    while (nparts < 6) {
        parts[nparts++] = p;
        const char* q = p;
        while (*q != '\0' && *q != ';') ++q;
        if (*q == '\0') break;              // 段数不足
        p = q + 1;
    }
    if (nparts != 6) return false;
    if (parts[0][0] != 'v' || parts[0][1] != '1') return false;   // 版本前缀 "v1"（后续 sscanf/strtof 会自然停在 ';'/',' 上）

    int cadenceSpm = 0, strideCount = 0;
    if (std::sscanf(parts[1], "%d", &cadenceSpm) != 1) return false;
    if (std::sscanf(parts[2], "%d", &strideCount) != 1) return false;

    // 解析 3 轴 × 64 点
    float* axes[3] = { s.gaitX, s.gaitY, s.gaitZ };
    for (int a = 0; a < 3; ++a) {
        const char* seg = parts[3 + a];
        for (int i = 0; i < 64; ++i) {
            char* end = nullptr;
            float v = std::strtof(seg, &end);
            if (end == seg) return false;   // 解析失败
            axes[a][i] = v;
            seg = (*end == ',') ? end + 1 : end;
        }
    }
    s.gaitCadenceSpm  = cadenceSpm;
    s.hasGaitTemplate = true;
    return true;
}

double stepsAtTime(int64_t nowMs) {
    StepSimState& s = gStep;
    if (!s.enableStepSim) return s.lastSteps;          // 未开启：保持最后值
    if (s.speedMs <= 0.05) return s.lastSteps;         // 静止：冻结，绝不回退
    if (s.startTimeMs == 0) { s.startTimeMs = nowMs; } // 首次事件：以当前时刻为起点

    double elapsedSec = (nowMs - s.startTimeMs) / 1000.0;
    if (elapsedSec < 0.0) elapsedSec = 0.0;

    double cadence = s.autoCadence ? (double)cadenceForSpeedImpl(s.speedMs)
                                   : (double)s.cadenceSpm;

    double steps;
    const Level& lv = kLevels[(s.realismLevel >= 0 && s.realismLevel <= 3) ? s.realismLevel : 0];
    double fluct = (s.speedFluctuationPct < 0 ? 0 : (s.speedFluctuationPct > 30 ? 30 : s.speedFluctuationPct)) / 100.0;
    if (fluct > 0.0 || lv.cadenceVariation > 0.0) {
        // 完整方案：steps = base + C/60·[ t + 0.4·fluct·∫N_speed + σ_cad·∫N_cad ]（闭式积分）
        ensureSession(s);
        constexpr double kCadenceShareOfSpeed = 0.4;
        double tTerm = elapsedSec
                     + kCadenceShareOfSpeed * fluct * g_speedNoise.integral(elapsedSec)
                     + lv.cadenceVariation * g_cadenceNoise.integral(elapsedSec);
        steps = (double)s.stepBase + cadence / 60.0 * tTerm;
    } else {
        // MVP：匀速匀步频
        steps = (double)s.stepBase + cadence / 60.0 * elapsedSec;
    }
    if (steps < s.lastSteps) steps = s.lastSteps;      // 单调性保护
    s.lastSteps = steps;
    return steps;
}

void synthAccel(double stepsFloat, double speed, float out[3], unsigned int noiseSeed) {
    StepSimState& s = gStep;
    const Level& level = kLevels[(s.realismLevel >= 0 && s.realismLevel <= 3) ? s.realismLevel : 0];

    int64_t stepIndex = (int64_t)std::floor(stepsFloat);
    double  stepPhase = stepsFloat - (double)stepIndex;
    double  strength  = 1.0 + level.stepAmplitudeVariation * hashUnit(/*seed*/0x57E9057EL, stepIndex);

    if (s.hasGaitTemplate) {
        // 用户录制模板：跨步（左右各一步）为一个周期，64 点，线性插值 + 保留重力均值 + strength 缩放
        double stridePhase = stepsFloat / 2.0;                       // steps → 跨步相位
        double pos = (stridePhase - std::floor(stridePhase)) * 64;
        int i = ((int)pos) % 64, j = (i + 1) % 64;
        double frac = pos - std::floor(pos);
        // 简单实现：直接三轴插值（模板已含重力均值，缩放动态分量）
        auto interp = [&](const float a[64]) {
            return a[i] * (1.0 - frac) + a[j] * frac;
        };
        // 模板均值代表重力方向，动态分量按 strength 缩放（此处对整轴做近似：保留均值）
        float x = interp(s.gaitX), y = interp(s.gaitY), z = interp(s.gaitZ);
        out[0] = x; out[1] = y; out[2] = z;
    } else {
        const GaitTables& t = tables();
        bool running = speed >= kRunThresh;
        double amplitude = running ? (6.0 + 1.5 * speed) : (2.0 + 1.0 * speed);
        float vertical = lookup(running ? t.runV : t.walkV, stepPhase);
        float forward  = lookup(running ? t.runF : t.walkF, stepPhase);

        double side = (stepIndex % 2 == 0) ? 1.0 : -1.0;             // 左右脚着地侧向冲击方向相反
        double lateral = 0.12 * amplitude * std::sin(kPi * stepsFloat)
                       + 0.10 * amplitude * side * bump(stepPhase, 0.02, 0.05);
        double z = (kGravity + strength * amplitude * vertical);
        if (z < 0.0) z = 0.0;

        out[0] = (float)(strength * lateral);
        out[1] = (float)(strength * 0.35 * amplitude * forward);
        out[2] = (float)z;
    }

    // 白噪声（OFF 等级 sensorNoise=0 自动关闭）
    if (level.sensorNoise > 0.0 && noiseSeed != 0) {
        // 简单 xorshift 高斯近似，仅用于完整方案；MVP 不进入
        uint32_t s0 = noiseSeed ^ 0x9E3779B9u;
        auto next = [&]() {
            s0 ^= s0 << 13; s0 ^= s0 >> 17; s0 ^= s0 << 5;
            return (float)((int32_t)s0) / 2147483648.0f;   // [-1,1)
        };
        out[0] += next() * (float)level.sensorNoise;
        out[1] += next() * (float)level.sensorNoise;
        out[2] += next() * (float)level.sensorNoise;
    }
}

} // namespace portal_step
