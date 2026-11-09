//
// Created by fuqiuluo on 2024/10/15.
// 融合步数/步频真值合成（改动：SensorEventQueueWrite 不再纯透传，改为按类型合成）
//
#include <dobby.h>
#include <unistd.h>
#include "sensor_hook.h"
#include "logging.h"
#include "elf_util.h"
#include "dobby_hook.h"
#include "step_sim.h"

#define LIBSF_PATH "/system/lib64/libsensorservice.so"

extern bool enableSensorHook;

// _ZN7android16SensorEventQueue5writeERKNS_2spINS_7BitTubeEEEPK12ASensorEventm
OriginalSensorEventQueueWriteType OriginalSensorEventQueueWrite = nullptr;

OriginalConvertToSensorEventType OriginalConvertToSensorEvent = nullptr;

// 传感器类型常量（稳定 ABI，与 frameworks/hardware/sensors.h 一致）
#define SENSOR_TYPE_ACCELEROMETER    1
#define SENSOR_TYPE_STEP_DETECTOR    18
#define SENSOR_TYPE_STEP_COUNTER     19

// 把单个 ASensorEvent 改写成伪造的步数/步频/加速度
static void synthStepEvent(ASensorEvent* ev) {
    using namespace portal_step;
    if (!gStep.enableStepSim) return;

    // 步数/步频/加速度都用「事件时间戳」现算，保证各连接、各进程一致
    int64_t nowMs = ev->timestamp / 1000000;   // ns → ms

    switch (ev->type) {
        case SENSOR_TYPE_STEP_COUNTER: {
            // 累计步数（单调递增，内部 lastSteps 保证不回退）
            ev->u64.step_counter = (uint64_t)stepsAtTime(nowMs);
            break;
        }
        case SENSOR_TYPE_STEP_DETECTOR: {
            // 运动中每收到一次事件给一个「检测到一步」脉冲；静止则 0
            ev->data[0] = (gStep.speedMs > 0.1) ? 1.0f : 0.0f;
            break;
        }
        case SENSOR_TYPE_ACCELEROMETER: {
            // 仅运动中合成步态震动，静态绝不篡改（保证人脸识别等传感器纯净）
            if (gStep.speedMs > 0.1) {
                double steps = stepsAtTime(nowMs);
                synthAccel(steps, gStep.speedMs, ev->data, (unsigned int)ev->timestamp);
            }
            break;
        }
        default:
            // 其它运动传感器（gyro/rotation_vector/linear_accel 等）：
            // 完整方案由步态相位派生；MVP 保持 Portal 原有「失效化」或透传
            break;
    }
}

int64_t SensorEventQueueWrite(void *tube, void *events, int64_t numEvents) {
    if (enableSensorHook && events != nullptr && numEvents > 0) {
        auto* ev = static_cast<ASensorEvent*>(events);
        for (int64_t i = 0; i < numEvents; ++i) {
            synthStepEvent(&ev[i]);
        }
    }
    return OriginalSensorEventQueueWrite(tube, events, numEvents);
}

void ConvertToSensorEvent(void *src, void *dst) {
    if (enableSensorHook) {
        auto a = *(int32_t *)((char*)src + 4);
        auto b = *(int32_t *)((char*)src + 8);
        auto c = *(int64_t *)((char*)src + 16);

        *(int64_t *)((char*)dst + 16) = 0LL;
        *(int32_t *)((char*)dst + 24) = 0;
        *(int64_t *)((char*)dst) = c;
        *(int32_t *)((char*)dst + 8) = a;
        *(int32_t *)((char*)dst + 12) = b;
        *(int8_t *)((char*)dst + 28) = b;

        if (b == 18) {
            *(float *)((char*)dst + 16) = -1.0;
        } else if (b == 19) {
            *(int64_t *)((char*)dst + 16) = -1;
        } else {
            *(float *)((char*)dst + 16) = -1.0;
            *(float *)((char*)dst + 24) = -1.0;
            *(int8_t *)((char*)dst + 28) = *(int8_t *)((char*)src + 36);
        }
    } else {
        OriginalConvertToSensorEvent(src, dst);
    }

    if (enableSensorHook) {
        LOGD("ConvertToSensorEvent called");
    }
}

void doSensorHook() {
    SandHook::ElfImg sensorService(LIBSF_PATH);

    if (!sensorService.isValid()) {
        LOGE("failed to load libsensorservice");
        return;
    }

    auto sensorWrite = sensorService.getSymbolAddress<void*>("_ZN7android16SensorEventQueue5writeERKNS_2spINS_7BitTubeEEEPK12ASensorEventm");
    if (sensorWrite == nullptr) {
        sensorWrite = sensorService.getSymbolAddress<void*>("_ZN7android16SensorEventQueue5writeERKNS_2spINS_7BitTubeEEEPK12ASensorEventj");
    }

    auto convertToSensorEvent = sensorService.getSymbolAddress<void*>("_ZN7android8hardware7sensors4V1_014implementation20convertToSensorEventERKNS2_5EventEP15sensors_event_t");

    LOGD("Dobby SensorEventQueue::write found at %p", sensorWrite);
    LOGD("Dobby convertToSensorEvent found at %p", convertToSensorEvent);

    if (sensorWrite != nullptr) {
        OriginalSensorEventQueueWrite = (OriginalSensorEventQueueWriteType)InlineHook(sensorWrite, (void *)SensorEventQueueWrite);
    }

    if (convertToSensorEvent != nullptr) {
        OriginalConvertToSensorEvent = (OriginalConvertToSensorEventType)InlineHook(convertToSensorEvent, (void *)ConvertToSensorEvent);
    }
}
