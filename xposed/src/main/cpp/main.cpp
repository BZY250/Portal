
#include <dobby.h>
#include <jni.h>
#include <sys/mman.h>
#include <unistd.h>
#include "sensor_hook.h"
#include "step_sim.h"

bool enableSensorHook = false;

JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM* vm, void* reserved) {
    JNIEnv* env;
    if (vm->GetEnv((void**) &env, JNI_VERSION_1_6) != JNI_OK) {
        return JNI_ERR;
    }

    doSensorHook();

    return JNI_VERSION_1_6;
}

extern "C"
JNIEXPORT void JNICALL
Java_moe_fuqiuluo_dobby_Dobby_setStatus(JNIEnv *env, jobject thiz, jboolean status) {
    enableSensorHook = status;
}

extern "C"
JNIEXPORT void JNICALL
Java_moe_fuqiuluo_dobby_Dobby_setMotion(JNIEnv *env, jobject thiz,
        jboolean enable, jboolean autoCadence, jint cadenceSpm, jdouble speedMs,
        jlong stepBase, jlong startTimeMs, jint realismLevel, jstring gaitTemplate) {
    using namespace portal_step;
    gStep.enableStepSim = (enable == JNI_TRUE);
    gStep.autoCadence   = (autoCadence == JNI_TRUE);
    gStep.cadenceSpm    = cadenceSpm;
    gStep.speedMs       = speedMs;
    gStep.stepBase      = stepBase;
    gStep.startTimeMs   = startTimeMs;
    gStep.realismLevel  = realismLevel;
    // 单调地板接续，跨会话不回退
    if ((double)stepBase > gStep.lastSteps) gStep.lastSteps = (double)stepBase;
    // 步态模板解码（格式 "v1;cadence;strideCount;x0,…;y0,…;z0,…"）
    if (gaitTemplate != nullptr) {
        const char* s = env->GetStringUTFChars(gaitTemplate, nullptr);
        if (s != nullptr) {
            decodeGaitTemplate(s);
            env->ReleaseStringUTFChars(gaitTemplate, s);
        }
    }
}
