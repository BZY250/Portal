# Portal（步数/步频融合版）

本仓库在 Portal（moe.fuqiuluo.portal，LSPosed 虚拟定位模块）基础上，融合了 **LocationSpoofer 的步数模拟 + 步频自动匹配** 能力：
把步数/步频/步态波形算法移植进 Portal 的 system_server 侧原生 hook（`libsensorservice.so` 的 `SensorEventQueue::write`），
使其**全局生效**（含 Android 工作应用空间 Work Profile），而不是只在单个 App 进程内生效。

## 改动清单

| 文件 | 改动 |
|---|---|
| `xposed/src/main/cpp/step_sim.h` / `step_sim.cpp` | **新增**：步数/步频/步态波形算法（速度→步频映射、时间积分、单调地板、SmoothNoise 闭式噪声、GaitTemplate 解码）。已主机侧 28 项测试通过 |
| `xposed/src/main/cpp/sensor_hook.cpp` | **改造**：`SensorEventQueue::write` 从纯透传 → 按事件类型合成 STEP_COUNTER / STEP_DETECTOR / ACCELEROMETER |
| `xposed/src/main/cpp/main.cpp` | **新增** `Dobby.setMotion` JNI（下发速度/开关/步数基线/步态模板到 native） |
| `xposed/src/main/cpp/CMakeLists.txt` | 加入 `step_sim.cpp` |
| `xposed/src/main/java/.../dobby/Dobby.kt` | 新增 `external fun setMotion(...)` |
| `xposed/src/main/java/.../utils/FakeLoc.kt` | 新增步数配置字段（enableStepSim/autoCadence/stepCadenceSpm/stepBase/stepStartTimeMs/realismLevel/gaitTemplate） |
| `xposed/src/main/java/.../RemoteCommandHandler.kt` | 新增 `syncStepToNative()`，在 start/stop/set_speed/update_location/move/put_config 出口下发；put_config/sync_config 增加步数配置字段 |
| `.github/workflows/build-apk.yml` | **新增**：云端 Gradle 构建完整 APK |

## 步频自动匹配（核心算法）

速度(m/s) → 步频(步/分钟 SPM) 分段映射：
`≤0.8→95` 慢走；`0.8~1.5→100~125` 快走；`1.5~2.8→135~160` 慢跑；`2.8~4.0→160~180` 匀速跑；`4.0~5.5→180~195` 快跑；`>5.5→200` 冲刺。

步数：`steps(t) = stepBase + (cadence/60) × t`（连续值，单调不回退，静止自动冻结）。
加速度计：内置步行/跑步 256 点去均值查表波形（含重力 9.80665），或回放用户录制步态模板。

## 云端构建（GitHub Actions）

1. 把本仓库推到 GitHub。
2. Actions 自动触发 `Build Portal step-sim APK`，产出 **arm64 debug 签名 APK**（可直接安装）。
3. 到 Artifacts 下载 `Portal-arm64-debug-apk`。

> 说明：app 有 `app`/`arm64`/`x64` 三个 flavor。`arm64` = 仅 arm64-v8a（现代手机通用）。
> debug 签名可直接安装；如需 release 签名，参考原 `build-apk.yml`（需 KEYSTORE 与 google-services 密钥）。

## 本地构建

```bash
# 需 JDK 17 + Android SDK(platform 35) + NDK 26.1.10909125
./gradlew :app:assembleArm64Debug
# 产物：app/build/outputs/apk/arm64/debug/*.apk
```

## 部署（需 Root + LSPosed）

1. 安装产出的 APK（替换旧 Portal）。
2. LSPosed 激活 Portal 模块，作用域勾选 `system`（或按需勾选目标 App）。
3. 首次启动 Portal → 授权 Root → 触发 load_library（把 libportal.so 加载进 system_server）。
4. 开启模拟并设定速度，步数/步频即全局生效（含工作空间 App）。

## 原理与设计文档

详见 `/root/workspace/locationspoofer_analysis/reports/` 与 `/root/workspace/locationspoofer_analysis/impl/`（分析报告、算法实现、Java 胶水、验证脚本）。
