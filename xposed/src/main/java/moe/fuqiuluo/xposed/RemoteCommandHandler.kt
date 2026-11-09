package moe.fuqiuluo.xposed

import android.annotation.SuppressLint
import android.os.Bundle
import android.os.IBinder
import android.os.Parcel
import android.os.SystemClock
import moe.fuqiuluo.dobby.Dobby
import moe.fuqiuluo.xposed.hooks.LocationServiceHook
import moe.fuqiuluo.xposed.utils.FakeLoc
import moe.fuqiuluo.xposed.utils.BinderUtils
import moe.fuqiuluo.xposed.utils.Logger
import java.util.Collections
import kotlin.random.Random

object RemoteCommandHandler {
    private val proxyBinders by lazy { Collections.synchronizedList(arrayListOf<IBinder>()) }
    private val needProxyCmd = arrayOf("start", "stop", "set_speed_amp", "set_altitude", "set_speed", "update_location", "set_bearing", "move", "put_config")
    internal val randomKey by lazy { "portal_" + Random.nextDouble() }
    private var isLoadedLibrary = false

    @SuppressLint("UnsafeDynamicallyLoadedCode")
    fun handleInstruction(command: String, rely: Bundle): Boolean {
        // Exchange key -> returns a random key -> is used to verify that it is the PortalManager
        if (command == "exchange_key") {
            val userId = BinderUtils.getCallerUid()
            if (BinderUtils.isLocationProviderEnabled(userId)) {
                rely.putString("key", randomKey)
                return true
            }
            // Go back and see if the instruction has been processed to prevent it from being detected by others
        } else if (command != randomKey) {
            return false
        }
        val commandId = rely.getString("command_id") ?: return false

        kotlin.runCatching {
            if (proxyBinders.isNotEmpty() && needProxyCmd.any { it == commandId }) {
                proxyBinders.removeIf {
                    if (it.isBinderAlive && it.pingBinder()) {
                        val data = Parcel.obtain()
                        data.writeBundle(rely)
                        it.transact(1, data, null, 0)
                        data.recycle()
                        false
                    } else true
                }
            }
        }.onFailure {
            Logger.error("Failed to transact with proxyBinder", it)
        }

        if (FakeLoc.enableDebugLog) {
            Logger.debug("commandId=$commandId, rely=$rely")
        }

        when (commandId) {
            "set_proxy" -> {
                Logger.info("SubProxyBinder: ${rely.getBinder("proxy")} from ${BinderUtils.getUidPackageNames()}!")
                rely.getBinder("proxy")?.let {
                    proxyBinders.add(it)
                }
                return true
            }
            "start" -> {
                val speed = rely.getDouble("speed", FakeLoc.speed)
                val altitude = rely.getDouble("altitude", FakeLoc.altitude)
                val accuracy = rely.getFloat("accuracy", FakeLoc.accuracy)

                FakeLoc.enable = true
                if (isLoadedLibrary) {
                    Dobby.setStatus(true)
                }

                FakeLoc.speed = speed
                FakeLoc.altitude = altitude
                FakeLoc.accuracy = accuracy

                // 步数模拟：开启会话（首次记录开始时刻，native 侧维护单调步数）
                if (FakeLoc.stepStartTimeMs == 0L) {
                    FakeLoc.stepStartTimeMs = SystemClock.elapsedRealtime()
                }
                syncStepToNative()

                return true
            }
            "stop" -> {
                FakeLoc.enable = false
                FakeLoc.hasBearings = false
                if (isLoadedLibrary) {
                    Dobby.setStatus(false)
                }
                syncStepToNative()
                return true
            }
            "is_start" -> {
                rely.putBoolean("is_start", FakeLoc.enable)
                return true
            }
            "start_gnss_mock" -> {
                FakeLoc.enableMockGnss = true
                return true
            }
            "stop_gnss_mock" -> {
                FakeLoc.enableMockGnss = false
                return true
            }
            "is_gnss_start" -> {
                rely.putBoolean("is_gnss_start", FakeLoc.enableMockGnss)
                return true
            }
            "is_wifi_mock_start" -> {
                rely.putBoolean("is_wifi_mock_start", FakeLoc.enableMockWifi)
                return true
            }
            "start_wifi_mock" -> {
                FakeLoc.enableMockWifi = true
                return true
            }
            "stop_wifi_mock" -> {
                FakeLoc.enableMockWifi = false
                return true
            }
            "get_location" -> {
                rely.putDouble("lat", FakeLoc.latitude)
                rely.putDouble("lon", FakeLoc.longitude)
                return true
            }
            "get_listener_size" -> {
                rely.putInt("size", LocationServiceHook.locationListeners.size)
                return true
            }
            "get_speed" -> {
                rely.putDouble("speed", FakeLoc.speed)
                return true
            }
            "get_bearing" -> {
                rely.putDouble("bearing", FakeLoc.bearing)
                return true
            }
            "get_altitude" -> {
                rely.putDouble("altitude", FakeLoc.altitude)
                return true
            }
            "set_speed_amp" -> {
                val speedAmplitude = rely.getDouble("speed_amplitude", 1.0)
                FakeLoc.speedAmplitude = speedAmplitude
                return true
            }
            "set_altitude" -> {
                val altitude = rely.getDouble("altitude", 0.0)
                FakeLoc.altitude = altitude
                return true
            }
            "set_speed" -> {
                val speed = rely.getDouble("speed", 0.0)
                FakeLoc.speed = speed
                syncStepToNative()
                return true
            }
            "set_bearing" -> {
                val bearing = rely.getDouble("bearing", 0.0)
                FakeLoc.bearing = bearing
                FakeLoc.hasBearings = true
                return true
            }
            "move" -> {
                val distance = rely.getDouble("n", 0.0)
                if (distance == 0.0) return true
                val bearing = rely.getDouble("bearing", 0.0)
                val newLoc = FakeLoc.moveLocation(
                    n = distance,
                    angle = bearing
                )
                if (FakeLoc.enableDebugLog) {
                    Logger.debug("move: distance=$distance, bearing=$bearing, newLoc=$newLoc")
                }
                FakeLoc.bearing = bearing
                FakeLoc.hasBearings = true
                return updateCoordinate(newLoc.first, newLoc.second).also {
                    if (FakeLoc.isSystemServerProcess) LocationServiceHook.callOnLocationChanged()
                }
            }
            "update_location" -> {
                val mode = rely.getString("mode")
                var newLat = rely.getDouble("lat", 0.0)
                var newLon = rely.getDouble("lon", 0.0)
                when(mode) {
                    "+" -> {
                        newLat += FakeLoc.latitude
                        newLon += FakeLoc.longitude
                        return updateCoordinate(newLat, newLon)
                    }
                    "-" -> {
                        newLat = FakeLoc.latitude - newLat
                        newLon = FakeLoc.longitude - newLon
                        return updateCoordinate(newLat, newLon)
                    }
                    "*" -> {
                        newLat *= FakeLoc.latitude
                        newLon *= FakeLoc.longitude
                        return updateCoordinate(newLat, newLon)
                    }
                    "/" -> {
                        if (newLat == 0.0 || newLon == 0.0) {
                            return false
                        }
                        newLat /= FakeLoc.latitude
                        newLon /= FakeLoc.longitude
                        return updateCoordinate(newLat, newLon)
                    }
                    "=" -> {
                        return updateCoordinate(newLat, newLon)
                    }
                    "random" -> {
                        return updateCoordinate(Random.nextDouble(-90.0, 90.0), Random.nextDouble(-180.0, 180.0))
                    }
                }
                return true
            }
            "put_config" -> {
                val enable = rely.getBoolean("enable", FakeLoc.enable)
                val speed = rely.getDouble("speed", FakeLoc.speed)
                val altitude = rely.getDouble("altitude", FakeLoc.altitude)
                val accuracy = rely.getFloat("accuracy", FakeLoc.accuracy)
                val enableDebugLog = rely.getBoolean("enable_debug_log", FakeLoc.enableDebugLog)
                val disableGetCurrentLocation = rely.getBoolean("disable_get_current_location", FakeLoc.disableGetCurrentLocation)
                val disableRegisterLocationListener = rely.getBoolean("disable_register_location_listener", FakeLoc.disableRegisterLocationListener)
                val disableFusedLocation = rely.getBoolean("disable_fused_location", FakeLoc.disableFusedLocation)
                val needDowngradeToCdma = rely.getBoolean("need_downgrade_to_2g", FakeLoc.needDowngradeToCdma)
                var minSatellites = rely.getInt("min_satellites", 12)
                if (minSatellites < 0) {
                    minSatellites = 12
                }

                val enableAGPS = rely.getBoolean("enable_agps", FakeLoc.enableAGPS)
                val enableNMEA = rely.getBoolean("enable_nmea", FakeLoc.enableNMEA)
                val disableRequestGeofence = rely.getBoolean("disable_request_geofence", FakeLoc.disableRequestGeofence)
                val disableGetFromLocation = rely.getBoolean("disable_get_from_location", FakeLoc.disableGetFromLocation)

                // 步数/步频模拟配置（融合自 LocationSpoofer）
                val enableStepSim = rely.getBoolean("enable_step_sim", FakeLoc.enableStepSim)
                val autoCadence = rely.getBoolean("auto_cadence", FakeLoc.autoCadence)
                var stepCadenceSpm = rely.getInt("step_cadence_spm", FakeLoc.stepCadenceSpm)
                if (stepCadenceSpm < 60) stepCadenceSpm = 60
                if (stepCadenceSpm > 240) stepCadenceSpm = 240
                val realismLevel = rely.getInt("realism_level", FakeLoc.realismLevel)
                val gaitTemplate = rely.getString("gait_template", FakeLoc.gaitTemplate)

                FakeLoc.enable = enable
                FakeLoc.speed = speed
                FakeLoc.altitude = altitude
                FakeLoc.accuracy = accuracy
                FakeLoc.enableDebugLog = enableDebugLog
                FakeLoc.disableGetCurrentLocation = disableGetCurrentLocation
                FakeLoc.disableRegisterLocationListener = disableRegisterLocationListener
                FakeLoc.disableFusedLocation = disableFusedLocation
                FakeLoc.needDowngradeToCdma = needDowngradeToCdma
                FakeLoc.minSatellites = minSatellites
                FakeLoc.enableAGPS = enableAGPS
                FakeLoc.enableNMEA = enableNMEA
                FakeLoc.disableRequestGeofence = disableRequestGeofence
                FakeLoc.disableGetFromLocation = disableGetFromLocation

                FakeLoc.enableStepSim = enableStepSim
                FakeLoc.autoCadence = autoCadence
                FakeLoc.stepCadenceSpm = stepCadenceSpm
                FakeLoc.realismLevel = realismLevel
                FakeLoc.gaitTemplate = gaitTemplate
                syncStepToNative()
                return true
            }
            "sync_config" -> {
                rely.putBoolean("enable", FakeLoc.enable)
                rely.putDouble("latitude", FakeLoc.latitude)
                rely.putDouble("longitude", FakeLoc.longitude)
                rely.putDouble("altitude", FakeLoc.altitude)
                rely.putDouble("speed", FakeLoc.speed)
                rely.putDouble("speed_amplitude", FakeLoc.speedAmplitude)
                rely.putBoolean("has_bearings", FakeLoc.hasBearings)
                rely.putDouble("bearing", FakeLoc.bearing)
                rely.putParcelable("last_location", FakeLoc.lastLocation)
                rely.putBoolean("enable_log", FakeLoc.enableLog)
                rely.putBoolean("enable_debug_log", FakeLoc.enableDebugLog)
                rely.putBoolean("disable_get_current_location", FakeLoc.disableGetCurrentLocation)
                rely.putBoolean("disable_register_location_listener", FakeLoc.disableRegisterLocationListener)
                rely.putBoolean("disable_fused_location", FakeLoc.disableFusedLocation)
                rely.putBoolean("enable_agps", FakeLoc.enableAGPS)
                rely.putBoolean("enable_nmea", FakeLoc.enableNMEA)
                rely.putBoolean("hide_mock", FakeLoc.hideMock)
                rely.putBoolean("hook_wifi", FakeLoc.hookWifi)
                rely.putBoolean("need_downgrade_to_2g", FakeLoc.needDowngradeToCdma)
                rely.putBoolean("enable_step_sim", FakeLoc.enableStepSim)
                rely.putBoolean("auto_cadence", FakeLoc.autoCadence)
                rely.putInt("step_cadence_spm", FakeLoc.stepCadenceSpm)
                rely.putInt("realism_level", FakeLoc.realismLevel)
                rely.putString("gait_template", FakeLoc.gaitTemplate)
                return true
            }
            "broadcast_location" -> {
                LocationServiceHook.callOnLocationChanged()
                return true
            }
            "load_library" -> {
                val path = rely.getString("path") ?: return false

                if (isLoadedLibrary && path.endsWith("libportal.so")) {
                    rely.putString("result", "success")
                    return true
                }
                runCatching {
                    System.load(path)
                }.onSuccess {
                    rely.putString("result", "success")
                    isLoadedLibrary = true
                }.onFailure {
                    rely.putString("result", it.stackTraceToString())
                }

                if (isLoadedLibrary) {
                    Dobby.setStatus(FakeLoc.enable)
                    syncStepToNative()
                }

                return true
            }
            else -> return false
        }
    }

//    private var hasHookSensor = false
//
//    private fun tryHookSensor(classLoader: ClassLoader = FakeLoc::class.java.classLoader!!) {
//        if (hasHookSensor || proxyBinders.isNullOrEmpty()) return
//
//
//
//        hasHookSensor = true
//    }

//    private fun generateLocation(): Location {
//        val (location, realLocation) = if (FakeLocationConfig.lastLocation != null) {
//            (FakeLocationConfig.lastLocation!! to true)
//        } else {
//            (Location(LocationManager.GPS_PROVIDER) to false)
//        }
//
//        return LocationServiceProxyHook.injectLocation(location, realLocation)
//    }

    /**
     * 把当前速度/开关/步数基线/步态模板下发到 native（libportal.so 的 gStep），
     * 让 native 在 system_server 的 SensorEventQueue::write 里合成步数/步频/加速度。
     * 注意：native 侧有单调地板，静止（speed<=0.05）自动冻结，跨会话不回退。
     */
    private fun syncStepToNative() {
        if (!isLoadedLibrary) return
        Dobby.setMotion(
            FakeLoc.enableStepSim && FakeLoc.enable,   // 定位开启且步数模拟开启时才合成
            FakeLoc.autoCadence,
            FakeLoc.stepCadenceSpm,
            FakeLoc.speed,
            FakeLoc.stepBase,
            FakeLoc.stepStartTimeMs,
            FakeLoc.realismLevel,
            FakeLoc.gaitTemplate,
        )
    }

    private fun updateCoordinate(newLat: Double, newLon: Double): Boolean {
        if (newLat in -90.0..90.0 && newLon in -180.0..180.0) {
            FakeLoc.latitude = newLat
            FakeLoc.longitude = newLon
            syncStepToNative()
            return true
        } else {
            Logger.error("Invalid latitude or longitude: $newLat, $newLon")
            return false
        }
    }
}