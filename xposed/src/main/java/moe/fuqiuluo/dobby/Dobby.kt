package moe.fuqiuluo.dobby

object Dobby {

    external fun setStatus(status: Boolean)

    /**
     * 步数/步频真值合成配置（对应 native libportal.so 的 gStep）。
     * 由 RemoteCommandHandler 在速度/开关/配置变化时下发。
     */
    external fun setMotion(
        enable: Boolean,        // 步数模拟总开关
        autoCadence: Boolean,   // 是否按速度自动匹配步频
        cadenceSpm: Int,        // 固定步频（autoCadence=false 时用），60..240
        speedMs: Double,        // 当前伪造速度 m/s（与定位同源）
        stepBase: Long,         // 本会话起始累计步数
        startTimeMs: Long,      // 本会话开始时刻（elapsedRealtimeMillis）
        realismLevel: Int,      // 0..3（OFF/LOW/MEDIUM/HIGH）
        gaitTemplate: String?,  // 步态模板编码（可 null）
    )
}
