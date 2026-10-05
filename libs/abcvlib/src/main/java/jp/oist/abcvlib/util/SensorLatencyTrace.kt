package jp.oist.abcvlib.util

/**
 * A low-lag reference for the orientation signal, plus the evidence needed to
 * decide whether the sensor timestamps mean what we assume.
 *
 * Two separate doubts motivated this:
 *
 * 1. `theta` comes from TYPE_ROTATION_VECTOR, a *fused* virtual sensor. Its
 *    output is the result of a filter, so it lags the physical orientation by
 *    the filter's group delay -- and that delay is nowhere in `event.timestamp`,
 *    which dates the sample, not the estimate. The raw gyroscope has no such
 *    filter, so the lag between the two is directly measurable by comparing
 *    them.
 * 2. `event.timestamp` is documented as elapsedRealtimeNanos, but that is a
 *    per-device promise rather than a guarantee. Recording the event stamp
 *    against *both* clocks at the moment of the callback settles which base it
 *    is on instead of assuming: on the right base the difference is a few
 *    milliseconds and stable, on the wrong one it is the accumulated deep-sleep
 *    time, which is large and grows.
 *
 * Written from the sensor thread, read from the control thread; each field is a
 * single volatile store, which is all the consistency a diagnostic needs.
 */
object SensorLatencyTrace {

    /**
     * Whether to register the raw gyroscope alongside the rotation vector.
     * Set before the publishers start. Off by default: registering extra
     * sensors on the single sensor thread is exactly what once produced an 8.5
     * second delivery backlog, so this stays an opt-in diagnostic until it has
     * been shown not to cost anything.
     */
    @Volatile
    var gyroEnabled: Boolean = false

    /**
     * Assignment from Python silently fails to reach Kotlin `var`s in an
     * object, so the flag is set through a method instead.
     */
    @JvmStatic
    fun setGyro(enabled: Boolean) {
        gyroEnabled = enabled
    }

    // -- raw gyroscope, device axes, rad/s -----------------------------------

    @Volatile var gyroX: Float = 0f
    @Volatile var gyroY: Float = 0f
    @Volatile var gyroZ: Float = 0f

    /** Monotonic (nanoTime) instant the gyro callback ran. */
    @Volatile var gyroCallbackNs: Long = 0L

    /** elapsedRealtimeNanos - event.timestamp, milliseconds. */
    @Volatile var gyroAgeMs: Double = 0.0

    @Volatile var gyroCount: Long = 0L

    // -- rotation vector -----------------------------------------------------

    @Volatile var rotAgeMs: Double = 0.0

    /**
     * The same age computed against nanoTime instead of elapsedRealtimeNanos.
     * Only one of the two can be right; if they disagree by a large and growing
     * amount, every "sensor age" ever reported by this app was on the wrong
     * base.
     */
    @Volatile var rotAgeUptimeMs: Double = 0.0

    @Volatile var rotCallbackNs: Long = 0L

    @Volatile var rotCount: Long = 0L

    /** As one JSON object; see ControlLatencyTrace.snapshotJson for why. */
    @JvmStatic
    fun snapshotJson(): String = (
        "{\"gyro_x\":" + gyroX +
        ",\"gyro_y\":" + gyroY +
        ",\"gyro_z\":" + gyroZ +
        ",\"gyro_age_ms\":" + gyroAgeMs +
        ",\"gyro_count\":" + gyroCount +
        ",\"imu_age_uptime_ms\":" + rotAgeUptimeMs +
        ",\"rot_count\":" + rotCount + "}")
}
