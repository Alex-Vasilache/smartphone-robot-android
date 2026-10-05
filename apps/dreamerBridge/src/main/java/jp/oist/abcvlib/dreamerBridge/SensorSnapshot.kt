package jp.oist.abcvlib.dreamerBridge

import android.os.SystemClock
import jp.oist.abcvlib.core.inputs.microcontroller.BatteryDataSubscriber
import jp.oist.abcvlib.core.inputs.microcontroller.WheelDataSubscriber
import jp.oist.abcvlib.core.inputs.phone.OrientationDataSubscriber

/**
 * The latest value of every sensor, kept in Kotlin so that the sensor threads
 * never have to enter Python.
 *
 * The control loop used to subscribe from Python through Chaquopy proxies.
 * Every orientation event -- ~200 a second -- then crossed into Python and
 * took the GIL, and the policy step, which is pure numpy under that same GIL,
 * paid for it: measured 2026-09-14 on the Pixel 3a, 3.5 ms per step with the
 * sensors off, 10.8 ms hot and 20-25 ms in the live loop with them on. In the
 * other direction, the callbacks queued behind the policy, so the IMU age
 * spiked by exactly the policy's duration.
 *
 * Now the publishers write here (plain volatile stores from their own
 * threads), and Python reads one JSON string per control step.
 */
object SensorSnapshot : OrientationDataSubscriber, WheelDataSubscriber,
    BatteryDataSubscriber {

    @Volatile private var theta = 0.0
    @Volatile private var angularVelocity = 0.0
    /** elapsedRealtimeNanos - event timestamp, at the callback. */
    @Volatile private var imuAgeMs = 0.0
    /** System.nanoTime at the callback: the same clock as Python's monotonic. */
    @Volatile private var imuCallbackNs = 0L

    @Volatile private var wheelCountL = 0
    @Volatile private var wheelCountR = 0
    @Volatile private var wheelDistanceL = 0.0
    @Volatile private var wheelDistanceR = 0.0
    @Volatile private var wheelSpeedL = 0.0
    @Volatile private var wheelSpeedR = 0.0
    @Volatile private var wheelCallbackNs = 0L

    @Volatile private var batteryVoltage = 0.0
    @Volatile private var chargerVoltage = 0.0
    @Volatile private var coilVoltage = 0.0

    override fun onOrientationUpdate(
        timestamp: Long, thetaRad: Double, angularVelocityRad: Double
    ) {
        theta = thetaRad
        angularVelocity = angularVelocityRad
        imuAgeMs = (SystemClock.elapsedRealtimeNanos() - timestamp) / 1e6
        imuCallbackNs = System.nanoTime()
    }

    override fun onWheelDataUpdate(
        timestamp: Long, wheelCountL: Int, wheelCountR: Int,
        wheelDistanceL: Double, wheelDistanceR: Double,
        wheelSpeedInstantL: Double, wheelSpeedInstantR: Double,
        wheelSpeedBufferedL: Double, wheelSpeedBufferedR: Double,
        wheelSpeedExpAvgL: Double, wheelSpeedExpAvgR: Double
    ) {
        this.wheelCountL = wheelCountL
        this.wheelCountR = wheelCountR
        this.wheelDistanceL = wheelDistanceL
        this.wheelDistanceR = wheelDistanceR
        // Instantaneous, not the exponential average: wheel data arrives at
        // the serial reply rate, so an EMA has a time constant of seconds.
        wheelSpeedL = wheelSpeedInstantL
        wheelSpeedR = wheelSpeedInstantR
        wheelCallbackNs = System.nanoTime()
    }

    override fun onBatteryVoltageUpdate(timestamp: Long, voltage: Double) {
        batteryVoltage = voltage
    }

    override fun onChargerVoltageUpdate(
        timestamp: Long, chargerVoltage: Double, coilVoltage: Double
    ) {
        this.chargerVoltage = chargerVoltage
        this.coilVoltage = coilVoltage
    }

    /** The subscriber instance, for `addSubscriber` from Python. */
    @JvmStatic
    fun subscriber(): SensorSnapshot = this

    /**
     * Everything above, as the `sensors` dict the bridge protocol carries plus
     * the two timing fields the control loop needs. One string, one call.
     */
    @JvmStatic
    fun snapshotJson(): String = (
        "{\"theta\":" + theta +
        ",\"angular_velocity\":" + angularVelocity +
        ",\"wheel_count_l\":" + wheelCountL +
        ",\"wheel_count_r\":" + wheelCountR +
        ",\"wheel_distance_l\":" + wheelDistanceL +
        ",\"wheel_distance_r\":" + wheelDistanceR +
        ",\"wheel_speed_l\":" + wheelSpeedL +
        ",\"wheel_speed_r\":" + wheelSpeedR +
        ",\"battery_voltage\":" + batteryVoltage +
        ",\"charger_voltage\":" + chargerVoltage +
        ",\"coil_voltage\":" + coilVoltage +
        ",\"imu_age_ms\":" + imuAgeMs +
        ",\"imu_callback_s\":" + imuCallbackNs / 1e9 +
        ",\"wheel_callback_s\":" + wheelCallbackNs / 1e9 + "}")
}
