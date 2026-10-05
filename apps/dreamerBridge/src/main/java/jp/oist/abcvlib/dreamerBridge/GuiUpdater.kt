package jp.oist.abcvlib.dreamerBridge

import android.app.Activity
import jp.oist.abcvlib.dreamerBridge.databinding.ActivityMainBinding
import java.text.DecimalFormat
import kotlin.concurrent.Volatile

/**
 * On-screen mirror of what the bridge is sending and receiving. Every field is
 * written from the Python control loop and read on the UI thread, so all of
 * them are volatile.
 */
class GuiUpdater(
    private val binding: ActivityMainBinding,
    private val activity: Activity
) {
    private val df = DecimalFormat("#.00")

    /** Rewards run small and can be negative, so they need a leading zero. */
    private val rewardFormat = DecimalFormat("0.000")

    @Volatile
    var batteryVoltage: Double = 0.0

    @Volatile
    var chargerVoltage: Double = 0.0

    @Volatile
    var coilVoltage: Double = 0.0

    @Volatile
    var thetaDeg: Double = 0.0

    @Volatile
    var angularVelocityDeg: Double = 0.0

    /** Reward the trainer scored for the state we last reported. */
    @Volatile
    var reward: Double = 0.0

    @Volatile
    var wheelLeftData: String = ""

    @Volatile
    var wheelRightData: String = ""

    /** Connection state of the trainer socket. */
    @Volatile
    var trainerStatus: String = "starting"

    /** Wheel output most recently commanded by the trainer. */
    @Volatile
    var lastAction: String = "stopped"

    /** Step counter within the current episode. */
    @Volatile
    var stepInfo: String = "0"

    /** Measured control rate, which is what the trainer actually sees. */
    @Volatile
    var controlHz: Double = 0.0

    fun displayValues() {
        activity.runOnUiThread {
            binding.voltageBattLevel.text = df.format(batteryVoltage)
            binding.voltageChargerLevel.text = df.format(chargerVoltage)
            binding.coilVoltageText.text = df.format(coilVoltage)
            binding.tiltAngle.text = df.format(thetaDeg)
            binding.angularVelcoity.text = df.format(angularVelocityDeg)
            binding.reward.text = rewardFormat.format(reward)
            binding.leftWheelData.text = wheelLeftData
            binding.rightWheelData.text = wheelRightData
            binding.soundData.text = stepInfo
            binding.frameRate.text = df.format(controlHz)
            binding.qrData.text = trainerStatus
            binding.objectDetector.text = lastAction
        }
    }
}
