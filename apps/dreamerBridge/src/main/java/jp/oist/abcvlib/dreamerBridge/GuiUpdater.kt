package jp.oist.abcvlib.dreamerBridge

import android.app.Activity
import android.graphics.Typeface
import android.view.Gravity
import android.widget.LinearLayout
import android.widget.TextView
import jp.oist.abcvlib.dreamerBridge.databinding.ActivityMainBinding
import java.util.Locale
import kotlin.concurrent.Volatile

/**
 * On-screen mirror of what the bridge is sending and receiving. Every field is
 * written from the Python control loop and read on the UI thread, so all of
 * them are volatile.
 *
 * Each number is printed at a fixed width with its sign, in a monospace font,
 * so digits do not jump around as values change, and is drawn as a bar over
 * the range it normally takes.
 */
class GuiUpdater(
    private val binding: ActivityMainBinding,
    private val activity: Activity
) {
    @Volatile var batteryVoltage: Double = 0.0
    @Volatile var chargerVoltage: Double = 0.0
    @Volatile var coilVoltage: Double = 0.0
    @Volatile var thetaDeg: Double = 0.0
    @Volatile var angularVelocityDeg: Double = 0.0

    /** Raw gyroscope pitch rate (device x axis), deg/s. No fusion filter. */
    @Volatile var gyroDeg: Double = 0.0

    /** Reward the trainer scored for the state we last reported. */
    @Volatile var reward: Double = 0.0
    @Volatile var wheelSpeedL: Double = 0.0
    @Volatile var wheelSpeedR: Double = 0.0
    @Volatile var wheelCountL: Long = 0
    @Volatile var wheelCountR: Long = 0

    /** Wheel command most recently applied, in [-1, 1]. */
    @Volatile var actionL: Double = 0.0
    @Volatile var actionR: Double = 0.0

    /** Step counter within the current episode. */
    @Volatile var step: Long = 0

    /** Measured control rate, which is what the trainer actually sees. */
    @Volatile var controlHz: Double = 0.0

    /** Connection state of the trainer socket. */
    @Volatile var trainerStatus: String = "starting"

    /** Free-text note on the wheels, e.g. "stopped". */
    @Volatile var lastAction: String = ""

    private inner class Row(
        label: String, unit: String, private val read: () -> Double,
        private val format: String, private val bar: BarView?,
    ) {
        private val text: TextView
        private val range: TextView?

        init {
            val line = LinearLayout(activity).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER_VERTICAL
                setPadding(0, dp(5), 0, dp(5))
            }
            line.addView(mono(label, LABEL).apply { maxLines = 1 },
                LinearLayout.LayoutParams(dp(72), LinearLayout.LayoutParams.WRAP_CONTENT))
            text = mono("", VALUE).apply { gravity = Gravity.END }
            line.addView(text,
                LinearLayout.LayoutParams(dp(78), LinearLayout.LayoutParams.WRAP_CONTENT))
            line.addView(mono(" $unit", UNIT).apply { textSize = 11f },
                LinearLayout.LayoutParams(dp(40), LinearLayout.LayoutParams.WRAP_CONTENT))
            if (bar != null) {
                line.addView(bar, LinearLayout.LayoutParams(0, dp(12), 1f))
                range = mono("", UNIT).apply { textSize = 10f; setPadding(dp(6), 0, 0, 0) }
                line.addView(range,
                    LinearLayout.LayoutParams(dp(44), LinearLayout.LayoutParams.WRAP_CONTENT))
            } else {
                range = null
            }
            binding.rows.addView(line)
        }

        fun update() {
            val v = read()
            text.text = String.format(Locale.US, format, v)
            bar?.value = v
            if (bar != null) range?.text = bar.rangeLabel()
        }
    }

    private val rows = mutableListOf<Row>()

    private fun dp(v: Int) = (v * activity.resources.displayMetrics.density).toInt()

    private fun mono(s: String, color: Int) = TextView(activity).apply {
        text = s
        typeface = Typeface.MONOSPACE
        textSize = 15f
        setTextColor(color)
    }

    private fun section(title: String) {
        binding.rows.addView(mono(title, ACCENT).apply {
            setTypeface(Typeface.MONOSPACE, Typeface.BOLD)
            textSize = 13f
            letterSpacing = 0.1f
            setPadding(0, dp(14), 0, dp(4))
        })
    }

    private fun signed(label: String, unit: String, range: Double, decimals: Int = 2,
                       auto: Boolean = false, read: () -> Double) {
        rows += Row(label, unit, read, "%+8.${decimals}f",
            BarView(activity, -range, range, signed = true, autoRange = auto))
    }

    private fun unsigned(label: String, unit: String, lo: Double, hi: Double,
                         decimals: Int = 2, read: () -> Double) {
        rows += Row(label, unit, read, "%8.${decimals}f",
            BarView(activity, lo, hi, signed = false))
    }

    private fun count(label: String, read: () -> Double) {
        rows += Row(label, "", read, "%8.0f", null)
    }

    init {
        section("BODY")
        signed("tilt", "deg", 45.0) { thetaDeg }
        signed("rate", "deg/s", 250.0, 1) { angularVelocityDeg }
        signed("gyro", "deg/s", 250.0, 1) { gyroDeg }
        section("WHEELS")
        signed("cmd L", "", 1.0) { actionL }
        signed("cmd R", "", 1.0) { actionR }
        signed("speed L", "", 1.0, auto = true) { wheelSpeedL }
        signed("speed R", "", 1.0, auto = true) { wheelSpeedR }
        count("count L") { wheelCountL.toDouble() }
        count("count R") { wheelCountR.toDouble() }
        section("CONTROL")
        unsigned("loop", "Hz", 0.0, 50.0, 1) { controlHz }
        signed("reward", "", 1.0, 3) { reward }
        count("step") { step.toDouble() }
        section("POWER")
        unsigned("battery", "V", 3.0, 4.3) { batteryVoltage }
        unsigned("charger", "V", 0.0, 6.0) { chargerVoltage }
        unsigned("coil", "V", 0.0, 6.0) { coilVoltage }
    }

    fun displayValues() {
        activity.runOnUiThread {
            rows.forEach { it.update() }
            binding.status.text = String.format(Locale.US, "trainer  %s%s", trainerStatus,
                if (lastAction.isNotEmpty()) "\nwheels   $lastAction" else "")
            binding.status.setTextColor(when {
                trainerStatus.startsWith("connected") || trainerStatus.startsWith("policy") -> GOOD
                trainerStatus.startsWith("standalone") -> ACCENT
                else -> WARN
            })
        }
    }

    private companion object {
        const val ACCENT = 0xFF4FC3F7.toInt()
        const val LABEL = 0xFFB0B0B0.toInt()
        const val VALUE = 0xFFFFFFFF.toInt()
        const val UNIT = 0xFF808080.toInt()
        const val GOOD = 0xFF81C784.toInt()
        const val WARN = 0xFFFFB74D.toInt()
    }
}
