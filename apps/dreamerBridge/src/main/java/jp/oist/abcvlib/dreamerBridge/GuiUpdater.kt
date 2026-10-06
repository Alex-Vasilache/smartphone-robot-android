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

    /** Reward the trainer scored for the state we last reported. */
    @Volatile var reward: Double = 0.0

    /** The same, averaged over ~0.5 s of steps: the headline number. */
    @Volatile var rewardAvg: Double = 0.0

    /** Mean reward per step so far this episode, and the return so far. */
    @Volatile var episodeMean: Double = 0.0
    @Volatile var episodeReturn: Double = 0.0

    /** The same for the last finished episode. */
    @Volatile var lastEpisodeMean: Double = Double.NaN
    @Volatile var lastEpisodeReturn: Double = Double.NaN
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

    /** Missed RP2040 replies, e.g. "3 missed replies, last 15:35:04". */
    @Volatile var serialNote: String = ""

    /** Free-text note on the wheels, e.g. "stopped". */
    @Volatile var lastAction: String = ""

    private fun dp(v: Int) = (v * activity.resources.displayMetrics.density).toInt()

    private fun text(s: String, color: Int, size: Float, bold: Boolean = false) =
        TextView(activity).apply {
            text = s
            typeface = Typeface.create(Typeface.MONOSPACE, if (bold) Typeface.BOLD else Typeface.NORMAL)
            textSize = size
            maxLines = 1
            setTextColor(color)
        }

    private fun spacer() {
        binding.rows.addView(android.view.View(activity),
            LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f))
    }

    private fun section(title: String, inset: Int = 0) {
        spacer()
        binding.rows.addView(text(title, Palette.INK_3, 11f, bold = true).apply {
            letterSpacing = 0.12f
            setPadding(dp(inset), dp(4), 0, dp(6))
        })
    }

    private fun hstack() = LinearLayout(activity).apply {
        orientation = LinearLayout.HORIZONTAL
        gravity = Gravity.CENTER_VERTICAL
    }

    private fun vstack() = LinearLayout(activity).apply { orientation = LinearLayout.VERTICAL }

    private fun weight(w: Float = 1f) =
        LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, w)

    private val updates = mutableListOf<() -> Unit>()

    /** Tier 2: a large signed gauge -- label and big value above a thick bar. */
    private fun gauge(label: String, unit: String, range: Double, format: String,
                      read: () -> Double) {
        val head = hstack()
        head.addView(text(label, Palette.INK_2, 14f), weight())
        val value = text("", Palette.INK, 26f, bold = true)
        head.addView(value)
        head.addView(text(" $unit", Palette.INK_3, 12f))
        binding.rows.addView(head)
        val bar = BarView(activity, -range, range, signed = true)
        binding.rows.addView(bar, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, dp(22)).apply { topMargin = dp(2) })
        val ends = hstack()
        ends.addView(text(String.format(Locale.US, "%+.0f", -range), Palette.INK_3, 10f), weight())
        ends.addView(text(String.format(Locale.US, "%+.0f", range), Palette.INK_3, 10f))
        binding.rows.addView(ends, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT).apply {
            bottomMargin = dp(6)
        })
        updates += {
            val v = read()
            value.text = String.format(Locale.US, format, v)
            bar.value = v
        }
    }

    /** Tier 2, drawn differently on purpose: one wheel as two vertical columns,
     *  the command (thick) and the measured speed (thin, auto-ranged); up is forward. */
    private fun wheel(name: String, cmd: () -> Double, speed: () -> Double): LinearLayout {
        val col = vstack().apply { gravity = Gravity.CENTER_HORIZONTAL }
        col.addView(text(name, Palette.INK_2, 14f).apply { gravity = Gravity.CENTER })
        val bars = hstack().apply { gravity = Gravity.BOTTOM or Gravity.CENTER_HORIZONTAL }
        val c = BarView(activity, -1.0, 1.0, signed = true, vertical = true)
        val sp = BarView(activity, -1.0, 1.0, signed = true, autoRange = true, vertical = true)
        bars.addView(c, LinearLayout.LayoutParams(dp(48), dp(104)))
        bars.addView(sp, LinearLayout.LayoutParams(dp(10), dp(104)).apply { marginStart = dp(6) })
        col.addView(bars, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT).apply {
            topMargin = dp(4); bottomMargin = dp(4)
        })
        val cv = text("", Palette.INK, 22f, bold = true).apply { gravity = Gravity.CENTER }
        col.addView(cv)
        val sv = text("", Palette.INK_3, 11f).apply { gravity = Gravity.CENTER }
        col.addView(sv)
        updates += {
            c.value = cmd(); sp.value = speed()
            cv.text = String.format(Locale.US, "%+.2f", c.value)
            sv.text = String.format(Locale.US, "speed %+.0f", sp.value)
        }
        return col
    }

    /** Tier 3: a small stat tile -- label, value, and an optional thin meter. */
    private fun tile(row: LinearLayout, label: String, meter: BarView?, read: () -> Pair<String, Double>) {
        val t = vstack().apply { setPadding(dp(6), 0, dp(6), 0) }
        t.addView(text(label, Palette.INK_3, 11f).apply { gravity = Gravity.CENTER })
        val v = text("", Palette.INK_2, 15f).apply { gravity = Gravity.CENTER }
        t.addView(v)
        if (meter != null) {
            t.addView(meter, LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(4)).apply { topMargin = dp(3) })
        }
        row.addView(t, weight())
        updates += {
            val (s, x) = read()
            v.text = s
            meter?.value = x
        }
    }

    // Per step the reward is at most 1.0 (upright and still) and in practice
    // no lower than about -0.6 (at a bumper, wobbling, wheels at full speed).
    private val rewardBar = BarView(activity, -0.5, 1.0, signed = true)

    init {
        binding.rewardBar.addView(rewardBar,
            LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(64)))

        section("BODY")
        gauge("tilt", "deg", 30.0, "%+7.2f") { thetaDeg }
        gauge("tilt rate", "deg/s", 200.0, "%+7.1f") { angularVelocityDeg }

        section("WHEELS  (command · speed)", inset = 40)
        // Below ~40% of the screen height the robot's cradle covers the sides,
        // narrowing towards the bottom, so everything from here down is inset.
        val wheels = hstack().apply { gravity = Gravity.CENTER; setPadding(dp(48), 0, dp(48), 0) }
        wheels.addView(wheel("left", { actionL }, { wheelSpeedL }), weight())
        wheels.addView(wheel("right", { actionR }, { wheelSpeedR }), weight())
        binding.rows.addView(wheels)

        section("SYSTEM", inset = 40)
        val r1 = hstack().apply { setPadding(dp(40), 0, dp(40), 0) }
        tile(r1, "loop", BarView(activity, 0.0, 50.0, signed = false)) {
            Pair(String.format(Locale.US, "%.1f Hz", controlHz), controlHz)
        }
        tile(r1, "battery", BarView(activity, 3.0, 4.3, signed = false)) {
            Pair(String.format(Locale.US, "%.2f V", batteryVoltage), batteryVoltage)
        }
        tile(r1, "step", null) { Pair(String.format(Locale.US, "%d", step), 0.0) }
        binding.rows.addView(r1)
        val r2 = hstack().apply { setPadding(dp(56), dp(8), dp(56), 0) }
        tile(r2, "count L / R", null) {
            Pair(String.format(Locale.US, "%d / %d", wheelCountL, wheelCountR), 0.0)
        }
        tile(r2, "charger · coil", null) {
            Pair(String.format(Locale.US, "%.2f · %.2f V", chargerVoltage, coilVoltage), 0.0)
        }
        binding.rows.addView(r2)
        spacer()
    }

    fun displayValues() {
        activity.runOnUiThread {
            val color = when {
                rewardAvg >= 0.7 -> Palette.GOOD
                rewardAvg >= 0.3 -> Palette.FAIR
                else -> Palette.POOR
            }
            rewardBar.positive = color
            rewardBar.negative = color
            rewardBar.value = rewardAvg
            rewardBar.label = String.format(Locale.US, "%+.2f", rewardAvg)
            binding.rewardEpisode.text = String.format(Locale.US,
                "this episode %+.2f/step     last %s",
                episodeMean,
                if (lastEpisodeMean.isNaN()) "  -" else String.format(Locale.US, "%+.2f/step", lastEpisodeMean))
            updates.forEach { it() }
            // Always the same lines, so a longer status never moves the layout.
            val trainer = trainerStatus.substringBefore(" (")
                .replace("connected to ", "trainer ").replace("policy ", "policy …")
            binding.status.text = String.format(Locale.US, "%s\n%s", trainer.take(28),
                if (serialNote.isEmpty()) "serial ok" else "serial " + serialNote
                    .replace(" missed replies, last ", " missed, last "))
        }
    }
}
