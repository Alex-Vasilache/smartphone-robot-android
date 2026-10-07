package jp.oist.abcvlib.dreamerBridge

import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.os.Bundle
import android.os.Process
import android.view.Gravity
import android.widget.LinearLayout
import android.widget.TextView

/**
 * The robot screen running one saved policy, with no trainer (Dreamer Player).
 *
 * The same activity as the bridge; the policy path in the intent is what puts
 * main.py into player mode. It runs in its own process (":player") so its
 * Python never meets a bridge control loop, and ends that process when it
 * closes: Python cannot be restarted in place, so the next policy chosen in
 * the picker gets a fresh one.
 */
class PlayerActivity : MainActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // Full width at the very bottom of the screen, big enough to hit on a
        // moving robot.
        val density = resources.displayMetrics.density
        val stop = TextView(this).apply {
            text = "STOP"
            gravity = Gravity.CENTER
            setTextColor(Color.WHITE)
            textSize = 22f
            letterSpacing = 0.15f
            typeface = Typeface.create(Typeface.MONOSPACE, Typeface.BOLD)
            background = GradientDrawable().apply {
                setColor(Color.parseColor("#FFC0392B"))
                cornerRadius = 10 * density
            }
            isClickable = true
            setOnClickListener { stopAndReturn() }
        }
        // The status lines keep a strip free below them; the button takes it.
        (binding.status.layoutParams as LinearLayout.LayoutParams).bottomMargin =
            (8 * density).toInt()
        binding.root.addView(stop, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, (64 * density).toInt()
        ).apply { topMargin = (8 * density).toInt() })
    }

    /** Wheels off now, then back to the policy list. */
    private fun stopAndReturn() {
        try {
            // Brake both wheels, with no slew limit so it is immediate.
            outputs.setWheelOutput(0.0f, 0.0f, true, true, 2.0f)
        } catch (e: Exception) {
            // No base (or not up yet): nothing to stop. Ending the process
            // below stops the commands either way, and the base stops the
            // wheels by itself 250 ms after they stop.
        }
        finish()
    }

    override fun onDestroy() {
        super.onDestroy()
        Process.killProcess(Process.myPid())
    }
}
