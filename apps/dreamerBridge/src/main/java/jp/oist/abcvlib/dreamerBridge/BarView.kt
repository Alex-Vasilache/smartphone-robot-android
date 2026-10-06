package jp.oist.abcvlib.dreamerBridge

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.RectF
import android.view.View
import kotlin.math.abs
import kotlin.math.max

/**
 * A horizontal gauge for one number. Signed quantities grow out of a centre
 * zero line; unsigned ones fill from the left edge of [lo, hi]. A value past
 * the range pins to the end and turns red, so clipping is never silent.
 *
 * With [autoRange] the range follows the largest magnitude seen, decaying
 * slowly, for quantities with no natural scale (wheel speed).
 */
class BarView(
    context: Context,
    private var lo: Double,
    private var hi: Double,
    private val signed: Boolean,
    private val autoRange: Boolean = false,
) : View(context) {

    var value: Double = 0.0
        set(v) {
            field = v
            if (autoRange && v.isFinite()) {
                val span = max(max(abs(v) * 1.1, hi * 0.995), minAutoSpan)
                hi = span
                lo = if (signed) -span else 0.0
            }
            invalidate()
        }

    /** The current range, for a label beside the bar. */
    fun rangeLabel(): String = if (signed) "±" + short(hi) else short(lo) + "–" + short(hi)

    private fun short(v: Double): String =
        if (v == Math.rint(v) || abs(v) >= 10) String.format(java.util.Locale.US, "%.0f", v)
        else String.format(java.util.Locale.US, "%.1f", v)

    /** Override the fill colour (e.g. to match a value's own colour). */
    var fillColor: Int? = null
        set(c) { field = c; invalidate() }

    private val minAutoSpan = if (autoRange) max(abs(hi), abs(lo)) else 0.0

    private val track = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFF2C2C2C.toInt() }
    private val zero = Paint().apply { color = 0xFF8A8A8A.toInt(); strokeWidth = 2f }
    private val positive = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFF4FC3F7.toInt() }
    private val negative = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFFFFB74D.toInt() }
    private val clipped = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = 0xFFEF5350.toInt() }
    private val rect = RectF()

    private fun x(v: Double): Float {
        val f = ((v - lo) / (hi - lo)).coerceIn(0.0, 1.0)
        return (f * width).toFloat()
    }

    override fun onDraw(canvas: Canvas) {
        val h = height.toFloat()
        val r = h / 2f
        rect.set(0f, 0f, width.toFloat(), h)
        canvas.drawRoundRect(rect, r, r, track)
        if (!value.isFinite()) return
        val base = if (signed) x(0.0) else 0f
        val end = x(value)
        val override = fillColor
        val paint = when {
            override != null -> positive.also { it.color = override }
            value > hi || value < lo -> clipped
            signed && value < 0 -> negative
            else -> positive
        }
        rect.set(minOf(base, end), 0f, max(base, end), h)
        canvas.drawRoundRect(rect, r, r, paint)
        if (signed) canvas.drawLine(base, 0f, base, h, zero)
    }
}
