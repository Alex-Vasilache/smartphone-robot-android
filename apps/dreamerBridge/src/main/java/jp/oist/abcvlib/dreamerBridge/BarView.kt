package jp.oist.abcvlib.dreamerBridge

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.graphics.Typeface
import android.view.View
import java.util.Locale
import kotlin.math.abs
import kotlin.math.max

/**
 * A gauge for one number over [lo, hi], horizontal or vertical.
 *
 * Signed quantities grow out of a zero line in one of two colours by sign (a
 * diverging scale); unsigned ones fill from the low end. A value past the range
 * pins to the end and gets a white end marker, so clipping is visible without
 * spending a colour on it. With [autoRange] the range follows the largest
 * magnitude seen, decaying slowly, for quantities with no natural scale.
 */
class BarView(
    context: Context,
    private var lo: Double,
    private var hi: Double,
    private val signed: Boolean,
    private val autoRange: Boolean = false,
    private val vertical: Boolean = false,
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

    /** Fill colours: [positive] for unsigned and for values above zero. */
    var positive: Int = Palette.BLUE
    var negative: Int = Palette.RED
    var track: Int = Palette.TRACK

    /** Text drawn centred on the bar, e.g. its own value. */
    var label: String? = null
        set(t) { field = t; invalidate() }

    /** The current range, for a label beside the bar. */
    fun rangeLabel(): String = if (signed) "±" + short(hi) else short(lo) + "–" + short(hi)

    private fun short(v: Double): String =
        if (v == Math.rint(v) || abs(v) >= 10) String.format(Locale.US, "%.0f", v)
        else String.format(Locale.US, "%.1f", v)

    private val minAutoSpan = if (autoRange) max(abs(hi), abs(lo)) else 0.0
    private val density = resources.displayMetrics.density
    private val paint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val zero = Paint().apply { color = Palette.ZERO; strokeWidth = 2f * density }
    private val cap = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Palette.INK }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Palette.INK
        typeface = Typeface.create(Typeface.MONOSPACE, Typeface.BOLD)
        textAlign = Paint.Align.CENTER
        setShadowLayer(4f, 0f, 0f, 0xFF000000.toInt())
    }
    private val rect = RectF()
    private val path = Path()

    /** Position of v along the bar, in pixels from the low end. */
    private fun pos(v: Double): Float {
        val f = ((v - lo) / (hi - lo)).coerceIn(0.0, 1.0)
        val length = if (vertical) height else width
        return (f * length).toFloat()
    }

    override fun onDraw(canvas: Canvas) {
        val w = width.toFloat()
        val h = height.toFloat()
        val r = minOf((if (vertical) w else h) / 2f, 4f * density)
        paint.color = track
        rect.set(0f, 0f, w, h)
        canvas.drawRoundRect(rect, r, r, paint)
        if (value.isFinite()) {
            val a = if (signed) pos(0.0) else 0f
            val b = pos(value)
            paint.color = if (signed && value < 0) negative else positive
            if (vertical) {
                // The low end is at the bottom, so forward reads as up.
                rect.set(0f, h - max(a, b), w, h - minOf(a, b))
            } else {
                rect.set(minOf(a, b), 0f, max(a, b), h)
            }
            canvas.drawRoundRect(rect, r, r, paint)
            if (value > hi || value < lo) drawCap(canvas, value > hi, w, h)
            if (signed) {
                if (vertical) canvas.drawLine(0f, h - a, w, h - a, zero)
                else canvas.drawLine(a, 0f, a, h, zero)
            }
        }
        label?.let {
            text.textSize = minOf(h * 0.55f, w * 0.12f)
            canvas.drawText(it, w / 2f, h / 2f - (text.descent() + text.ascent()) / 2f, text)
        }
    }

    /** A small white triangle at the end the value ran off. */
    private fun drawCap(canvas: Canvas, high: Boolean, w: Float, h: Float) {
        val s = 5f * density
        path.reset()
        if (vertical) {
            val y = if (high) 0f else h
            val d = if (high) s else -s
            path.moveTo(w / 2f - s, y + d); path.lineTo(w / 2f + s, y + d); path.lineTo(w / 2f, y)
        } else {
            val x = if (high) w else 0f
            val d = if (high) -s else s
            path.moveTo(x + d, h / 2f - s); path.lineTo(x + d, h / 2f + s); path.lineTo(x, h / 2f)
        }
        path.close()
        canvas.drawPath(path, cap)
    }
}

/**
 * Colours from the dataviz skill's validated dark palette: a blue/red diverging
 * pair for signed values, status colours reserved for the reward's state, and
 * neutral ink for all text.
 */
object Palette {
    const val SURFACE = 0xFF1A1A19.toInt()
    const val TRACK = 0xFF383835.toInt()
    const val ZERO = 0xFF8A8985.toInt()
    const val INK = 0xFFFFFFFF.toInt()
    const val INK_2 = 0xFFC3C2B7.toInt()
    const val INK_3 = 0xFF8A8985.toInt()
    // Softened steps, re-validated (dark surface, all pairs): deutan ΔE 11.5,
    // normal-vision ΔE 15.4, chroma >= 0.1. Lighter than the categorical band,
    // which is the point of softening and reads fine on this surface.
    const val BLUE = 0xFF6A9FD8.toInt()
    const val RED = 0xFFD98080.toInt()
    // Reward state. Not green/amber/red: green vs red measured ΔE 4.1 under
    // deuteranopia (validate_palette.js --pairs all), i.e. the same colour.
    // Blue/yellow/red holds there. The colour is redundant with the bar's
    // length and the printed value, so it never carries the meaning alone.
    const val GOOD = BLUE
    const val FAIR = 0xFFD9B45A.toInt()
    const val POOR = RED
}
