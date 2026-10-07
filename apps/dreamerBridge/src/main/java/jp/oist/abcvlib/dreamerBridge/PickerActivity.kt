package jp.oist.abcvlib.dreamerBridge

import android.app.Activity
import android.content.Intent
import android.graphics.Color
import android.graphics.Typeface
import android.os.Bundle
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.Switch
import android.widget.TextView
import org.json.JSONObject
import java.io.File

/**
 * Dreamer Player's first screen: the saved policies, one per row. Choosing one
 * opens the robot screen running it (PlayerActivity).
 *
 * Policies are <name>.npz files with a <name>.json sidecar in the app's
 * external files dir under policies/, put there by tools/save_policy.sh in the
 * dreamerv3 repo; the list is reread every time this screen shows.
 */
class PickerActivity : Activity() {

    private lateinit var list: LinearLayout
    private lateinit var sample: Switch

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(BACKGROUND)
            setPadding(dp(20), dp(32), dp(20), dp(16))
        }
        root.addView(label("DREAMER PLAYER", MUTED, 11f, bold = true).apply {
            letterSpacing = 0.1f
        })
        root.addView(label("Choose a policy", Color.WHITE, 24f, bold = true).apply {
            setPadding(0, dp(6), 0, dp(16))
        })
        list = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        root.addView(ScrollView(this).apply { addView(list) },
            LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f))
        sample = Switch(this).apply {
            text = "Sample actions, as in training"
            setTextColor(MUTED)
            textSize = 14f
            setPadding(0, dp(12), 0, 0)
        }
        root.addView(sample)
        setContentView(root)
    }

    override fun onResume() {
        super.onResume()
        list.removeAllViews()
        val dir = File(getExternalFilesDir(null), "policies")
        val files = dir.listFiles { f -> f.name.endsWith(".npz") }?.sortedBy { it.name }.orEmpty()
        if (files.isEmpty()) {
            list.addView(label("No policies in ${dir.path}.\n\nSave one from the Mac with\n" +
                    "tools/save_policy.sh <name>", MUTED, 14f))
        }
        files.forEach { list.addView(row(it)) }
    }

    private fun row(npz: File): LinearLayout {
        val name = npz.nameWithoutExtension
        val side = File(npz.parentFile, "$name.json")
        val detail = try {
            val j = JSONObject(side.readText())
            listOf(
                j.optString("published"),
                "%.0f Hz".format(j.optDouble("hz", 25.0)),
                j.optString("job").substringBefore("-"),
            ).filter { it.isNotEmpty() }.joinToString("  ·  ")
        } catch (e: Exception) {
            "no settings file; runs at 25 Hz"
        }
        return LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(CARD)
            setPadding(dp(16), dp(14), dp(16), dp(14))
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT
            ).apply { bottomMargin = dp(10) }
            addView(label(name, Color.WHITE, 20f, bold = true))
            addView(label(detail, MUTED, 13f).apply { setPadding(0, dp(4), 0, 0) })
            isClickable = true
            setOnClickListener {
                startActivity(Intent(this@PickerActivity, PlayerActivity::class.java)
                    .putExtra(MainActivity.EXTRA_POLICY, npz.path)
                    .putExtra(MainActivity.EXTRA_EVAL, !sample.isChecked))
            }
        }
    }

    private fun label(s: String, color: Int, size: Float, bold: Boolean = false) =
        TextView(this).apply {
            text = s
            setTextColor(color)
            textSize = size
            typeface = Typeface.create(Typeface.MONOSPACE, if (bold) Typeface.BOLD else Typeface.NORMAL)
        }

    companion object {
        private val BACKGROUND = Color.parseColor("#FF1A1A19")
        private val CARD = Color.parseColor("#FF262624")
        private val MUTED = Color.parseColor("#FF8A8985")
    }
}
