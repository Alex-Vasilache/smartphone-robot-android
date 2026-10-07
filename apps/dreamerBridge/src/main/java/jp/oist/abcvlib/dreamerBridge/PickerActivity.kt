package jp.oist.abcvlib.dreamerBridge

import android.app.Activity
import android.app.AlertDialog
import android.content.Intent
import android.graphics.Color
import android.graphics.Typeface
import android.os.Bundle
import android.text.InputType
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.Switch
import android.widget.TextView
import org.json.JSONObject
import java.io.File

/**
 * The policy list, reached with back from the training screen.
 *
 * The top row returns to training. Below it, every saved policy: a <name>.npz
 * with a <name>.json sidecar under policies/ in the app's external files dir.
 * Each training run keeps its latest weights there under the run's name
 * (main.py:weight_saver); tools/save_policy.sh adds others. Tap one to run it
 * with no trainer (PlayerActivity); long-press to rename or delete it.
 */
class PickerActivity : Activity() {

    private lateinit var list: LinearLayout
    private lateinit var sample: Switch
    private val dir by lazy { File(getExternalFilesDir(null), "policies") }

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(BACKGROUND)
            setPadding(dp(20), dp(32), dp(20), dp(16))
        }
        root.addView(label("DREAMER", MUTED, 11f, bold = true).apply { letterSpacing = 0.1f })
        root.addView(label("Train or run a policy", Color.WHITE, 24f, bold = true).apply {
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
        refresh()
    }

    private fun refresh() {
        list.removeAllViews()
        list.addView(card("Train", "connect to the trainer and learn", TRAIN) {
            startActivity(Intent(this, MainActivity::class.java))
            finish()
        })
        list.addView(label("POLICIES  ·  long-press to rename", MUTED, 11f, bold = true).apply {
            letterSpacing = 0.1f
            setPadding(0, dp(14), 0, dp(10))
        })
        val files = dir.listFiles { f -> f.name.endsWith(".npz") }
            ?.sortedByDescending { it.lastModified() }.orEmpty()
        if (files.isEmpty()) {
            list.addView(label("None yet. Every training run keeps its latest weights here.",
                MUTED, 14f))
        }
        files.forEach { npz ->
            list.addView(card(npz.nameWithoutExtension, details(npz), CARD,
                onLong = { edit(npz) }) {
                startActivity(Intent(this, PlayerActivity::class.java)
                    .putExtra(MainActivity.EXTRA_POLICY, npz.path)
                    .putExtra(MainActivity.EXTRA_EVAL, !sample.isChecked))
            })
        }
    }

    private fun details(npz: File): String = try {
        val j = JSONObject(sidecar(npz).readText())
        val run = j.optString("job").substringBefore("-")
        listOf(
            j.optString("published"),
            "%.0f Hz".format(j.optDouble("hz", 25.0)),
            if (run != npz.nameWithoutExtension) run else "",
        ).filter { it.isNotEmpty() }.joinToString("  ·  ")
    } catch (e: Exception) {
        "no settings file; runs at 25 Hz"
    }

    private fun sidecar(npz: File) = File(npz.parentFile, npz.nameWithoutExtension + ".json")

    /** Rename (keeping the sidecar with it) or delete a saved policy. */
    private fun edit(npz: File) {
        val input = EditText(this).apply {
            setText(npz.nameWithoutExtension)
            inputType = InputType.TYPE_CLASS_TEXT
            setSelection(text.length)
        }
        AlertDialog.Builder(this)
            .setTitle("Rename policy")
            .setView(input)
            .setPositiveButton("Rename") { _, _ ->
                val name = input.text.toString().trim()
                    .replace(Regex("[^A-Za-z0-9._-]"), "_")
                if (name.isEmpty() || name == npz.nameWithoutExtension) return@setPositiveButton
                val target = File(dir, "$name.npz")
                if (target.exists()) {
                    toastLike("$name already exists")
                    return@setPositiveButton
                }
                val side = sidecar(npz)
                npz.renameTo(target)
                if (side.exists()) {
                    val j = try { JSONObject(side.readText()) } catch (e: Exception) { JSONObject() }
                    j.put("name", name)
                    File(dir, "$name.json").writeText(j.toString(1))
                    side.delete()
                }
                refresh()
            }
            .setNeutralButton("Delete") { _, _ ->
                AlertDialog.Builder(this)
                    .setTitle("Delete ${npz.nameWithoutExtension}?")
                    .setPositiveButton("Delete") { _, _ ->
                        npz.delete()
                        sidecar(npz).delete()
                        refresh()
                    }
                    .setNegativeButton("Cancel", null)
                    .show()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun toastLike(s: String) =
        android.widget.Toast.makeText(this, s, android.widget.Toast.LENGTH_SHORT).show()

    private fun card(title: String, detail: String, color: Int, onLong: (() -> Unit)? = null,
                     onTap: () -> Unit) =
        LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(color)
            setPadding(dp(16), dp(14), dp(16), dp(14))
            layoutParams = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT
            ).apply { bottomMargin = dp(10) }
            addView(label(title, Color.WHITE, 20f, bold = true))
            addView(label(detail, if (color == TRAIN) Color.WHITE else MUTED, 13f).apply {
                setPadding(0, dp(4), 0, 0)
            })
            isClickable = true
            setOnClickListener { onTap() }
            if (onLong != null) setOnLongClickListener { onLong(); true }
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
        private val TRAIN = Color.parseColor("#FF2E6DA4")
        private val MUTED = Color.parseColor("#FF8A8985")
    }
}
