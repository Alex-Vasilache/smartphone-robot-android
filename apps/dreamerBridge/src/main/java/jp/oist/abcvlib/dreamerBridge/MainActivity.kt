package jp.oist.abcvlib.dreamerBridge

import android.content.Context
import android.net.wifi.WifiManager
import android.content.Intent
import android.os.Bundle
import android.os.Process
import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.view.Gravity
import android.view.View
import android.widget.LinearLayout
import android.widget.TextView
import androidx.activity.addCallback
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import androidx.lifecycle.lifecycleScope
import com.chaquo.python.PyException
import com.chaquo.python.Python
import com.chaquo.python.android.AndroidPlatform
import jp.oist.abcvlib.dreamerBridge.databinding.ActivityMainBinding
import jp.oist.abcvlib.core.AbcvlibActivity
import jp.oist.abcvlib.util.Logger
import jp.oist.abcvlib.util.SerialReadyListener
import jp.oist.abcvlib.util.UsbSerial
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import org.json.JSONObject
import java.io.File

open class MainActivity : AbcvlibActivity(), SerialReadyListener {
    // keep them public to be visible for python
    lateinit var binding: ActivityMainBinding
    lateinit var guiUpdater: GuiUpdater

    /**
     * Keeps the WiFi radio out of power save for the length of a run.
     *
     * Measured 2026-09-03: without it the control loop's own tick was clean at
     * 20.0ms but the trainer received those frames in clumps -- p90 46ms, p99
     * 176ms between arrivals, 1.47 frames per wake -- and ICMP to this phone
     * ran 6ms best case against a 129ms average. That is the radio dozing
     * between beacons and the access point buffering for it. It cost a third
     * of every observation at 50Hz.
     */
    private var wifiLock: WifiManager.WifiLock? = null

    /** STOP, shown while a saved policy drives (setStopVisible). */
    private lateinit var stopButton: TextView
    private var statusMargin = -1   // the layout's own, restored on hiding

    override fun onCreate(savedInstanceState: Bundle?) {
        enableMainLoop(false)
        // The serial reader hex-dumps every chunk at debug level; that runs
        // between the RP2040's reply and the control loop seeing it.
        Logger.setQuiet(true)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        // Full screen: the display is read from a distance on a moving robot,
        // so every pixel goes to the gauges. Swipe from an edge to see the bars.
        supportActionBar?.hide()
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, binding.root).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
        guiUpdater = GuiUpdater(binding, this)
        addStopButton()
        // A robot run is unattended; letting the screen sleep would suspend the
        // control loop and the trainer would sit blocked waiting for us.
        binding.root.keepScreenOn = true
        val wifi = applicationContext.getSystemService(Context.WIFI_SERVICE)
                as WifiManager
        wifiLock = wifi.createWifiLock(
            WifiManager.WIFI_MODE_FULL_HIGH_PERF, "dreamerBridge:link"
        ).apply {
            setReferenceCounted(false)
            acquire()
        }
        super.onCreate(savedInstanceState)
        // Training mode: back goes to the policy list (PickerActivity), which
        // can replay any saved policy or come back here. Python cannot stop in
        // place, so leaving training ends this process; the base stops the
        // wheels by itself 250 ms after the commands stop.
        if (this !is PlayerActivity) {
            onBackPressedDispatcher.addCallback(this) {
                startActivity(Intent(this@MainActivity, PickerActivity::class.java))
                finish()
                Process.killProcess(Process.myPid())
            }
        }
        // No base: the serial link never comes up, so onSerialReady never
        // fires. Start the bridge anyway; main.py skips the serial and wheels.
        if (noBase()) {
            Logger.i("MainActivity", "no_base set in trainer.json; starting without the robot")
            initPython()
        }
    }

    /**
     * Full width at the very bottom of the screen, big enough to hit on a
     * moving robot. Hidden until a saved policy drives: always in the Dreamer
     * Player, and when one is chosen on the web dashboard during training.
     */
    private fun addStopButton() {
        val density = resources.displayMetrics.density
        stopButton = TextView(this).apply {
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
            visibility = View.GONE
            setOnClickListener { stopAndReturn() }
        }
        binding.root.addView(stopButton, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, (64 * density).toInt()
        ).apply { topMargin = (8 * density).toInt() })
    }

    /** From Python, on any thread. */
    fun setStopVisible(visible: Boolean) {
        runOnUiThread {
            stopButton.visibility = if (visible) View.VISIBLE else View.GONE
            // The status lines keep a strip free below them; the button takes it.
            val params = binding.status.layoutParams as LinearLayout.LayoutParams
            if (statusMargin < 0) statusMargin = params.bottomMargin
            params.bottomMargin =
                if (visible) (8 * resources.displayMetrics.density).toInt() else statusMargin
            binding.status.requestLayout()
        }
    }

    /** Brake both wheels at once, with no slew limit. */
    protected fun brake() {
        try {
            outputs.setWheelOutput(0.0f, 0.0f, true, true, 2.0f)
        } catch (e: Exception) {
            // No base (or not up yet): nothing to stop. Ending the process
            // stops the commands either way, and the base stops the wheels by
            // itself 250 ms after they stop.
        }
    }

    /** Wheels off now, then to the policy list, as the back button does. */
    protected open fun stopAndReturn() {
        brake()
        startActivity(Intent(this, PickerActivity::class.java))
        finish()
        Process.killProcess(Process.myPid())
    }

    private fun noBase(): Boolean = try {
        val file = File(getExternalFilesDir(null), "trainer.json")
        file.exists() && JSONObject(file.readText()).optBoolean("no_base", false)
    } catch (e: Exception) {
        false
    }

    override fun onDestroy() {
        // Held for the length of a run, not the length of the process: leaving
        // the radio pinned after the app is gone is a battery bug.
        wifiLock?.let { if (it.isHeld) it.release() }
        wifiLock = null
        super.onDestroy()
    }

    override fun onSerialReady(usbSerial: UsbSerial) {
        initPython()
    }

    private fun initPython() {
        // With no_base set while a base is plugged in, the no-base start in
        // onCreate and onSerialReady (on a coroutine) both get here at once,
        // and two racing Python.start calls crashed the app with "Python
        // already started" (2026-10-08). A second run() is fine: abcvlib.py
        // keeps the one control loop and only restarts the serial side.
        synchronized(MainActivity::class.java) {
            if (!Python.isStarted()) {
                Python.start(AndroidPlatform(this))
            }
        }

        val py = Python.getInstance()
        val setupModule = py.getModule("abcvlib")
        // inject variables to python
        // main.loop() is one full control step and paces itself against
        // CONTROL_HZ, so the bridge must not add a delay of its own.
        setupModule.put("loop_delay", 0.0)
        setupModule.put("context", this)
        // Set only when started from the player's picker (PlayerActivity).
        setupModule.put("player_policy", intent.getStringExtra(EXTRA_POLICY))
        setupModule.put("player_eval", intent.getBooleanExtra(EXTRA_EVAL, true))

        lifecycleScope.launch(Dispatchers.Default) {
            try {
                setupModule.callAttr("run")
            } catch (e: PyException) {
                Logger.e("MainActivity", "Python error: ${e.message}")
            }
        }
    }

    /**
     * Called from Python after setup is complete,
     * since `super.onSerialReady()` can't be invoked from Python directly
     */
    fun onSetupReady() {
        super.onSerialReady(usbSerial)
    }

    companion object {
        const val EXTRA_POLICY = "jp.oist.abcvlib.dreamerBridge.POLICY"
        const val EXTRA_EVAL = "jp.oist.abcvlib.dreamerBridge.EVAL"
    }
}
