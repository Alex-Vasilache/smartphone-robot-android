package jp.oist.abcvlib.dreamerBridge

import android.content.Context
import android.net.wifi.WifiManager
import android.os.Bundle
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
        // No base: the serial link never comes up, so onSerialReady never
        // fires. Start the bridge anyway; main.py skips the serial and wheels.
        if (noBase()) {
            Logger.i("MainActivity", "no_base set in trainer.json; starting without the robot")
            initPython()
        }
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
        if (!Python.isStarted()) {
            Python.start(AndroidPlatform(this))
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
