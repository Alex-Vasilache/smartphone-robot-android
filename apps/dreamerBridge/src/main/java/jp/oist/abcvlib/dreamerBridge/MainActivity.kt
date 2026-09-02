package jp.oist.abcvlib.dreamerBridge

import android.os.Bundle
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

class MainActivity : AbcvlibActivity(), SerialReadyListener {
    // keep them public to be visible for python
    lateinit var binding: ActivityMainBinding
    lateinit var guiUpdater: GuiUpdater

    override fun onCreate(savedInstanceState: Bundle?) {
        enableMainLoop(false)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        guiUpdater = GuiUpdater(binding, this)
        // A robot run is unattended; letting the screen sleep would suspend the
        // control loop and the trainer would sit blocked waiting for us.
        binding.root.keepScreenOn = true
        super.onCreate(savedInstanceState)
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
}

