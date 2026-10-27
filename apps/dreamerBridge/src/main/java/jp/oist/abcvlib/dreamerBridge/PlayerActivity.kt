package jp.oist.abcvlib.dreamerBridge

import android.os.Bundle
import android.os.Process

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
        setStopVisible(true)
    }

    /** Wheels off now, then back to the policy list below this screen. */
    override fun stopAndReturn() {
        brake()
        finish()
    }

    override fun onDestroy() {
        super.onDestroy()
        Process.killProcess(Process.myPid())
    }
}
