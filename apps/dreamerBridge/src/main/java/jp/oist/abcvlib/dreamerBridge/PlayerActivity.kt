package jp.oist.abcvlib.dreamerBridge

import android.os.Process

/**
 * The robot screen running one saved policy, with no trainer (Dreamer Player).
 *
 * The same activity as the bridge; the policy path in the intent is what puts
 * main.py into player mode. It runs in its own process (":player") so its
 * Python never meets a bridge control loop, and ends that process when it
 * closes: Python cannot be restarted in place, so the next policy chosen in
 * the picker gets a fresh one. The base stops the wheels 250 ms after the
 * commands stop.
 */
class PlayerActivity : MainActivity() {
    override fun onDestroy() {
        super.onDestroy()
        Process.killProcess(Process.myPid())
    }
}
