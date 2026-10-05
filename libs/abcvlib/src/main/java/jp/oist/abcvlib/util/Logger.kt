package jp.oist.abcvlib.util

import android.util.Log
import jp.oist.abcvlib.core.BuildConfig

/**
 * Custom logger wrapper.
 * Logs (v/d/i/w) only for DEBUG builds.
 * Errors (e) are always logged.
 */
object Logger {

    /**
     * Mutes v/d/i/w at runtime. In a debug build they are all live, and the
     * serial path logs a hex dump of every USB chunk and a line per parsed
     * byte -- string formatting on the reader thread, between the RP2040's
     * reply and the control loop seeing it. An app whose control loop runs
     * on that reply sets this. Errors are never muted.
     */
    @JvmField
    @Volatile
    var quiet: Boolean = false

    @JvmStatic
    fun setQuiet(value: Boolean) {
        quiet = value
    }

    @JvmStatic
    fun v(tag: String, msg: String) {
        if (BuildConfig.DEBUG && !quiet) Log.v(tag, msg)
    }

    @JvmStatic
    fun v(tag: String, msg: String, tr: Throwable) {
        if (BuildConfig.DEBUG && !quiet) Log.v(tag, msg, tr)
    }

    @JvmStatic
    fun d(tag: String, msg: String) {
        if (BuildConfig.DEBUG && !quiet) Log.d(tag, msg)
    }

    @JvmStatic
    fun d(tag: String, msg: String, tr: Throwable) {
        if (BuildConfig.DEBUG && !quiet) Log.d(tag, msg, tr)
    }

    @JvmStatic
    fun i(tag: String, msg: String) {
        if (BuildConfig.DEBUG && !quiet) Log.i(tag, msg)
    }

    @JvmStatic
    fun i(tag: String, msg: String, tr: Throwable) {
        if (BuildConfig.DEBUG && !quiet) Log.i(tag, msg, tr)
    }

    @JvmStatic
    fun w(tag: String, msg: String) {
        if (BuildConfig.DEBUG && !quiet) Log.w(tag, msg)
    }

    @JvmStatic
    fun w(tag: String, msg: String, tr: Throwable) {
        if (BuildConfig.DEBUG && !quiet) Log.w(tag, msg, tr)
    }

    @JvmStatic
    fun e(tag: String, msg: String) {
        Log.e(tag, msg)
    }

    @JvmStatic
    fun e(tag: String, msg: String, tr: Throwable) {
        Log.e(tag, msg, tr)
    }
}