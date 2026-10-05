package jp.oist.abcvlib.core.inputs.phone

import android.content.Context
import android.hardware.Sensor
import android.hardware.Sensor.TYPE_ACCELEROMETER_UNCALIBRATED
import android.hardware.Sensor.TYPE_GYROSCOPE
import android.hardware.Sensor.TYPE_ROTATION_VECTOR
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import jp.oist.abcvlib.core.inputs.Publisher
import jp.oist.abcvlib.core.inputs.PublisherManager
import jp.oist.abcvlib.util.Logger
import jp.oist.abcvlib.util.SensorLatencyTrace
import android.os.SystemClock

/**
 * MotionSensors reads and processes the data from the Android phone gyroscope and
 * accelerometer. The three main goals of this class are:
 * 
 * 1.) To estimate the tilt angle of the phone by combining the input from the
 * gyroscope and accelerometer
 * 2.) To calculate the speed of each wheel (speedRightWheel and speedLeftWheel) by using
 * existing and past quadrature encoder states
 * 3.) Provides the get() methods for all relevant sensory variables
 * 
 * This thread typically updates every 5 ms, but this depends on the
 * SensorManager.SENSOR_DELAY_FASTEST value. This represents the fastest possible sampling rate for
 * the sensor. Note this value can change depending on the Android phone used and corresponding
 * internal sensor hardware.
 * 
 * A counter is keep track of the number of sensor change events via sensorChangeCount.
 * 
 * @author Jiexin Wang https://github.com/ha5ha6
 * @author Christopher Buckley https://github.com/topherbuckley
 */
class OrientationData(context: Context, publisherManager: PublisherManager) :
    Publisher<OrientationDataSubscriber>(context, publisherManager), SensorEventListener {
    /*
         * Keeps track of current history index.
         * indexCurrent calculates the correct index within the time history arrays in order to
         * continuously loop through and rewrite the encoderCountHistoryLength indexes.
         * E.g. if sensorChangeCount is 15 and encoderCountHistoryLength is 15, then indexCurrent
         * will resolve to 0 and the values for each history array will be updated from index 0 until
         * sensorChangeCount exceeds 30 at which point it will loop to 0 again, so on and so forth.
         */
    private var indexCurrentRotation = 1

    /**
     * Length of past timestamps and encoder values you keep in memory. 15 is not significant,
     * just what was deemed appropriate previously.
     */
    private var windowLength = 3

    //Total number of times the sensors have changed data
    private var sensorChangeCountRotation = 1

    private val sensorManager: SensorManager =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val gyroscope: Sensor? = sensorManager.getDefaultSensor(TYPE_GYROSCOPE)
    private val rotationSensor: Sensor? = sensorManager.getDefaultSensor(TYPE_ROTATION_VECTOR)
    private val accelerometer: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private var accelerometerUncalibrated: Sensor? =
        sensorManager.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED)

    /**
     * orientation vector See link below for android doc
     * https://developer.android.com/reference/android/hardware/SensorManager.html#getOrientation(float%5B%5D,%2520float%5B%5D)
     */
    private val orientation = FloatArray(3)

    /**
     * thetaRad calculated from rotation vector
     */
    private val thetaRad = DoubleArray(windowLength)

    /**
     * rotation matrix
     */
    private val rotationMatrix = FloatArray(16)

    /**
     * rotation matrix remapped
     */
    private val rotationMatrixRemap = FloatArray(16)

    /**
     * angularVelocity calculated from RotationMatrix.
     */
    private val angularVelocityRad = DoubleArray(windowLength)

    private var timerCount: Int = 1

    //----------------------------------------------------------------------------------------------
    //----------------------------------------- Timestamps -----------------------------------------
    /**
     * Keeps track of both gyro and accelerometer sensor timestamps
     */
    private val timeStamps = LongArray(windowLength)

    /**
     * indexHistoryOldest calculates the index for the oldest encoder count still within
     * the history. Using the most recent historical point could lead to speed calculations of zero
     * in the event the quadrature encoders slip/skip and read the same wheel count two times in a
     * row even if the wheel is moving with a non-zero speed.
     */
    private var indexHistoryOldest: Int = 0 // Keeps track of oldest history index.
    private var dt: Double = 0.0


    class Builder(private val context: Context, private val publisherManager: PublisherManager) {
        fun build(): OrientationData {
            return OrientationData(context, publisherManager)
        }
    }

    /**
     * Assume this is only used for sensors that have the ability to change accuracy (e.g. GPS)
     * @param sensor Sensor object that has changed its accuracy
     * @param accuracy Accuracy. See SensorEvent on Android Dev documentation for details
     */
    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
        // Not sure if we need to worry about this. I think this is more for more variable sensors like GPS but could be wrong.
    }

    /**
     * This is called every time a registered sensor provides data. Sensor must be registered before
     * it will fire the event which calls this method. If statements handle differentiating between
     * accelerometer and gyroscope events.
     * @param event SensorEvent object that has updated its output
     */
    override fun onSensorChanged(event: SensorEvent) {
        val sensor = event.sensor

        if (sensor.type == TYPE_GYROSCOPE) {
            // Diagnostic only: nothing downstream consumes this. It exists so
            // the fused rotation vector can be compared against a signal that
            // has no fusion filter in front of it.
            SensorLatencyTrace.gyroX = event.values[0]
            SensorLatencyTrace.gyroY = event.values[1]
            SensorLatencyTrace.gyroZ = event.values[2]
            SensorLatencyTrace.gyroAgeMs =
                (SystemClock.elapsedRealtimeNanos() - event.timestamp) / 1e6
            SensorLatencyTrace.gyroCallbackNs = System.nanoTime()
            SensorLatencyTrace.gyroCount++
            return
        }

        // if(sensor.getType()==Sensor.TYPE_GYROSCOPE){
        // indexCurrentGyro = sensorChangeCountGyro % windowLength;
        // indexPreviousGyro = (sensorChangeCountGyro - 1) % windowLength;
        // Rotation around x-axis
        // See https://developer.android.com/reference/android/hardware/SensorEvent.html
        // thetaDotGyro = event.values[0];
        // thetaDotGyroDeg = (thetaDotGyro * (180 / Math.PI));
        // timeStampsGyro[indexCurrentGyro] = event.timestamp;
        // dtGyro = (timeStampsGyro[indexCurrentGyro] - timeStampsGyro[indexPreviousGyro]) / 1000000000f;
        // sensorChangeCountGyro++;
        // if (loggerOn){
        //         sendToLog();
        //     }
        // }
        if (sensor.type == TYPE_ROTATION_VECTOR) {
            // Timer for only TYPE_ROTATION_VECTOR sensor change
            indexCurrentRotation = sensorChangeCountRotation % windowLength
            val indexPreviousRotation = (sensorChangeCountRotation - 1) % windowLength
            indexHistoryOldest = (sensorChangeCountRotation + 1) % windowLength
            timeStamps[indexCurrentRotation] = event.timestamp
            dt = ((timeStamps[indexCurrentRotation] - timeStamps[indexPreviousRotation])
                    / 1000000000f).toDouble()

            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
            SensorManager.remapCoordinateSystem(
                rotationMatrix,
                SensorManager.AXIS_MINUS_X,
                SensorManager.AXIS_Z,
                rotationMatrixRemap
            )
            SensorManager.getOrientation(rotationMatrixRemap, orientation)
            thetaRad[indexCurrentRotation] = orientation[1].toDouble() //Pitch
            angularVelocityRad[indexCurrentRotation] =
                (thetaRad[indexCurrentRotation] - thetaRad[indexPreviousRotation]) / dt

            // Both clock bases, so the age can be checked rather than believed.
            SensorLatencyTrace.rotAgeMs =
                (SystemClock.elapsedRealtimeNanos() - event.timestamp) / 1e6
            SensorLatencyTrace.rotAgeUptimeMs =
                (System.nanoTime() - event.timestamp) / 1e6
            SensorLatencyTrace.rotCallbackNs = System.nanoTime()
            SensorLatencyTrace.rotCount++

            // Update all previous variables with current ones
            sensorChangeCountRotation++
        }

        timerCount++

        if (!paused) {
            for (subscriber in subscribers) {
                subscriber.onOrientationUpdate(
                    timeStamps[indexCurrentRotation],
                    thetaRad[indexCurrentRotation],
                    angularVelocityRad[indexCurrentRotation]
                )
            }
        }
    }

    /**
     * Registering sensorEventListeners for accelerometer and gyroscope only.
     */
    fun register(handler: Handler) {
        // Only the rotation vector is registered, and at a bounded rate.
        //
        // This used to register the rotation vector, gyroscope, accelerometer
        // and uncalibrated accelerometer, all at SENSOR_DELAY_FASTEST, onto a
        // single sensorThread -- while onSensorChanged only ever handles
        // TYPE_ROTATION_VECTOR, so three of the four produced events purely to
        // be discarded. The thread could not drain that, and the queue reached
        // a standing backlog: measured on a Pixel 3a, sensor events arrived at
        // the app 8.5 SECONDS after their hardware timestamp, which makes any
        // closed-loop control impossible while looking like a healthy signal.
        if (rotationSensor != null) {
            sensorManager.registerListener(
                this,
                rotationSensor,
                SAMPLING_PERIOD_US,
                handler
            )
        } else {
            Logger.e("SensorTesting", "No rotation vector sensor available")
        }
        if (SensorLatencyTrace.gyroEnabled && gyroscope != null) {
            sensorManager.registerListener(this, gyroscope, SAMPLING_PERIOD_US, handler)
        }
    }

    fun unregister() {
        // Check if rotation_sensor exists before trying to turn off the listener
        if (rotationSensor != null) {
            sensorManager.unregisterListener(this, rotationSensor)
        }
        // Check if gyro exists before trying to turn off the listener
        if (gyroscope != null) {
            sensorManager.unregisterListener(this, gyroscope)
        }
        // Check if rotation_sensor exists before trying to turn off the listener
        if (accelerometer != null) {
            sensorManager.unregisterListener(this, accelerometer)
        }
        // Check if gyro exists before trying to turn off the listener
        if (accelerometerUncalibrated != null) {
            sensorManager.unregisterListener(this, accelerometerUncalibrated)
        }
    }

    /**
     * Sets the history length for which to base the derivative functions off of (angular velocity,
     * linear velocity).
     * @param len length of array for keeping history
     */
    fun setWindowLength(len: Int) {
        windowLength = len
    }

    override fun start() {
        // Urgent priority, not the default. Measured 2026-09-07, an
        // orientation event reaches this callback ~7.4ms after its hardware
        // timestamp, of which only ~2.5ms is sampling aliasing (the sensor
        // reports maxRate=200Hz, no batching, so 5000us is already its
        // ceiling -- SENSOR_DELAY_FASTEST would change nothing). The rest is
        // delivery, and a default-priority HandlerThread competing with the
        // control loop is part of it. This is latency on the critical path of
        // a balancing robot, where the whole budget is ~30ms.
        mHandlerThread = HandlerThread(
            "sensorThread", android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY)
        mHandlerThread.start()
        handler = Handler(mHandlerThread.looper)
        register(handler)
        publisherManager.onPublisherInitialized()
        super.start()
    }

    override fun stop() {
        mHandlerThread.quitSafely()
        unregister()
        super.stop()
    }

    override fun getRequiredPermissions(): ArrayList<String> {
        return arrayListOf()
    }

    companion object {
        /**
         * @return utility function converting radians to degrees
         */
        /**
         * Sampling period in microseconds. 5000us is 200Hz, far above any
         * control loop here and low enough that the sensor thread keeps up.
         * SENSOR_DELAY_FASTEST let the queue build an 8.5s backlog.
         */
        const val SAMPLING_PERIOD_US = 5000

        fun getThetaDeg(radians: Double): Double {
            return (radians * (180 / Math.PI))
        }

        /**
         * @return utility function converting rad/s to deg/s
         */
        fun getAngularVelocityDeg(radPerSec: Double): Double {
            return getThetaDeg(radPerSec)
        }

        /**
         * This seems to be a very convoluted way to do this, but it seems to work just fine
         * @param angle Tilt angle in radians
         * @return Wrapped angle in radians from -Pi to Pi
         */
        private fun wrapAngle(angle: Float): Float {
            var angle = angle
            while (angle < -Math.PI) angle += (2 * Math.PI).toFloat()
            while (angle > Math.PI) angle -= (2 * Math.PI).toFloat()
            return angle
        }
    }
}
