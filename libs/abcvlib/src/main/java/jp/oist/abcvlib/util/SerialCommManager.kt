package jp.oist.abcvlib.util

import android.os.SystemClock
import com.hoho.android.usbserial.driver.SerialTimeoutException
import jp.oist.abcvlib.core.inputs.microcontroller.BatteryData
import jp.oist.abcvlib.core.inputs.microcontroller.WheelData
import jp.oist.abcvlib.util.HexBinConverters.bytesToHex
import java.io.IOException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.charset.StandardCharsets
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs

/**
 * Class to manage request-response patter between Android phone and USB Serial connection
 * over a separate thread.
 * send()
 * onReceive()
 */
class SerialCommManager @JvmOverloads constructor(
    private val usbSerial: UsbSerial,
    batteryData: BatteryData? = null,
    wheelData: WheelData? = null
) {
    private val androidToRP2040Packet = AndroidToRP2040Packet()
    private val rp2040State: RP2040State?

    private var command: ByteArray? = null
    private var queuedMotorLevelsAtMs: Long = 0L
    private var queuedAtNs: Long = 0L
    private var queuedMotorLevelsTrace: MotorCommandTrace? = null
    private var nextSerialTraceSeq: Long = 1L
    private var inFlightTrace: SerialCommandTrace? = null
    private val commandLock = Object()
    private val lifecycleLock = Any()
    private var runContext: RunContext? = null

    /**
     * How long to wait for the RP2040 to answer a command before giving up on
     * it and sending the next one. The 10 s default suits the stock firmware,
     * which can stall ~1 s after a coast. The fast firmware (RTT-LoopReduction)
     * answers in ~8 ms, and there a single lost reply otherwise freezes the
     * wheels for the full 10 s, so callers on it should set this much lower.
     */
    @Volatile var replyTimeoutMs: Int = 10000

    private var startTimeAndroid: Long = 0
    private var cnt: Int = 0
    private var durationAndroid: Long = 0

    private data class MotorCommandTrace(
        val left: Float,
        val right: Float,
        val leftBrake: Boolean,
        val rightBrake: Boolean,
        val leftControl: Byte,
        val rightControl: Byte
    )

    private data class SerialCommandTrace(
        val seq: Long,
        val command: AndroidToRP2040Command,
        val sentAtMs: Long,
        val motor: MotorCommandTrace? = null
    )

    // Constructor to initialize SerialCommManager
    init {
        if (batteryData == null || wheelData == null) {
            Logger.w(
                "serial", "batteryData or wheelData was null. " +
                        "Ignoring all rp2040 state values. You must initialize both to use rp2040 state"
            )
            rp2040State = null
        } else {
            rp2040State = RP2040State(batteryData, wheelData)
        }
    }


    private class RunContext(
        val stopRequested: AtomicBoolean = AtomicBoolean(false),
        var writerExecutor: ScheduledExecutorServiceWithException? = null
    )

    private fun buildAndroid2PiWriter(context: RunContext): Runnable = Runnable {
        startTimeAndroid = System.nanoTime()
        var queueWaitNs = 0L
        while (!context.stopRequested.get()) {
            val nextCommand: ByteArray? = try {
                synchronized(commandLock) {
                    while (command == null && !context.stopRequested.get()) {
                        commandLock.wait()
                    }
                    if (context.stopRequested.get()) {
                        return@synchronized null
                    }
                    val localCommand = command ?: return@synchronized null
                    val commandType = AndroidToRP2040Command.getEnumByValue(localCommand[1])
                    val traceSeq = nextSerialTraceSeq++
                    inFlightTrace = SerialCommandTrace(
                        seq = traceSeq,
                        command = commandType ?: AndroidToRP2040Command.NACK,
                        sentAtMs = SystemClock.uptimeMillis(),
                        motor = queuedMotorLevelsTrace
                    )
                    command = null
                    queuedMotorLevelsTrace = null
                    val dequeuedNs = System.nanoTime()
                    ControlLatencyTrace.stampDequeued(dequeuedNs)
                    queueWaitNs = if (queuedAtNs > 0L) dequeuedNs - queuedAtNs else 0L
                    queuedAtNs = 0L
                    localCommand
                }
            } catch (e: InterruptedException) {
                continue
            }

            if (nextCommand == null) {
                break
            }

            if (queuedMotorLevelsAtMs > 0L) {
                ControlLatencyTrace.queueToSendMs = SystemClock.uptimeMillis() - queuedMotorLevelsAtMs
                queuedMotorLevelsAtMs = 0L
            }
            logSerialTx(inFlightTrace)
            val isMotor = AndroidToRP2040Command.getEnumByValue(nextCommand[1]) ==
                    AndroidToRP2040Command.SET_MOTOR_LEVELS
            if (ControlLatencyTrace.asyncMotor && isMotor) {
                // Fire and forget, then drain whatever the RP2040 has answered
                // in the meantime. The state packets still land, they are just
                // no longer something a wheel command has to wait behind.
                val t0 = System.nanoTime()
                try {
                    usbSerial.sendOnly(nextCommand)
                } catch (e: IOException) {
                    throw RuntimeException(e)
                }
                val t1 = System.nanoTime()
                parseFifoPacket()
                ControlLatencyTrace.record(
                    queueWaitNs / 1000, 0L, (t1 - t0) / 1000, 0L)
            } else {
                sendPacket(nextCommand, queueWaitNs)
            }
            cnt++
            if (cnt == 100) {
                durationAndroid = (System.nanoTime() - startTimeAndroid) / 100
                cnt = 0
                startTimeAndroid = System.nanoTime()
                // convert from nanoseconds to microseconds
                Logger.e(
                    "AndroidSide",
                    "Average time per command: " + durationAndroid / 1000 + "us"
                )
            }
        }
    }


    // Start method to start the thread
    @JvmOverloads
    fun start(initialDelay: Long = 0, delay: Long = 10) {
        synchronized(lifecycleLock) {
            val existing = runContext
            if (existing != null && !existing.stopRequested.get()) {
                Logger.w("serial", "SerialCommManager already started; ignoring duplicate start()")
                return
            }

            val context = RunContext()
            val priorityFactory = ProcessPriorityThreadFactory(
                Thread.MAX_PRIORITY,
                "SerialCommManager_Android2Pi"
            )
            context.writerExecutor = ScheduledExecutorServiceWithException(1, priorityFactory).also {
                it.scheduleWithFixedDelay(
                    buildAndroid2PiWriter(context),
                    initialDelay,
                    delay,
                    TimeUnit.MILLISECONDS
                )
            }
            runContext = context
        }
    }

    fun stop() {
        var contextToStop: RunContext? = null
        synchronized(lifecycleLock) {
            contextToStop = runContext
            runContext = null
        }
        contextToStop?.let { context ->
            context.stopRequested.set(true)
            context.writerExecutor?.shutdownNow()
            context.writerExecutor = null
        }
    }

    //TODO paseFifoPacket() should call the various SerialResponseListener methods.
    internal fun parseFifoPacket() {
        var result = 0
        val fifoQueuePair: FifoQueuePair?
        var packet: ByteArray?
        // Run the command on a thread handler to allow the queue to keep being added to
        synchronized(usbSerial.fifoQueue) {
            fifoQueuePair = usbSerial.fifoQueue.poll()
        }
        // Check if there is a packet in the queue (fifoQueue
        if (fifoQueuePair != null) {
            packet = fifoQueuePair.byteArray

            // Log packet as an array of hex bytes
            Logger.i(Thread.currentThread().name, "Received packet: ${bytesToHex(packet)}")

            // The first byte after the start mark is the command
            val command = fifoQueuePair.androidToRP2040Command
            val trace = inFlightTrace
            if (trace != null) {
                Logger.i(
                    "BasicAssemblerTrace",
                    "SERIAL_RX seq=${trace.seq} sentType=${trace.command} rxType=$command " +
                            "rttMs=${SystemClock.uptimeMillis() - trace.sentAtMs}"
                )
            } else {
                Logger.w("BasicAssemblerTrace", "SERIAL_RX seq=unknown rxType=$command")
            }
            Logger.i(Thread.currentThread().name, "Received $command from pi")
            if (command == null) {
                inFlightTrace = null
                Logger.e("Pi2AndroidReader", "Command not found")
                return
            }
            when (command) {
                AndroidToRP2040Command.GET_LOG -> {
                    parseLog(packet)
                    result = 1
                }

                AndroidToRP2040Command.SET_MOTOR_LEVELS,
                AndroidToRP2040Command.GET_STATE,
                AndroidToRP2040Command.RESET_STATE -> {
                    parseStatus(packet)
                    logStateResponse(trace, command)
                    result = 1
                }

                AndroidToRP2040Command.NACK -> {
                    onNack(packet)
                    Logger.w("Pi2AndroidReader", "Nack issued from device")
                    result = -1
                }

                AndroidToRP2040Command.ACK -> {
                    onAck(packet)
                    result = 1
                    Logger.d("Pi2AndroidReader", "parseAck")
                }

                AndroidToRP2040Command.START -> {
                    Logger.e("Pi2AndroidReader", "parseStart. Start should never be a command")
                    result = -1
                }

                AndroidToRP2040Command.STOP -> {
                    Logger.e("Pi2AndroidReader", "parseStop. Stop should never be a command")
                    result = -1
                }

                else -> {
                    Logger.e("Pi2AndroidReader", "parsePacket. Command not found")
                    result = -1
                }
            }
            inFlightTrace = null
        } else {
            Logger.i(Thread.currentThread().name, "No packet in queue")
            result = 0
        }
    }


    /**
     * Do not use this method unless you are very familiar with the protocol on both the rp2040 and
     * Android side. This method is used to send raw bytes to the rp2040. It is recommended to use
     * the wrapped methods for doing higher level commands such as setMotorLevels. Sending the wrong
     * bytes to the rp2040 can cause it to crash and require a reset or worse.
     * @param bytes The raw bytes to be sent to the rp2040
     * @return 0 if successful, -1 if mResponse is not large enough to hold all response and the stop mark,
     * -2 if SerialTimeoutException on send
     */
    private fun sendPacket(bytes: ByteArray, queueWaitNs: Long = 0L): Int {
        require(bytes.size == AndroidToRP2040Packet.packetSize) {
            "Input byte array must have a length of " + AndroidToRP2040Packet.packetSize
        }
        // Three stages, timed separately: clearing the parser, the USB bulk
        // write, and waiting for the RP2040 to answer. Their sum is what a
        // controller actually pays to move a wheel, and until it was split
        // there was no way to tell whose fault the ~84ms was.
        val t0 = System.nanoTime()
        usbSerial.prepareForCommand(AndroidToRP2040Command.getEnumByValue(bytes[1]))
        val t1 = System.nanoTime()
        try {
            this.usbSerial.send(bytes, 10000)
        } catch (e: SerialTimeoutException) {
            throw RuntimeException(
                "SerialTimeoutException on send. The serial connection " +
                        "with the rp2040 is not working as expected and timed out"
            )
        } catch (e: IOException) {
            throw RuntimeException(e)
        }
        val t2 = System.nanoTime()
        receivePacket()
        val t3 = System.nanoTime()
        ControlLatencyTrace.record(
            queueWaitNs / 1000, (t1 - t0) / 1000, (t2 - t1) / 1000, (t3 - t2) / 1000)
        return 0
    }

    private fun receivePacket() {
        val receivedStatus = usbSerial.awaitPacketReceived(replyTimeoutMs)
        if (receivedStatus == 1) {
            //Note this is actually calling the functions like parseLog, parseStatus, etc.
            parseFifoPacket()
        } else {
            ControlLatencyTrace.timeoutCount++
            inFlightTrace?.let {
                Logger.e(
                    "BasicAssemblerTrace",
                    "SERIAL_TIMEOUT seq=${it.seq} type=${it.command} elapsedMs=${SystemClock.uptimeMillis() - it.sentAtMs}"
                )
            }
            inFlightTrace = null
        }
    }

    private fun logSerialTx(trace: SerialCommandTrace?) {
        if (trace == null) {
            Logger.w("BasicAssemblerTrace", "SERIAL_TX seq=unknown")
            return
        }
        val motor = trace.motor
        if (motor == null) {
            Logger.i(
                "BasicAssemblerTrace",
                "SERIAL_TX seq=${trace.seq} type=${trace.command}"
            )
        } else {
            Logger.i(
                "BasicAssemblerTrace",
                "SERIAL_TX seq=${trace.seq} type=${trace.command} " +
                        "left=${motor.left} right=${motor.right} leftBrake=${motor.leftBrake} " +
                        "rightBrake=${motor.rightBrake} leftControl=0x${"%02x".format(motor.leftControl.toInt() and 0xFF)} " +
                        "rightControl=0x${"%02x".format(motor.rightControl.toInt() and 0xFF)}"
            )
        }
    }

    private fun logStateResponse(trace: SerialCommandTrace?, command: AndroidToRP2040Command?) {
        val state = rp2040State ?: return
        val seq = trace?.seq?.toString() ?: "unknown"
        val rttMs = trace?.let { SystemClock.uptimeMillis() - it.sentAtMs } ?: -1L
        Logger.i(
            "BasicAssemblerState",
            "SERIAL_STATE seq=$seq rxType=$command rttMs=$rttMs " +
                    "encL=${state.motorsState.encoderCounts.left} " +
                    "encR=${state.motorsState.encoderCounts.right} " +
                    "faultL=0x${"%02x".format(state.motorsState.faults.left.toInt() and 0xFF)} " +
                    "faultR=0x${"%02x".format(state.motorsState.faults.right.toInt() and 0xFF)} " +
                    "wireless=${state.chargeSideUSB.isWirelessChargerAttached()} " +
                    "usbMv=${state.chargeSideUSB.usb_charger_voltage.toInt() and 0xFFFF} " +
                    "vrectMv=${state.chargeSideUSB.wireless_charger_vrect.toInt() and 0xFFFF} " +
                    "batteryMv=${state.batteryDetails.voltageMv.toInt() and 0xFFFF}"
        )
    }

    private fun generateSetMotorLevels(
        androidToRP2040Packet: AndroidToRP2040Packet,
        left: Float,
        right: Float,
        leftBrake: Boolean,
        rightBrake: Boolean
    ): ByteArray {

        androidToRP2040Packet.clear()
        androidToRP2040Packet.setCommand(AndroidToRP2040Command.SET_MOTOR_LEVELS)

        val LOWER_LIMIT = 0.49f

        // Normalize [-1,1] to [-5.06,5.06] as this is the range accepted by the chip
        // Note - signs are to invert the direction of the motors as the motors are mounted in a
        // polar opposite direction.
        val leftNrm = -left * 5.06f
        val rightNrm = -right * 5.06f

        val DRV8830_IN1_BIT: Byte = 0
        val DRV8830_IN2_BIT: Byte = 1

        val voltages = floatArrayOf(leftNrm, rightNrm)
        val abs_voltages = FloatArray(2)
        val control_values = ByteArray(2)
        val brakes = booleanArrayOf(leftBrake, rightBrake)

        for (i in voltages.indices) {
            control_values[i] = 0 // Reset the control value
            // Exclude or truncate voltages between -0.48V and 0.48V to 0V
            // Changing to 0.49 as the scaling function would result in 0x05h for 0.48V and
            // cause the rp2040 to perform unexpectedly as it is a reserved register value
            if (voltages[i] >= -LOWER_LIMIT && voltages[i] <= LOWER_LIMIT) {
                voltages[i] = 0.0f // Update the value in the array
            } else {
                // Clamp the voltage within the valid range
                // Need to clamp here rather than at byte representation to prevent overflow
                if (voltages[i] < -5.06) {
                    voltages[i] = -5.06f // Update the value in the array
                } else if (voltages[i] > 5.06) {
                    voltages[i] = 5.06f // Update the value in the array
                }

                abs_voltages[i] = abs(voltages[i])
                // Convert voltage to control value (-0x3F to 0x3F)
                control_values[i] = (((64 * abs_voltages[i]) / (4 * 1.285)) - 1).toInt().toByte()
                // voltage is defined by bits 2-7. Shift the control value to the correct position
                control_values[i] = (control_values[i].toInt() shl 2).toByte()
            }


            // Set the IN1 and IN2 bits based on the sign of the voltage
            var cv = control_values[i].toInt()
            if (brakes[i]) {
                cv = cv or (1 shl DRV8830_IN1_BIT.toInt())
                cv = cv or (1 shl DRV8830_IN2_BIT.toInt())
            } else {
                if (voltages[i] < 0) {
                    cv = cv or (1 shl DRV8830_IN1_BIT.toInt())
                    cv = cv and (1 shl DRV8830_IN2_BIT.toInt()).inv()
                } else if (voltages[i] > 0) {
                    cv = cv or (1 shl DRV8830_IN2_BIT.toInt())
                    cv = cv and (1 shl DRV8830_IN1_BIT.toInt()).inv()
                } else {
                    // Standby/Coast: Both IN1 and IN2 set to 0
                    cv = 0
                }
            }
            control_values[i] = cv.toByte()
            androidToRP2040Packet.payload.put(control_values[i])
        }
        return androidToRP2040Packet.packetToBytes()
    }

    private fun generateGetLogCmd(): ByteArray {
        androidToRP2040Packet.clear()
        androidToRP2040Packet.setCommand(AndroidToRP2040Command.GET_LOG)
        return androidToRP2040Packet.packetToBytes()
    }

    //-------------------------------------------------------------------///
    // ---- API function calls for requesting something from the mcu ----///
    //-------------------------------------------------------------------///
    //    private void sendAck() throws IOException {
    //        byte[] ack = new byte[]{AndroidToRP2040Command.ACK.getHexValue()};
    //        sendPacket(ack);
    //    }
    /*
    parameters:
    float: left [-1,1] representing full speed backward to full speed forward
    float: right (same as left)
    */
    fun setMotorLevels(left: Float, right: Float, leftBrake: Boolean, rightBrake: Boolean) {
        val activeContext: RunContext
        synchronized(lifecycleLock) {
            activeContext = runContext ?: return
            if (activeContext.stopRequested.get()) {
                return
            }
        }
        synchronized(commandLock) {
            if (activeContext.stopRequested.get()) {
                return
            }
            // One slot, not a queue: if the previous command has not been
            // sent yet it is silently replaced. That is the right policy for
            // a controller (the newest action is the only useful one) but it
            // means the wheels never see most of what the policy chose, so
            // count it rather than letting it stay invisible.
            if (command != null) ControlLatencyTrace.droppedCount++
            command =
                generateSetMotorLevels(androidToRP2040Packet, left, right, leftBrake, rightBrake)
            queuedMotorLevelsTrace = MotorCommandTrace(
                left = left,
                right = right,
                leftBrake = leftBrake,
                rightBrake = rightBrake,
                leftControl = command!![2],
                rightControl = command!![3]
            )
            queuedMotorLevelsAtMs = SystemClock.uptimeMillis()
            queuedAtNs = System.nanoTime()
            ControlLatencyTrace.stampQueued(queuedAtNs)
            ControlLatencyTrace.markQueued()
            commandLock.notify()
        }
    }

    fun getLog() {
        val activeContext: RunContext
        synchronized(lifecycleLock) {
            activeContext = runContext ?: return
            if (activeContext.stopRequested.get()) {
                return
            }
        }
        synchronized(commandLock) {
            if (activeContext.stopRequested.get()) {
                return
            }
            command = generateGetLogCmd()
            ControlLatencyTrace.markQueued()
            queuedMotorLevelsTrace = null
            queuedMotorLevelsAtMs = 0L
            commandLock.notify()
        }
    }

    //----------------------------------------------------------///
    // ---- Handlers for when data is returned from the mcu ----///
    // ---- Override these defaults with your own handlers -----///
    //----------------------------------------------------------///
    private fun parseLog(bytes: ByteArray) {
        Logger.d("serial", "parseLogs")
        val string = String(bytes, StandardCharsets.US_ASCII)
        val lines = string.lines()
        for (line in lines) {
            Logger.i("rp2040Log", line)
        }
    }

    private fun parseStatus(bytes: ByteArray) {
        Logger.d("serial", "parseStatus")
        if (rp2040State != null) {
            val byteBuffer = ByteBuffer.wrap(bytes)
            byteBuffer.order(ByteOrder.LITTLE_ENDIAN)
            val returnedLeftControl = byteBuffer.get()
            val returnedRightControl = byteBuffer.get()
            if (rp2040State.motorsState.controlValues.left != returnedLeftControl) {
                //Logger.e("serial", "Left control value mismatch");
            }
            if (rp2040State.motorsState.controlValues.right != returnedRightControl) {
                //Logger.e("serial", "Right control value mismatch");
            }
            rp2040State.motorsState.faults.left = byteBuffer.get()
            rp2040State.motorsState.faults.right = byteBuffer.get()
            Logger.v("serial", "left motor fault: " + rp2040State.motorsState.faults.left)
            Logger.v("serial", "right motor fault: " + rp2040State.motorsState.faults.right)
            rp2040State.motorsState.encoderCounts.left = byteBuffer.getInt()
            rp2040State.motorsState.encoderCounts.right = byteBuffer.getInt()
            Logger.v("serial", "Left encoder count: " + rp2040State.motorsState.encoderCounts.left)
            Logger.v(
                "serial",
                "Right encoder count: " + rp2040State.motorsState.encoderCounts.right
            )
            rp2040State.batteryDetails.voltageMv = byteBuffer.getShort()
            rp2040State.batteryDetails.safetyStatus = byteBuffer.get()
            ControlLatencyTrace.recordState(
                rp2040State.motorsState.faults.left,
                rp2040State.motorsState.faults.right,
                rp2040State.batteryDetails.safetyStatus)
            rp2040State.batteryDetails.temperature = byteBuffer.getShort()
            rp2040State.batteryDetails.stateOfHealth = byteBuffer.get()
            rp2040State.batteryDetails.flags = byteBuffer.getShort()
            Logger.v("serial", "Battery voltage: " + rp2040State.batteryDetails.voltageMv)
            Logger.v("serial", "Battery voltage in V: " + rp2040State.batteryDetails.getVoltage())
            rp2040State.chargeSideUSB.max77976_chg_details = byteBuffer.getInt()
            rp2040State.chargeSideUSB.ncp3901_wireless_charger_attached =
                byteBuffer.get().toInt() == 1
            Logger.v(
                "serial",
                "ncp3901_wireless_charger_attached: " + rp2040State.chargeSideUSB.isWirelessChargerAttached()
            )
            rp2040State.chargeSideUSB.usb_charger_voltage = byteBuffer.getShort()
            rp2040State.chargeSideUSB.wireless_charger_vrect = byteBuffer.getShort()
            // Logger.v("serial", "usb_charger_voltage: " + rp2040State.chargeSideUSB.usb_charger_voltage);
            rp2040State.updatePublishers()
        }
    }

    private fun onNack(bytes: ByteArray) {
        Logger.d("serial", "parseNack")
    }

    private fun onAck(bytes: ByteArray) {
        Logger.d("serial", "parseAck")
    }
}
