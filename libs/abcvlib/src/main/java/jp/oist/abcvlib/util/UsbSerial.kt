package jp.oist.abcvlib.util

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbManager
import android.os.Build
import androidx.core.content.ContextCompat
import androidx.core.content.ContextCompat.RECEIVER_NOT_EXPORTED
import androidx.core.content.IntentCompat
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver
import com.hoho.android.usbserial.driver.ProbeTable
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import jp.oist.abcvlib.util.AndroidToRP2040Command.Companion.getEnumByValue
import jp.oist.abcvlib.util.ErrorHandler.eLog
import org.apache.commons.collections4.queue.CircularFifoQueue
import java.io.IOException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.TimeUnit
import java.util.concurrent.locks.Condition
import java.util.concurrent.locks.ReentrantLock

class UsbSerial @Throws(IOException::class) constructor(
    private val context: Context,
    private val usbManager: UsbManager,
    private val serialReadyListener: SerialReadyListener
) : SerialInputOutputManager.Listener {
    private lateinit var port: UsbSerialPort

    // Initialize as the smallest of the two packet sizes
    private var packetDataSize = RP2020_PACKET_SIZE_STATE
    private var packetType: AndroidToRP2040Command = AndroidToRP2040Command.NACK
    private var expectedPacketType: AndroidToRP2040Command? = null
    private val timeout: Int = 1000 //1s
    private val totalBytesRead: Int = 0 // Track total bytes read
    private val pwm = floatArrayOf(1.0f, 0.5f, 0.0f, -0.5f, -1.0f)
    private val cnt = 0
    private var badPacketCount = 0

    internal val fifoQueue: CircularFifoQueue<FifoQueuePair> = CircularFifoQueue<FifoQueuePair>(256)
    private val packetBuffer = ByteBuffer.allocate((512 * 128) + 8) // Adjust size as needed

    private class StartStopIndex(var startIdx: Int, var stopIdx: Int)

    //Ensure a proper start and stop mark present before adding anything to the fifoQueue
    private var startStopIdx: StartStopIndex

    companion object {
        private const val TAG = "UsbSerial"

        private val lock = ReentrantLock()
        private val packetReceived: Condition = lock.newCondition()

        // 1024 + 3 for start, command, and stop markers sent as putchar. (i.e. they won't be optimized anyway).
        private const val RP2020_PACKET_SIZE_LOG = 1027
        private const val RP2020_PACKET_SIZE_STATE = 64
        private const val ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION"
        private const val BAD_PACKET_THRESHOLD = 5

    }

    init {
        // Find all available drivers from attached devices.
        val deviceList = usbManager.deviceList
        startStopIdx = StartStopIndex(-1, -1)
        packetBuffer.order(ByteOrder.LITTLE_ENDIAN)

        if (deviceList.isEmpty()) {
            throw IOException("No USB devices found")
        }

        for (d in deviceList.values) {
            if (d.manufacturerName == "Seeed" && d.productName == "Seeeduino XIAO") {
                Logger.i(Thread.currentThread().name, "Found a XIAO. Connecting...")
                connect(d)
            } else if (d.manufacturerName.equals("Raspberry Pi") && d.productName.equals("Pico Test Device")) {
                Logger.i(Thread.currentThread().name, "Found a Pico Test Device. Connecting...")
                connect(d)
            } else if (d.manufacturerName == "Raspberry Pi" && d.productName == "Pico") {
                Logger.i(Thread.currentThread().name, "Found a Pi. Connecting...")
                connect(d)
            }
        }
        val filter = IntentFilter(ACTION_USB_PERMISSION).apply {
            addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
        }
        val usbReceiver: BroadcastReceiver = MyBroadcastReceiver()
        ContextCompat.registerReceiver(context, usbReceiver, filter, RECEIVER_NOT_EXPORTED)
    }

    @Throws(IOException::class)
    private fun connect(device: UsbDevice) {
        if (usbManager.hasPermission(device)) {
            Logger.i(Thread.currentThread().name, "Has permission to connect to device")
            val connection: UsbDeviceConnection = usbManager.openDevice(device)
            openPort(connection)
        } else {
            Logger.i(Thread.currentThread().name, "Requesting permission to connect to device")
            // Make the permission intent mutable for Android 12 and above
            val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                PendingIntent.FLAG_MUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
            } else {
                0
            }
            val usbPermissionIntent: Intent = Intent(ACTION_USB_PERMISSION).apply {
                setPackage(context.packageName)
            }
            val permissionIntent =
                PendingIntent.getBroadcast(context, 0, usbPermissionIntent, flags)

            usbManager.requestPermission(device, permissionIntent)
        }
    }

    private fun openPort(connection: UsbDeviceConnection) {
        Logger.i(Thread.currentThread().name, "Opening port")
        val driver = getDriver()
        val port = driver.ports[0] // Most devices have just one port (port 0)
        try {
            port.open(connection)
            port.setParameters(115200, 8, 1, UsbSerialPort.PARITY_NONE)
            port.dtr = true
            this.port = port
            val usbIoManager = SerialInputOutputManager(port, this)
            // Do NOT bound the reader's timeout. It looks like it should help
            // -- the reader holds the USB connection inside bulkTransfer, so
            // in principle a write waits behind it -- but measured
            // 2026-09-10 it does not: writes took 0.86ms with the reader
            // parked, so there is no contention to fix. What a 20ms read
            // timeout *did* do was lose reply bytes between transfers, so
            // packets stopped assembling, awaitPacketReceived hit its 10s
            // ceiling, and the effective command rate fell from 12Hz to 2.3Hz.
            // Adding this as there doesn't appear to be any call back in the usbIoManager that
            // will call onSerialReady after initialization. As it stands, there were things occurring
            // in onSerialReady that were being executed before the usbIoManager was initialized.
            Thread.sleep(100)
            usbIoManager.start()
            serialReadyListener.onSerialReady(this)
        } catch (e: IOException) {
            e.printStackTrace()
        } catch (e: InterruptedException) {
            throw RuntimeException(e)
        }
    }

    private fun getDriver(): UsbSerialDriver {
        val customTable = ProbeTable().apply {
            addProduct(0x2886, 0x802F, CdcAcmSerialDriver::class.java) // Seeeduino XIAO
            addProduct(11914, 10, CdcAcmSerialDriver::class.java) // Raspberry Pi Pico
            addProduct(0x0000, 0x0001, CdcAcmSerialDriver::class.java) // Custom Raspberry Pi Pico
        }
        val prober = UsbSerialProber(customTable)
        val availableDrivers = prober.findAllDrivers(usbManager)
        if (availableDrivers.isEmpty()) {
            eLog("Serial", "No USB Serial drivers found", Exception(), true)
        }
        return availableDrivers[0]
    }

    override fun onNewData(data: ByteArray) {
        ControlLatencyTrace.markData()
        // TODO I feel this should be executed in a separate thread otherwise the
        // SerialInputOutputManager thread may be delayed and miss data

        // print the byte[] as an array of hex values

        val sb = StringBuilder()
        for (b in data) {
            sb.append(String.format("%02X ", b))
        }
        Logger.d(TAG, "onNewData Received: $sb")

        // Run the packet verification in a separate thread
        try {
            lock.lock()
            if (verifyPacket(data)) {
                Logger.d(TAG, "Packet verified")
                Logger.d(TAG, "packetReceived.signal()")
                packetReceived.signal()
            } else {
                Logger.d(TAG, "Incomplete Packet. Waiting for more data")
            }
        } catch (e: IOException) {
            throw RuntimeException(e)
        } finally {
            lock.unlock()
        }
    }

    @Throws(IOException::class)
    internal fun send(packet: ByteArray, timeout: Int) {
        port.write(packet, timeout)
        ControlLatencyTrace.markSend()
        Logger.i(Thread.currentThread().name, "send() bytes=${HexBinConverters.bytesToHex(packet)}")
    }

    /**
     * Write without clearing the parser or waiting for a reply.
     *
     * The wheels do not need the reply: it carries encoder counts and battery
     * voltage, which are telemetry. Keeping the parser running instead of
     * resetting it before every command is what makes that safe -- replies
     * still arrive and are still parsed, just not waited for.
     */
    @Throws(IOException::class)
    internal fun sendOnly(packet: ByteArray) {
        port.write(packet, 200)
        ControlLatencyTrace.markSend()
    }

    internal fun prepareForCommand(expectedCommand: AndroidToRP2040Command?) {
        lock.lock()
        try {
            synchronized(fifoQueue) {
                fifoQueue.clear()
            }
            expectedPacketType = expectedCommand
            resetParserState()
            packetBuffer.clear()
        } finally {
            lock.unlock()
        }
    }

    /**
     * Blocks until a response is received
     */
    internal fun awaitPacketReceived(timeout: Int): Int {
        var returnVal = -1
        // Wait until packet is available
        lock.lock()
        try {
            if (!packetReceived.await(timeout.toLong(), TimeUnit.MILLISECONDS)) {
                // throw new RuntimeException("SerialTimeoutException on send. The serial connection " +
                // "with the rp2040 is not working as expected and timed out");
            } else {
                Logger.d(
                    Thread.currentThread().name,
                    "packetReceived.await() completed. Packet received from rp2040"
                )
                returnVal = 1
            }
        } catch (e: InterruptedException) {
            e.printStackTrace()
        } finally {
            lock.unlock()
        }
        return returnVal
    }

    @Throws(IOException::class)
    private fun startStopIndexSearch(data: ByteBuffer): StartStopIndex {
        val startStopIdx = StartStopIndex(-1, -1)
        var i = 0
        data.flip()
        while (i < data.limit()) {
            // get value at position and increment position by 1 (so don't call it multiple times)
            val value = data.get()

            // Always overwrite the value of stopIdx as you want the last instance
            if (value == AndroidToRP2040Command.STOP.hexValue) {
                startStopIdx.stopIdx = i
                Logger.v("serial", "stopIdx found at $i")
            } else if (value == AndroidToRP2040Command.START.hexValue && startStopIdx.startIdx < 0) {
                startStopIdx.startIdx = i
                Logger.v("serial", "startIdx found at $i")
            }
            i++
        }
        // flip will set limit to position and position to 0
        // need to set limit back to capacity after reading
        data.limit(data.capacity())

        return startStopIdx
    }

    /**
     * synchronized as newData might be called again while this method is running
     * @param bytes to be verified to contain a valid packet. If so, the packet is added to the fifoQueue
     * @return 0 if successfully found a complete packet or found an erroneous packet,
     * -1 if not enough data to determine
     * @throws IOException if fifoQueue is full
     */
    @Synchronized
    @Throws(IOException::class)
    internal fun verifyPacket(bytes: ByteArray): Boolean {
        if (packetBuffer.remaining() < bytes.size) {
            Logger.e("verifyPacket", "Buffer overflow risk: clearing buffer and resynchronizing.")
            packetBuffer.clear()
            onBadPacket()
            return false
        }
        packetBuffer.put(bytes)

        // If startIdx not yet found, find it
        if (startStopIdx.startIdx < 0) {
            startStopIdx = startStopIndexSearch(packetBuffer)
        }

        // If startIdx still not found, return false
        if (startStopIdx.startIdx < 0) {
            Logger.v("serial", "StartIdx not yet found. Waiting for more data")
            return false
        } else {
            Logger.v("serial", "StartIdx found at " + startStopIdx.startIdx)
            // if packetType not yet found, find it
            if (packetType == AndroidToRP2040Command.NACK) {
                // If position is 0, nothing received yet. If position is 1 only the start mark has been received.
                // position 2 is the packetType, and positions 3 and 4 are a short for packetSize
                if (packetBuffer.position() >= RP2040ToAndroidPacket.Offsets.DATA) {
                    try {
                        val parsedPacketType = getEnumByValue(
                            packetBuffer.get(startStopIdx.startIdx + RP2040ToAndroidPacket.Offsets.PACKET_TYPE)
                        )
                        if (parsedPacketType == null) {
                            Logger.e("serial", "Unknown packetType byte")
                            onBadPacket()
                            return false
                        }
                        packetType = parsedPacketType
                    } catch (e: IndexOutOfBoundsException) {
                        Logger.e("serial", "IndexOutOfBoundsException: " + e.message)
                        Logger.e("serial", "Unknown packetType: $packetType")
                        // Ignore this packet as it is corrupted by some other data being sent between.
                        onBadPacket()
                        return false
                    }
                    if (expectedPacketType != null && packetType != expectedPacketType) {
                        Logger.e(
                            "verifyPacket",
                            "Unexpected packetType: $packetType while waiting for $expectedPacketType. Resynchronizing."
                        )
                        onBadPacket()
                        return false
                    }
                    // get the packetDataSize. It is stored at index 3 and 4 as a short
                    packetDataSize = packetBuffer.getShort(
                        startStopIdx.startIdx + RP2040ToAndroidPacket.Offsets.DATA_SIZE
                    ).toInt() and 0xFFFF
                    if (!isReasonablePacketSize(packetType, packetDataSize)) {
                        Logger.e(
                            "verifyPacket",
                            "Unreasonable packet size: $packetDataSize. Resynchronizing."
                        )
                        onBadPacket()
                        return false
                    }
                    Logger.v(
                        "serial",
                        packetType.toString() + " packetType of size " + packetDataSize + " found at "
                                + (startStopIdx.startIdx + RP2040ToAndroidPacket.Offsets.PACKET_TYPE)
                    )
                } else {
                    Logger.v("serial", "Nothing other than startIdx found. Waiting for more data")
                    return false
                }
            }

            // Check if you have enough data for a full packet before processing anything
            // +1 for moving 1 past packetBuffer.position() to endIdx byte
            startStopIdx.stopIdx =
                startStopIdx.startIdx + packetDataSize + (RP2040ToAndroidPacket.Offsets.DATA)
            // +1 for packetBuffer.position() needing to be 1 after endIdx byte to indicate that you have the stopIdx byte
            if (packetBuffer.position() < (startStopIdx.stopIdx + 1)) {
                Logger.d(
                    "verifyPacket", "Data received not yet enough to fill " +
                            packetType + " packetType. Waiting for more data"
                )
                Logger.d(
                    "verifyPacket",
                    packetBuffer.position()
                        .toString() + " bytes recvd thus far. Require " + startStopIdx.stopIdx
                )
                return false
            } else {
                if (packetBuffer.get(startStopIdx.stopIdx) != AndroidToRP2040Command.STOP.hexValue) {
                    onBadPacket()
                    return false
                } else {
                    when (packetType) {
                        AndroidToRP2040Command.GET_LOG -> {
                            Logger.i("verifyPacket", "GET_LOG command received")
                        }

                        AndroidToRP2040Command.SET_MOTOR_LEVELS -> {
                            Logger.i("verifyPacket", "SET_MOTOR_LEVELS command received")
                        }

                        AndroidToRP2040Command.GET_STATE -> {
                            Logger.i("verifyPacket", "GET_STATE command received")
                        }

                        else -> {
                            Logger.e("verifyPacket", "Unknown packetType: $packetType")
                            onBadPacket()
                            return false
                        }
                    }
                    onCompletePacketReceived()
                    return true
                }
            }
        }
    }

    private fun onCompletePacketReceived() {
        packetBuffer.position(startStopIdx.startIdx + RP2040ToAndroidPacket.Offsets.DATA)
        packetBuffer.limit(startStopIdx.stopIdx)
        val partialArray = ByteArray(packetBuffer.remaining())
        packetBuffer.get(partialArray)
        packetBuffer.clear()
        synchronized(fifoQueue) {
            if (fifoQueue.isAtFullCapacity) {
                Logger.e("serial", "fifoQueue is full")
                throw RuntimeException("fifoQueue is full")
            } else {
                // Log partialArray as array of hex values
                val sb = StringBuilder()
                for (b in partialArray) {
                    sb.append(String.format("%02X ", b))
                }
                Logger.d("verifyPacket", "Adding Packet: $sb to fifoQueue")
                val fifoQueuePair = FifoQueuePair(packetType, partialArray)
                fifoQueue.add(fifoQueuePair) // Add the partialArray to the queue
            }
        }
        // reset to default value;
        resetParserState()
        expectedPacketType = null
        badPacketCount = 0 // Reset on successful packet
    }

    private fun onBadPacket() {
        Logger.e("serial", "Bad packet received. Clearing buffer and sending next command.")
        resyncAfterBadPacket()
        resetParserState()
        badPacketCount++
        if (badPacketCount >= BAD_PACKET_THRESHOLD) {
            Logger.e(
                "serial",
                "Too many consecutive bad packets. Consider resetting connection or notifying user."
            )
            // Optionally: trigger recovery, e.g., re-initialize serial connection or notify user
            // recoverSerialConnection();
            // For now, just reset the counter
            badPacketCount = 0
        }
    }

    private fun resetParserState() {
        startStopIdx = StartStopIndex(-1, -1)
        packetType = AndroidToRP2040Command.NACK
        packetDataSize = RP2020_PACKET_SIZE_STATE
    }

    private fun isReasonablePacketSize(
        packetType: AndroidToRP2040Command,
        packetDataSize: Int
    ): Boolean {
        return when (packetType) {
            AndroidToRP2040Command.GET_LOG -> packetDataSize in 0..RP2020_PACKET_SIZE_LOG
            AndroidToRP2040Command.SET_MOTOR_LEVELS,
            AndroidToRP2040Command.GET_STATE,
            AndroidToRP2040Command.RESET_STATE,
            AndroidToRP2040Command.NACK,
            AndroidToRP2040Command.ACK -> packetDataSize in 0..RP2020_PACKET_SIZE_STATE
            AndroidToRP2040Command.START,
            AndroidToRP2040Command.STOP -> false
        }
    }

    private fun resyncAfterBadPacket() {
        val writePosition = packetBuffer.position()
        var nextStartIdx = -1
        for (i in (startStopIdx.startIdx + 1).coerceAtLeast(0)..<writePosition) {
            if (packetBuffer.get(i) == AndroidToRP2040Command.START.hexValue) {
                nextStartIdx = i
                break
            }
        }

        if (nextStartIdx >= 0) {
            packetBuffer.position(nextStartIdx)
            packetBuffer.limit(writePosition)
            packetBuffer.compact()
        } else {
            packetBuffer.clear()
        }
    }

    override fun onRunError(e: Exception) {
        Logger.e("serial", "error: " + e.localizedMessage)
        e.printStackTrace()
    }

    private inner class MyBroadcastReceiver : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            val device = IntentCompat.getParcelableExtra(
                intent,
                UsbManager.EXTRA_DEVICE,
                UsbDevice::class.java
            )
            if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                val action = intent.action
                if (ACTION_USB_PERMISSION == action) {
                    synchronized(this) {
                        try {
                            connect(device!!)
                        } catch (e: IOException) {
                            e.printStackTrace()
                        }
                    }
                }
            } else {
                Logger.d("serial", "permission denied for device $device")
            }
        }
    }


}
