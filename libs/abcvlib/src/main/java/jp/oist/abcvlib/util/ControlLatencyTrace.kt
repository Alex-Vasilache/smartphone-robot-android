package jp.oist.abcvlib.util

/**
 * Where the time between "a controller chose an action" and "the motors were
 * told about it" actually goes.
 *
 * Every timing number the robot reported before this existed stopped at the
 * moment `setWheelOutput` returned -- which only drops a byte array into a
 * one-slot mailbox and wakes the serial writer. Measured 2026-09-10, the
 * writer needs ~84ms per command, so the wheels were being told about actions
 * long after the tick that chose them, and most commands were overwritten in
 * the mailbox before they were ever sent. None of that was visible.
 *
 * All fields are plain volatiles read from Python on the control thread; the
 * writer thread is the only writer. Times are microseconds unless named ms.
 */
object ControlLatencyTrace {
    @Volatile
    var requestedLeft: Float = 0f

    @Volatile
    var requestedRight: Float = 0f

    /** After the `maxChange` slew limit in Outputs -- what the wheels got. */
    @Volatile
    var sentLeft: Float = 0f

    @Volatile
    var sentRight: Float = 0f

    @Volatile
    var outputsDtMs: Long = 0L

    @Volatile
    var queueToSendMs: Long = 0L

    // -- the split of one command's service time -----------------------------

    /** Mailbox wait: setMotorLevels -> the writer picked it up. */
    @Volatile
    var queueUs: Long = 0L

    /** usbSerial.prepareForCommand: clearing the parser and FIFO. */
    @Volatile
    var prepUs: Long = 0L

    /** usbSerial.send: the USB bulk write itself. */
    @Volatile
    var writeUs: Long = 0L

    /** awaitPacketReceived: waiting for the RP2040 to answer. */
    @Volatile
    var respUs: Long = 0L

    /** prepare + write + response, i.e. one full round trip. */
    @Volatile
    var serviceUs: Long = 0L

    // -- counters ------------------------------------------------------------

    /** Commands handed to the serial writer. */
    @Volatile
    var sentCount: Long = 0L

    /**
     * Commands that were overwritten in the one-slot mailbox before the writer
     * could send them. This is the count of actions the wheels never saw.
     */
    @Volatile
    var droppedCount: Long = 0L

    /**
     * Send motor commands without blocking for the RP2040's reply.
     *
     * Measured 2026-09-10: a SET_MOTOR_LEVELS round trip costs 84ms, of which
     * the USB write is 1ms and the rest is waiting for a status packet that
     * only carries telemetry. Throttling the command rate to 3Hz did not
     * shorten it, so it is a fixed cost inside the microcontroller rather than
     * a cadence we are catching. Blocking on it caps the wheels at 12Hz and
     * silently discards 60% of a 50Hz policy's actions in the one-slot mailbox.
     */
    @Volatile
    var asyncMotor: Boolean = false

    @JvmStatic
    fun useAsyncMotor(enabled: Boolean) {
        asyncMotor = enabled
    }

    /** Round trips that gave up in awaitPacketReceived without a reply. */
    @Volatile
    var timeoutCount: Long = 0L

    // -- inbound: is the RP2040 slow, or is our parser late? ------------------

    /** send() -> the first byte of the reply arriving from the wire. */
    @Volatile var firstDataUs: Long = 0L

    /** Chunks delivered by the USB reader between one send and its reply. */
    @Volatile var dataChunks: Long = 0L

    /** Gap between consecutive inbound chunks, whether or not we asked. */
    @Volatile var dataGapUs: Long = 0L

    private var lastSendNs = 0L
    private var lastDataNs = 0L
    private var chunks = 0L
    private var accFirstData = 0L
    private var nFirstData = 0

    /** Called by UsbSerial the instant the write returns. */
    @JvmStatic
    @Synchronized
    fun markSend() {
        lastSendNs = System.nanoTime()
        chunks = 0
    }

    /** Called by UsbSerial for every chunk the USB reader hands us. */
    @JvmStatic
    @Synchronized
    fun markData() {
        val now = System.nanoTime()
        if (lastDataNs > 0L) dataGapUs = (now - lastDataNs) / 1000
        lastDataNs = now
        chunks++
        if (chunks == 1L && lastSendNs > 0L) {
            firstDataUs = (now - lastSendNs) / 1000
            accFirstData += firstDataUs
            nFirstData++
        }
        dataChunks = chunks
    }

    // -- what the RP2040 said about the motor driver in its last reply ---------

    /** DRV8830 FAULT register, left / right, from the last state reply. */
    @Volatile var faultL: Int = 0
    @Volatile var faultR: Int = 0

    /** Replies whose fault register was non-zero, either side. */
    @Volatile var faultCount: Long = 0L

    /** Battery pack safety status byte from the last state reply. */
    @Volatile var battSafety: Int = 0

    @JvmStatic
    @Synchronized
    fun recordState(faultLeft: Byte, faultRight: Byte, safety: Byte) {
        faultL = faultLeft.toInt() and 0xFF
        faultR = faultRight.toInt() and 0xFF
        battSafety = safety.toInt() and 0xFF
        if (faultL != 0 || faultR != 0) faultCount++
    }

    private var accService = 0L
    private var accResp = 0L
    private var accWrite = 0L
    private var accPrep = 0L
    private var accQueue = 0L
    private var maxService = 0L
    private var n = 0

    /**
     * Everything above as one JSON object.
     *
     * Kotlin `var`s in an `object` are instance properties behind static
     * accessors, and Chaquopy cannot read them as Python attributes -- reading
     * one throws `has no attribute`, from the control loop, mid-run. A single
     * @JvmStatic method is the one shape that is reliably callable, so the
     * whole trace goes through here rather than field by field.
     */
    @JvmStatic
    @Synchronized
    fun snapshotJson(): String = (
        "{\"ser_queue_ms\":" + queueUs / 1e3 +
        ",\"ser_prep_ms\":" + prepUs / 1e3 +
        ",\"ser_write_ms\":" + writeUs / 1e3 +
        ",\"ser_resp_ms\":" + respUs / 1e3 +
        ",\"ser_service_ms\":" + serviceUs / 1e3 +
        ",\"ser_first_byte_ms\":" + firstDataUs / 1e3 +
        ",\"ser_chunks\":" + dataChunks +
        ",\"ser_gap_ms\":" + dataGapUs / 1e3 +
        ",\"ser_sent\":" + sentCount +
        ",\"ser_dropped\":" + droppedCount +
        ",\"ser_timeouts\":" + timeoutCount +
        ",\"ser_async\":" + (if (asyncMotor) 1 else 0) +
        ",\"t_queued\":" + lastQueuedNs / 1e9 +
        ",\"t_dequeued\":" + lastDequeuedNs / 1e9 +
        ",\"t_reply\":" + lastReplyNs / 1e9 +
        ",\"fault_l\":" + faultL +
        ",\"fault_r\":" + faultR +
        ",\"fault_count\":" + faultCount +
        ",\"batt_safety\":" + battSafety +
        ",\"req_l\":" + requestedLeft +
        ",\"req_r\":" + requestedRight +
        ",\"sent_l\":" + sentLeft +
        ",\"sent_r\":" + sentRight + "}")

    // -- is the RP2040 free right now? -----------------------------------------

    // Two flags, not one. "Idle" is mailbox empty AND nothing in flight; a
    // single pending flag cleared by the reply got it wrong whenever a command
    // was queued while the previous one was still out: the *previous* reply
    // cleared it with ours still waiting, the loop saw idle, and every command
    // ran one round trip behind -- measured 2026-09-14 as a 50 ms mailbox wait
    // on a link that was supposedly free.
    @Volatile private var mailboxFull = false
    @Volatile private var inFlight = false
    @Volatile private var idleSinceNs = 0L

    /** Called by setMotorLevels/getLog when a command enters the mailbox. */
    @JvmStatic
    fun markQueued() {
        mailboxFull = true
    }

    /**
     * Microseconds since the writer finished its last round trip, or -1 while
     * a command is queued or in flight.
     *
     * The microcontroller reads USB only between commands and is busy for
     * ~83 ms after each one, so the only moment a new command is applied
     * promptly is right after its reply lands. A control loop that paces
     * itself on this instead of on a wall clock gets every action applied
     * and applied fresh; one paced on a 50 Hz clock gets 12 Hz with the rest
     * overwritten in the mailbox at random.
     */
    @JvmStatic
    fun serialIdleUs(): Long {
        if (mailboxFull || inFlight) return -1L
        val since = idleSinceNs
        if (since == 0L) return Long.MAX_VALUE / 2
        return (System.nanoTime() - since) / 1000
    }

    /** Called once per command by the serial writer. */
    /** Absolute nanoTime stamps of the last command, for lining up with the
     *  control loop's own clock (Python's time.monotonic is the same clock). */
    @Volatile var lastQueuedNs: Long = 0L
    @Volatile var lastDequeuedNs: Long = 0L
    @Volatile var lastReplyNs: Long = 0L

    @JvmStatic
    fun stampQueued(ns: Long) { lastQueuedNs = ns }

    @JvmStatic
    fun stampDequeued(ns: Long) {
        lastDequeuedNs = ns
        mailboxFull = false
        inFlight = true
    }

    @JvmStatic
    @Synchronized
    fun record(queueUs: Long, prepUs: Long, writeUs: Long, respUs: Long) {
        lastReplyNs = System.nanoTime()
        this.queueUs = queueUs
        this.prepUs = prepUs
        this.writeUs = writeUs
        this.respUs = respUs
        this.serviceUs = prepUs + writeUs + respUs
        sentCount++
        idleSinceNs = System.nanoTime()
        inFlight = false
        accQueue += queueUs
        accPrep += prepUs
        accWrite += writeUs
        accResp += respUs
        accService += serviceUs
        if (serviceUs > maxService) maxService = serviceUs
        n++
        if (n >= 100) {
            // Logger.e, not .i: the i/d/v levels are compiled out unless the
            // library itself is a debug build, and this is the number that
            // decides whether the robot can react at all.
            Logger.e(
                "SerialLatency",
                "n=$n queue=${accQueue / n / 1000}ms prep=${accPrep / n / 1000}ms " +
                        "write=${accWrite / n / 1000}ms resp=${accResp / n / 1000}ms " +
                        "service=${accService / n / 1000}ms max=${maxService / 1000}ms " +
                        "firstByte=${if (nFirstData > 0) accFirstData / nFirstData / 1000 else -1}ms " +
                        "chunks=$dataChunks gap=${dataGapUs / 1000}ms " +
                        "sent=$sentCount dropped=$droppedCount timeouts=$timeoutCount"
            )
            accQueue = 0; accPrep = 0; accWrite = 0; accResp = 0; accService = 0
            accFirstData = 0; nFirstData = 0
            maxService = 0
            n = 0
        }
    }
}
