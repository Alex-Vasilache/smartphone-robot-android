"""Entry point abcvlib calls once the USB serial link is up.

`run()` is invoked from `MainActivity.onSerialReady`, which fires on *every*
USB attach -- the first one, and again whenever the RP2040 re-enumerates
(cable bumped, or the microcontroller browning out under motor load and
rebooting). abcvlib cancels the previous coroutine before calling it again,
but a coroutine blocked inside Python cannot be cancelled, so the first
`loop()` keeps running and a second one starts beside it.

Measured 2026-09-14, after two such re-entries: three sensor threads and two
serial writers in one process, the control loop reporting 50 ms of work per
20 ms tick, `step` going backwards, and the policy acting on IMU samples that
were **28 seconds** old. Nothing logged an error. So `run()` is guarded: the
publishers, the serial manager and the loop are built once per process, and a
repeat call only lets the framework restart what it owns.
"""

import threading
import time

import main

loop_delay = 0.0
context = None

_lock = threading.Lock()
_started = False


def run():
    global _started
    main.context = context
    with _lock:
        first = not _started
        _started = True
    if not first:
        print('dreamerBridge: serial link came back; keeping the existing '
              'control loop and publishers rather than starting a second set')
        # The base class still needs to restart the serial writer and rebuild
        # the outputs for the re-opened port. This is the only part of the
        # original setup() that must run again.
        context.onSetupReady()
        return
    main.setup()
    while True:
        main.loop()
        time.sleep(loop_delay)
