"""Phone side of the DreamerV3 bridge.

The policy runs on the training computer, not here. Each control step this
client receives one action, applies it to the wheels, waits out the control
period, then reports the proprio sensors it collected. The phone therefore owns
the control clock and the trainer blocks on us, which keeps the two ends in
lockstep and means an action is never applied twice.

The trainer is `embodied/envs/robot.py` in the dreamerv3 repo, and
`tools/fake_robot_phone.py` there is the same protocol without a robot, useful
for testing the computer side on its own.

Wire format, both directions: a 4-byte big-endian header length, a UTF-8 JSON
header, then `blob_len` bytes of binary payload. The blob is unused while the
observation is proprio only; it is where camera frames go.

Set the trainer address in `config.json` at the repository root (copy
`config.template.json`); it is compiled into `BuildConfig.IP`/`PORT`.
"""

import json
import math
import socket
import struct
import time

from java import dynamic_proxy
from jp.oist.abcvlib.core import BuildConfig
from jp.oist.abcvlib.core.inputs import PublisherManager
from jp.oist.abcvlib.core.inputs.microcontroller import (
    BatteryData, WheelData, BatteryDataSubscriber, WheelDataSubscriber)
from jp.oist.abcvlib.core.inputs.phone import (
    OrientationData, OrientationDataSubscriber)
from jp.oist.abcvlib.util import SerialCommManager

PROTOCOL = 1
CONTROL_HZ = 50.0
# The GUI is for a human watching the robot, so it does not need to keep up
# with the control loop. Above ~10 Hz the runOnUiThread hop starts costing
# more than the control step itself and shows up as jitter at the trainer.
GUI_PERIOD = 0.1
RECV_TIMEOUT = 30.0
# Connecting must fail fast. The control loop is single threaded, so a blocking
# connect freezes sensing and the display with it -- which looks exactly like a
# stale sensor. A refused connection returns immediately; this only bounds the
# case where the trainer's host swallows the SYN.
CONNECT_TIMEOUT = 0.5
RETRY_DELAY = 2.0
# How often the idle loop refreshes the display while waiting for a trainer.
IDLE_PERIOD = 0.05
ROBOT_ID = 1

context = None  # Activity context, injected by abcvlib.py.

# Latest value from each subscriber callback. Callbacks fire on publisher
# threads and the control loop reads them; every one is a single assignment of
# an immutable float, so the GIL is enough and no lock is needed.
sensors = dict(
    wheel_speed_l=0.0, wheel_speed_r=0.0,
    wheel_distance_l=0.0, wheel_distance_r=0.0,
    wheel_count_l=0.0, wheel_count_r=0.0,
    theta=0.0, angular_velocity=0.0,
    battery_voltage=0.0, charger_voltage=0.0, coil_voltage=0.0,
)

sock = None
stream = None
step = 0
next_tick = 0.0
last_attempt = 0.0
last_report = 0.0
last_gui = 0.0
control_hz = 0.0


class WheelSubscriber(dynamic_proxy(WheelDataSubscriber)):

    def onWheelDataUpdate(self, timestamp, wheel_count_l, wheel_count_r,
                          wheel_distance_l, wheel_distance_r,
                          wheel_speed_instant_l, wheel_speed_instant_r,
                          wheel_speed_buffered_l, wheel_speed_buffered_r,
                          wheel_speed_exp_avg_l, wheel_speed_exp_avg_r):
        sensors['wheel_count_l'] = float(wheel_count_l)
        sensors['wheel_count_r'] = float(wheel_count_r)
        sensors['wheel_distance_l'] = float(wheel_distance_l)
        sensors['wheel_distance_r'] = float(wheel_distance_r)
        # Instantaneous, not the exponential average. Wheel data arrives over
        # serial at only ~5.5Hz, so an EMA at weight 0.1 has a time constant
        # near 2s: measured, a 0.5s command produced 6s of reported wheel
        # motion decaying 2098 -> 320 -> 86 -> 1. That lag is invisible in the
        # logs but poisons any controller that reads wheel speed.
        sensors['wheel_speed_l'] = float(wheel_speed_instant_l)
        sensors['wheel_speed_r'] = float(wheel_speed_instant_r)


class OrientationSubscriber(dynamic_proxy(OrientationDataSubscriber)):

    def onOrientationUpdate(self, timestamp, theta_rad, angular_velocity_rad):
        sensors['theta'] = float(theta_rad)
        sensors['angular_velocity'] = float(angular_velocity_rad)


class BatterySubscriber(dynamic_proxy(BatteryDataSubscriber)):

    def onBatteryVoltageUpdate(self, timestamp, battery_voltage):
        sensors['battery_voltage'] = float(battery_voltage)

    def onChargerVoltageUpdate(self, timestamp, charger_voltage, coil_voltage):
        sensors['charger_voltage'] = float(charger_voltage)
        sensors['coil_voltage'] = float(coil_voltage)


def setup():
    publisher_manager = PublisherManager()

    battery_data = BatteryData.Builder(context, publisher_manager).build()
    battery_data.addSubscriber(BatterySubscriber())

    wheel_data = (WheelData.Builder(context, publisher_manager)
                  .setBufferLength(10)
                  .setExpWeight(0.5)
                  .build())
    wheel_data.addSubscriber(WheelSubscriber())

    (OrientationData.Builder(context, publisher_manager).build()
     .addSubscriber(OrientationSubscriber()))

    publisher_manager.initializePublishers()
    publisher_manager.startPublishers()

    context.setSerialCommManager(
        SerialCommManager(context.usbSerial, battery_data, wheel_data))
    context.onSetupReady()
    context.guiUpdater.setTrainerStatus('connecting to %s:%d' % (
        BuildConfig.IP, BuildConfig.PORT))


def loop():
    """One control step, or one reconnect attempt if we are not connected."""
    global step, next_tick
    if sock is None:
        # Keep refreshing the screen while idle, so the phone is a usable
        # sensor readout on its own and not only while a trainer is attached.
        # Connection attempts stay on their own slower cadence.
        stop_wheels()
        update_gui()
        if time.monotonic() - last_attempt >= RETRY_DELAY:
            connect()
        else:
            time.sleep(IDLE_PERIOD)
        return
    try:
        # Split the step so a stall can be attributed. wait_ms is time blocked
        # on the trainer, so it carries the network and the policy compute;
        # work_ms is purely local and should be a near-constant control period.
        # A spike in work_ms means the phone stalled (GC, CPU contention); a
        # spike only in wait_ms means the link or the trainer did.
        before_read = time.monotonic()
        act = read()
        after_read = time.monotonic()
        if act['type'] != 'act':
            raise ValueError('Expected an act message, got %r' % act['type'])
        if act['reset']:
            stop_wheels()
            step = 0
        else:
            drive(act['left'], act['right'])
        wait_for_tick()
        before_write = time.monotonic()
        write(dict(type='obs', step=step, t=time.time(),
                   wait_ms=(after_read - before_read) * 1e3,
                   work_ms=(before_write - after_read) * 1e3,
                   sensors=dict(sensors)))
        step += 1
    except Exception as e:
        print('dreamerBridge: lost the trainer at step %d: %s' % (step, e))
        context.guiUpdater.setTrainerStatus('disconnected (%s)' % e)
        stop_wheels()
        disconnect()
    finally:
        update_gui()


def update_gui():
    """Mirror the current step onto the phone screen, at a human rate."""
    global last_report, last_gui, control_hz
    now = time.monotonic()
    if sock is None:
        # Idle refresh rate is not the control rate; do not let it pollute it.
        control_hz = 0.0
        last_report = 0.0
    elif last_report:
        # Exponential average, so a single slow step does not dominate.
        measured = 1.0 / max(now - last_report, 1e-6)
        control_hz = 0.9 * control_hz + 0.1 * measured
        last_report = now
    else:
        last_report = now
    if now - last_gui < GUI_PERIOD:
        return
    last_gui = now
    gui = context.guiUpdater
    gui.setBatteryVoltage(sensors['battery_voltage'])
    gui.setChargerVoltage(sensors['charger_voltage'])
    gui.setCoilVoltage(sensors['coil_voltage'])
    gui.setThetaDeg(math.degrees(sensors['theta']))
    gui.setAngularVelocityDeg(math.degrees(sensors['angular_velocity']))
    gui.setWheelLeftData('%d : %.2f : %.2f' % (
        sensors['wheel_count_l'], sensors['wheel_distance_l'],
        sensors['wheel_speed_l']))
    gui.setWheelRightData('%d : %.2f : %.2f' % (
        sensors['wheel_count_r'], sensors['wheel_distance_r'],
        sensors['wheel_speed_r']))
    gui.setStepInfo(str(step))
    gui.setControlHz(control_hz)
    gui.displayValues()


def wait_for_tick():
    """Hold the action for the rest of the control period."""
    global next_tick
    now = time.monotonic()
    period = 1.0 / CONTROL_HZ
    if next_tick <= now - period:
        # First step, or we fell far enough behind that catching up would mean
        # a burst of zero-length steps. Start the schedule fresh instead.
        next_tick = now
    next_tick += period
    remaining = next_tick - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)


def drive(left, right):
    context.outputs.setWheelOutput(float(left), float(right), False, False)
    context.guiUpdater.setLastAction('L %+.2f  R %+.2f' % (left, right))


def stop_wheels():
    if context is not None and context.outputs is not None:
        context.outputs.setWheelOutput(0.0, 0.0, True, True)
        context.guiUpdater.setLastAction('stopped')


def connect():
    """Try once to reach the trainer, without blocking the loop for long."""
    global sock, stream, last_attempt, next_tick
    last_attempt = time.monotonic()
    address = (BuildConfig.IP, BuildConfig.PORT)
    try:
        candidate = socket.create_connection(address, timeout=CONNECT_TIMEOUT)
        # Reads block for much longer than the connect handshake may.
        candidate.settimeout(RECV_TIMEOUT)
        candidate.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock, stream = candidate, candidate.makefile('rb')
        write(dict(type='hello', protocol=PROTOCOL, control_hz=CONTROL_HZ,
                   robot_id=ROBOT_ID))
        hello = read()
        if hello['type'] != 'hello' or hello['protocol'] != PROTOCOL:
            raise ValueError('Unexpected handshake %r' % hello)
        next_tick = 0.0
        print('dreamerBridge: connected to %s:%d' % address)
        context.guiUpdater.setTrainerStatus('connected to %s:%d' % address)
    except Exception as e:
        print('dreamerBridge: cannot reach %s:%d: %s' % (address + (e,)))
        context.guiUpdater.setTrainerStatus('no trainer at %s:%d' % address)
        disconnect()


def disconnect():
    global sock, stream
    for handle in (stream, sock):
        try:
            handle and handle.close()
        except OSError:
            pass
    sock, stream = None, None


def write(header, blob=b''):
    header = dict(header, blob_len=len(blob))
    payload = json.dumps(header).encode('utf-8')
    sock.sendall(struct.pack('>I', len(payload)) + payload + blob)


def read():
    length, = struct.unpack('>I', readexactly(4))
    header = json.loads(readexactly(length).decode('utf-8'))
    readexactly(header.get('blob_len', 0))  # Unused until camera frames land.
    return header


def readexactly(amount):
    if not amount:
        return b''
    data = stream.read(amount)
    if data is None or len(data) < amount:
        raise ConnectionError('Trainer closed the connection')
    return data
