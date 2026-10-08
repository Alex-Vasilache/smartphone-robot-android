"""Phone side of the DreamerV3 bridge.

The policy runs on the training computer, not here. Each control step this
client applies an action to the wheels, waits out the control period, then
reports the proprio sensors it collected. The phone owns the control clock.

The trainer's handshake reply picks how tightly the two ends are coupled:

* Pipelined (the default). We tick on our own clock at CONTROL_HZ, apply the
  most recent action that has arrived, and report every tick without ever
  blocking on the trainer. The round trip hides inside the control period, so
  the loop runs at CONTROL_HZ no matter what the policy costs. An action
  therefore lands one tick after the observation that prompted it, and an
  action can be applied over more than one tick if the next one is late.

* Lock-step (`pipeline` false in the handshake). We block for an action before
  every tick, so an action is never applied twice but the loop rate is
  1/(control period + round trip). Measured at ~9.7Hz against a 50Hz tick.

The trainer is `embodied/envs/robot.py` in the dreamerv3 repo, and
`tools/fake_robot_phone.py` there is the same protocol without a robot, useful
for testing the computer side on its own.

Wire format, both directions: a 4-byte big-endian header length, a UTF-8 JSON
header, then `blob_len` bytes of binary payload. The blob is unused while the
observation is proprio only; it is where camera frames go.

The trainer address defaults to `config.json` at the repository root (copy
`config.template.json`), compiled into `BuildConfig.IP`/`PORT`. A
`trainer.json` pushed with adb overrides it without a rebuild, and is reread
on every connection attempt:

    adb push trainer.json /sdcard/Android/data/jp.oist.abcvlib.dreamerBridge/files/

with `{"ip": "10.210.28.40", "port": 3000, "max_hz": 25}`; every key is
optional.
"""

import json
import math
import os
import socket
import struct
import sys
import time

from android.os import SystemClock
from java import dynamic_proxy
from jp.oist.abcvlib.core import BuildConfig
from jp.oist.abcvlib.core.inputs import PublisherManager
from jp.oist.abcvlib.core.inputs.microcontroller import (
    BatteryData, WheelData, BatteryDataSubscriber, WheelDataSubscriber)
from jp.oist.abcvlib.core.inputs.phone import (
    OrientationData, OrientationDataSubscriber)
from jp.oist.abcvlib.util import (
    ControlLatencyTrace, SensorLatencyTrace, SerialCommManager)
from jp.oist.abcvlib.dreamerBridge import SensorSnapshot

import bridge_link
from command import CommandSource
import webui

PROTOCOL = 3
CONTROL_HZ = 50.0
# The GUI is for a human watching the robot, so it does not need to keep up
# with the control loop. Above ~10 Hz the runOnUiThread hop starts costing
# more than the control step itself and shows up as jitter at the trainer.
GUI_PERIOD = 0.1
# The web page (webui.py) gets its own, faster refresh: it is pushed over a
# socket on another thread, so it costs the loop only building a small dict.
WEB_PERIOD = 0.04
RECV_TIMEOUT = 30.0
# Connecting must fail fast. The control loop is single threaded, so a blocking
# connect freezes sensing and the display with it -- which looks exactly like a
# stale sensor. A refused connection returns immediately; this only bounds the
# case where the trainer's host swallows the SYN.
CONNECT_TIMEOUT = 0.5
RETRY_DELAY = 2.0
# Pipelined mode never blocks on the trainer, so nothing stops the wheels by
# itself if the trainer stalls or the link goes quiet: without this the robot
# keeps driving the last action it got. Lock-step got this for free.
ACTION_TIMEOUT = 1.0
# How often the idle loop refreshes the display while waiting for a trainer.
IDLE_PERIOD = 0.05
ROBOT_ID = 1
# Display smoothing of the per-step reward: ~0.5 s of steps at 25 Hz.
REWARD_AVG_ALPHA = 0.08
# Serial pacing runs as fast as the RP2040 answers. On the stock firmware that
# is ~11 Hz; on the fast firmware (RTT-LoopReduction) it can exceed 50 Hz,
# which shortens the return horizon in seconds and, in the real-time cartpole
# sim, lost reacher_easy. Cap the decision rate here. 0 disables. trainer.json
# and the trainer's handshake may override it.
MAX_HZ = 25.0
# How the control loop paces itself. 'serial': one decision per RP2040 command
# cycle, taken the moment the previous reply lands -- the only instant a new
# command is applied promptly, because the microcontroller reads USB only
# between commands and is busy for ~83 ms after each one. 'clock': the original
# CONTROL_HZ tick, which on this firmware gets the same 12 Hz of wheel updates
# but chooses them at random from the tick stream and applies them ~20-40 ms
# late. The trainer may override this in the handshake.
PACE = 'serial'
# Sample this long after the RP2040's reply. Measured 2026-09-16 by sweeping
# it: the firmware's first USB poll that a command can catch is ~13.5 ms after
# the reply is seen here, and polls repeat every 10 ms. With the policy at
# ~6 ms and drive() at ~1 ms, sampling at +3..4 ms is the freshest observation
# whose command still makes that poll; +8 misses it and costs a whole poll.
WAKE_LEAD_MS = 3.0
# Serial pacing, trainer-driven: how long to hold the free microcontroller
# waiting for the trainer's action before repeating the last one. The round
# trip is ~10 ms; the firmware polls every 10 ms, so a miss costs one poll.
ACT_WAIT_MS = 25.0
# While waiting for the RP2040, spin on the idle check instead of sleeping in
# 1 ms slices. A core that sleeps ~90 ms of every 100 has its clock dropped by
# the governor, and the policy then runs cold: measured 2026-09-14, 21-25 ms
# per step in the loop against 3.5 ms in a hot benchmark on the same phone.
# Each Java call in the spin releases the GIL, so the sensor callbacks still
# get through.
SPIN_WAIT = False
# Spin (no sleep) for the last few ms before the reply is due, so the core
# does not drop into idle between the warm-up and the real step. Bounded, and
# safe now that no sensor callback needs the GIL. 0 disables.
SPIN_BEFORE_MS = 0.0
# Decide *before* the reply lands: sample and run the policy this many ms
# before the RP2040's reply is due, then write the command the instant the
# link is free. The observation is older by that much, but the command is in
# the microcontroller's FIFO for its first poll rather than its second, which
# is one 10 ms poll off the cycle. 0 disables (decide after the reply).
PRESAMPLE_MS = 0.0
# Trainer-driven counterpart: ship the observation this long before the reply
# is due, so the trainer's action (~17 ms round trip) is back by the time the
# link is free and the command makes the +13.5 ms poll. Measured 2026-09-16:
# without it the cycle was 112 ms; the observation is ~10 ms older at apply.
TRAINER_PRESAMPLE_MS = 12.0
# Pin the control thread to the big cores (CPUs 6-7 on the Pixel 3a): the
# policy step's p95 went from 24 ms to 10 ms with nothing else changed.
PIN_CPUS = (6, 7)
# Precompute the observation-independent half of the policy step (the GRU)
# right after each action, while the RP2040 is busy. See PolicyRunner.prepare.
PREPARE_AHEAD = True
# Longest the loop will wait for the RP2040 before going on without it. A
# normal cycle is ~83 ms and a coast-induced stall was 1.09 s; a dead link is
# forever, and measured 2026-09-16 the loop then hung in wait_for_serial with
# the trainer connected and no observation ever sent. Past this the frame is
# still reported, with `ser_ok` false, and the command is queued for when the
# link comes back.
SERIAL_WAIT_MAX_MS = 250.0
# How long the serial writer waits for the RP2040's reply before sending the
# next command anyway. The fast firmware answers in ~8 ms (p95 ~10, max ~16
# on 2026-10-06), and with abcvlib's 10 s default one lost reply froze the
# wheels for 10 s. Raise to ~1500 for the stock firmware, which stalls ~1.1 s
# after a coast.
REPLY_TIMEOUT_MS = 250
# Run a throwaway policy step this many ms before the RP2040's reply is due,
# so the real one runs on a warm core. The step costs 3.5 ms hot and 10 ms
# cold on this phone (measured 2026-09-16, sensors already off the GIL and the
# thread pinned to the big cores), and 10 ms is just enough to miss the
# firmware's first USB poll after its reply, which costs a whole extra poll
# per cycle. The reply lands ~81 ms after the previous dequeue, p95 87.
# 0 disables.
WARM_MS = 6.0
# Typical dequeue -> reply on this firmware; only used to time the warm-up.
REPLY_MS = 81.0
# Largest change in wheel command per drive() call, passed to
# Outputs.setWheelOutput. abcvlib's default of 0.4 makes a reversal step
# through 0.0, which the firmware maps to coast; measured 2026-09-14, every
# 1 s serial stall followed a coast -> drive step. The trainer may set this per
# action ('slew') to test alternatives.
SLEW_MAX = 2.0
# What to send when the command is inside the dead zone, |cmd| < 0.097, which
# abcvlib maps to coast (H-bridge off). Measured 2026-09-14 with a PRBS
# between 0.4 and 0: coast -> drive transitions stalled the RP2040 for 1.09 s
# on 9 of 15 edges (DRV8830 UVLO when the bridge re-energises, then the
# firmware's 1 s I2C timeout); 'brake' and 'min' gave 0 stalls in ~270 cycles.
#   'coast'  pass it through (abcvlib behaviour)
#   'brake'  short the motor instead (IN1 = IN2 = 1); bridge stays energised
#   'min'    the smallest driven value, keeping the sign of the last command --
#            ~0.66 V on a 5 V motor, below stiction, so the torque is nil but
#            the bridge never drops out. The closest thing to "zero" the
#            hardware can do safely.
ZERO_MODE = 'min'
DEAD_ZONE = 0.097
# The smallest command that reaches DRV8830 register 0x06, its lowest legal
# VSET: abcvlib's scaling gives 0x05 -- a reserved value -- for anything under
# 0.111. Nothing about the dead zone itself.
MIN_DRIVE = 0.13
# Above this the IMU sample is not late, it is from a different moment: the
# sensor handler is backed up. Normal is ~10 ms.
STALE_SENSOR_MS = 250.0

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

# Onboard policy. When the trainer agrees at handshake, the policy runs here
# and the trainer only receives experience and pushes weights back. The point
# is not bandwidth -- the round trip already hides inside a control period --
# it is that the action can be applied in the SAME tick as the observation
# that produced it. Measured 2026-09-07, waiting for the trainer's reply cost a
# full 20ms of a ~30ms sensor-to-torque budget, against a robot whose tilt
# diverges with a ~50ms time constant. See docs/ONDEVICE_POLICY.md.
onboard = False
# Sticky across disconnects. `onboard` is negotiated per connection, but a
# robot that is balancing must not have its wheels cut the moment the trainer
# goes quiet -- stopping is what makes it fall over. Once a trainer has agreed
# we hold the policy, we keep driving on the weights we have and reconnect in
# the background.
standalone = False
runner = None

sock = None
# Own framing buffer instead of a buffered file object, for the handshake only;
# after it the socket belongs to `receiver` (bridge_link.Receiver), a thread
# that reads it and parses pushed weights so the control loop never does.
buffer = bytearray()
receiver = None
# No base on the phone: no serial link and no wheels. Everything else -- the
# sensors, the policy, the trainer link and the pacing -- runs as on the robot,
# which is what makes it a test of the link at a given rate. Set with
# "no_base": true in trainer.json; MainActivity then starts without USB.
no_base = False
# Player mode (the Dreamer Player launcher): run one saved policy with no
# trainer at all. `player_policy` is the .npz path, injected by abcvlib.py;
# its <name>.json sidecar (tools/save_policy.sh) gives the rate, episode length
# and reward settings, so the screen scores steps the way training did.
player_policy = None
player_eval = True
player = None      # the sidecar dict once loaded
prev_act = (0.0, 0.0)
pipeline = True
imu_stamp = 0.0     # monotonic time of the last orientation callback
imu_age_ms = 0.0    # sensor hardware time to callback, milliseconds
step = 0
next_tick = 0.0
last_attempt = 0.0
last_report = 0.0
last_gui = 0.0
last_act = 0.0
control_hz = 0.0
drive_ms = 0.0    # cost of handing one action to the serial writer
pace = PACE        # negotiated per connection
max_hz = MAX_HZ    # negotiated per connection
cmd_scale = 1.0    # every wheel command is multiplied by this (trainer's hello)
weights_installed = 0  # weight pushes swapped in this session
reward_avg = 0.0   # display only, see note_reward
ep_return = 0.0
ep_steps = 0
next_slot = 0.0    # monotonic time of the next decision when max_hz caps it
last_drive = 0.0   # monotonic time of the previous drive(), for cycle_ms
seq = 0            # observation sequence, echoed by the trainer in its action
cycle_ms = 0.0     # time between the previous drive() and this one
last_wait_ms = 0.0
last_work_ms = 0.0
last_command = (0.0, 0.0)
t_drive = 0.0      # monotonic time of the last drive(), same clock as nanoTime
t_dequeued = 0.0   # ~ when the writer sent it; the reply is due REPLY_MS later
slew_max = SLEW_MAX
zero_mode = ZERO_MODE
wake_lead_ms = WAKE_LEAD_MS
spin_wait = SPIN_WAIT
spin_before_ms = SPIN_BEFORE_MS
presample_ms = PRESAMPLE_MS
trainer_presample_ms = TRAINER_PRESAMPLE_MS
warm_ms = WARM_MS
warm_n = 1
prepare_ahead = PREPARE_AHEAD
serial_ok = True          # the RP2040 answered within SERIAL_WAIT_MAX_MS
serial_misses = 0         # waits that hit SERIAL_WAIT_MAX_MS, for the display
last_serial_warning = 0.0
last_stale_warning = 0.0
# The command a command-task policy follows (command.py), and the web page
# that steers it and mirrors this screen (webui.py).
commands = CommandSource()
web = None
web_url = ''
# Wheel speeds that command 1.0 means, from the trainer's settings; only for
# showing what the robot is doing next to what it is asked. None: unknown.
command_units = None
last_reward = 0.0
last_web = 0.0
web_status = ('', '')  # trainer status and serial note, as of the last GUI refresh
last_ep_mean = None
last_ep_return = None


# The sensor subscribers live in Kotlin (SensorSnapshot.kt). They used to be
# Python classes here, and every one of the ~200 orientation events a second
# then crossed Chaquopy and took the GIL from the policy step: 3.5 ms per step
# became 20-25 ms, and the sensor callbacks queued behind the policy in turn.
# Python now reads one JSON snapshot per control step and enters no callback.


def setup():
    global runner, no_base
    no_base = bool(trainer_settings().get('no_base', False))
    # The receiver thread parses weight pushes in Python. At the default 5 ms
    # switch interval it could hold the GIL for that long while the control
    # thread waits to decide; 1 ms bounds the wait to a fraction of a tick.
    sys.setswitchinterval(0.001)
    # Built before the first connection so the handshake can honestly say
    # whether we can act on our own. With no weights on disk yet this is a
    # runner that is not `ready`, and the trainer keeps the policy.
    if PIN_CPUS:
        try:
            os.sched_setaffinity(0, set(PIN_CPUS))
        except Exception as e:  # noqa: BLE001
            print('dreamerBridge: could not pin the control thread: %s' % e)
    global web, web_url
    web = webui.WebUI(commands)
    web_url = web.start() or ''
    from policy_runner import PolicyRunner
    if player_policy:
        setup_player(PolicyRunner)
    else:
        runner = PolicyRunner(boot_policy())
    print('dreamerBridge: onboard policy %s' % (
        'ready (%s)' % runner.stamp if runner.ready else 'absent'))
    if runner.ready:
        # Which stage dominates is a property of this phone, not of the port.
        print('dreamerBridge: policy timing  %s' % runner.benchmark())
        try:
            import numpy as _np
            blas = _np.show_config('dicts')['Build Dependencies']['blas']
            print('dreamerBridge: numpy %s blas=%s' % (_np.__version__, blas))
        except Exception as e:  # noqa: BLE001
            print('dreamerBridge: numpy config unavailable: %s' % e)

    # Diagnostic reference for the fused orientation signal. Registered before
    # the publishers start, because that is when the listeners are attached.
    SensorLatencyTrace.setGyro(True)
    # Off by default; the trainer turns it on with a `ctrl` frame so the
    # change can be measured against the blocking path in one session.
    ControlLatencyTrace.useAsyncMotor(False)

    publisher_manager = PublisherManager()

    snapshot = SensorSnapshot.subscriber()
    battery_data = BatteryData.Builder(context, publisher_manager).build()
    battery_data.addSubscriber(snapshot)

    wheel_data = (WheelData.Builder(context, publisher_manager)
                  .setBufferLength(10)
                  .setExpWeight(0.5)
                  .build())
    wheel_data.addSubscriber(snapshot)

    (OrientationData.Builder(context, publisher_manager).build()
     .addSubscriber(snapshot))

    publisher_manager.initializePublishers()
    publisher_manager.startPublishers()

    if no_base:
        print('dreamerBridge: no base -- no serial link, wheels not driven')
    else:
        serial = SerialCommManager(context.usbSerial, battery_data, wheel_data)
        serial.setReplyTimeoutMs(REPLY_TIMEOUT_MS)
        context.setSerialCommManager(serial)
        context.onSetupReady()
    if player:
        context.guiUpdater.setTrainerStatus('playing %s' % player['name'])
        return
    settings = trainer_settings()
    context.guiUpdater.setTrainerStatus('connecting to %s:%s' % (
        settings['ip'], settings['port']))


def setup_player(PolicyRunner):
    """Load the chosen policy and its settings; no trainer will connect."""
    global runner, player, onboard, max_hz, cmd_scale, pace
    path = str(player_policy)
    side = os.path.splitext(path)[0] + '.json'
    player = dict(name=os.path.basename(os.path.splitext(path)[0]), hz=MAX_HZ,
                  length=500, command_scale=1.0)
    try:
        with open(side) as f:
            player.update(json.load(f))
    except Exception as e:  # noqa: BLE001 -- run on defaults rather than not at all
        print('dreamerBridge: no settings for %s (%s); using defaults' % (path, e))
    runner = PolicyRunner()
    runner.load_path(path, stamp=player['name'])
    runner.mode = 'eval' if player_eval else 'train'
    # `onboard` turns on the warm-up before each step, as when training.
    onboard = True
    max_hz = float(player['hz'])
    cmd_scale = float(player.get('command_scale', 1.0))
    # Nobody needs random targets on a finished policy: it balances until the
    # joystick says otherwise.
    commands.set_mode('manual')
    note_command_units(player)
    pace = PACE
    print('dreamerBridge: player running %s at %g Hz (%s actions)' % (
        player['name'], max_hz, runner.mode))


def note_command_units(settings):
    """Wheel speeds per command unit, signed: forward_sign -1 means forward
    is the robot's negative wheel direction (robot.py)."""
    global command_units
    if settings and settings.get('command_speed') and settings.get('command_turn'):
        command_units = (
            float(settings['command_speed'])
            * float(settings.get('forward_sign', 1.0)),
            float(settings['command_turn']))


def player_reward(sensors, act, cmd):
    """The training reward (embodied/envs/robot.py, tasks balance and
    command), for display."""
    global prev_act
    p = player
    if 'theta_zero' not in p:
        return 0.0
    offset = float(sensors['theta']) - p['theta_zero']
    reach = p['theta_hi'] if offset > 0 else abs(p['theta_lo'])
    linear = 1.0 - min(1.0, abs(offset) / max(reach, 1e-6))
    bonus = math.exp(-(offset / p['theta_sigma']) ** 2)
    left = float(sensors['wheel_speed_l'])
    right = float(sensors['wheel_speed_r'])
    balance, track = 0.5 * linear + 0.5 * bonus, 0.0
    if p.get('task') == 'command' and command_units:
        # _command_reward: penalties measured from the commanded wheel speeds.
        speed, turn = command_units
        sigma = float(p.get('command_sigma', 0.3))
        forward = 0.5 * (left + right) / speed
        spin = 0.5 * (left - right) / turn
        track = 0.5 * (math.exp(-((forward - cmd[0]) / sigma) ** 2)
                       + math.exp(-((spin - cmd[1]) / sigma) ** 2))
        left -= cmd[0] * speed + cmd[1] * turn
        right -= cmd[0] * speed - cmd[1] * turn
        # 'sum' (e1299 and older sidecars, which carry no command_reward)
        # is half and half; 'product' pays balance times tracking.
        if p.get('command_reward', 'sum') == 'product':
            balance, track = balance * track, 0.0
        else:
            balance, track = 0.5 * balance, 0.5 * track
    left *= p['speed_scale']
    right *= p['speed_scale']
    clip = p['drift_clip']
    reward = (balance + track
              - p['rate_penalty'] * abs(float(sensors['angular_velocity']))
              - p['drift_penalty'] * min(abs(0.5 * (left + right)), clip)
              - p['wheel_penalty'] * 0.5 * (min(abs(left), clip)
                                            + min(abs(right), clip)))
    if p.get('action_rate_penalty'):
        reward -= p['action_rate_penalty'] * 0.5 * (
            abs(act[0] - prev_act[0]) + abs(act[1] - prev_act[1]))
    prev_act = (act[0], act[1])
    return reward


def loop_player():
    """One control step of a saved policy: the onboard path minus the trainer."""
    global step, prev_act
    wait_for_pace()
    snap, _, _ = sample()
    cmd = commands.tick()
    _, act = runner.act(snap, cmd)
    drive(act[0], act[1])
    note_reward(player_reward(snap, act, cmd))
    step += 1
    if step >= int(player['length']):
        # Episodes as long as in training: the policy never saw a longer
        # context, so start its memory afresh. The wheels keep going.
        end_episode()
        runner.reset()
        prev_act = (0.0, 0.0)
        step = 0
    if prepare_ahead:
        runner.prepare()
    update_gui()


def sample():
    """Take the sensors and the age of the sample that produced them, together.

    Reading `sensors` field by field while the sensor thread writes it can mix
    two different samples, and reading `imu_stamp` later -- as the timing block
    used to -- dates the observation by a callback that had not happened yet
    when the observation was taken. That is why `imu_stale_ms` was reported as
    *negative*: an impossible number, and a warning that the timestamps were
    not measuring what they claimed.
    """
    global last_stale_warning, imu_stamp, imu_age_ms
    snap = json.loads(SensorSnapshot.snapshotJson())
    imu_age_ms = snap.pop('imu_age_ms')
    imu_stamp = snap.pop('imu_callback_s')
    snap.pop('wheel_callback_s', None)
    sensors.update(snap)  # for update_gui() and the idle display
    # A sample this old means the sensor pipeline is backed up, not that the
    # robot is slow: the control loop is running on a picture of the world
    # that is seconds out of date, and nothing else will say so.
    if imu_age_ms > STALE_SENSOR_MS and \
            time.monotonic() - last_stale_warning > 5.0:
        last_stale_warning = time.monotonic()
        print('dreamerBridge: IMU sample is %.0f ms old -- the sensor pipeline '
              'is backed up' % imu_age_ms)
        context.guiUpdater.setTrainerStatus(
            'STALE SENSORS: %.0f ms' % imu_age_ms)
    return snap, imu_stamp, imu_age_ms


def latency_block(taken_at, sensor_stamp, sensor_age_ms):
    """Everything known about how old this observation is, and what the wheels
    were actually told, in one place.

    `drive()` returns as soon as the command is in the serial writer's one-slot
    mailbox, so no phone-side timing before this covered the part that moves
    the robot: the USB round trip to the RP2040, and the commands that get
    overwritten in the mailbox before they are ever sent.
    """
    block = dict(
        imu_age_ms=sensor_age_ms,
        # Age of the sample the policy actually saw, measured at the instant it
        # was taken rather than when the message was built.
        imu_stale_ms=(taken_at - sensor_stamp) * 1e3 if sensor_stamp else -1.0,
    )
    # Both traces arrive as JSON: see ControlLatencyTrace.snapshotJson.
    block.update(json.loads(ControlLatencyTrace.snapshotJson()))
    block.update(json.loads(SensorLatencyTrace.snapshotJson()))
    block['t_drive'] = t_drive
    block['t_obs'] = taken_at
    block['ser_ok'] = serial_ok
    return block


def loop():
    """One control step, or one reconnect attempt if we are not connected."""
    global step, next_tick, last_act
    if player is not None:
        return loop_player()
    if sock is None:
        if standalone and runner is not None and runner.ready:
            # No trainer, but we still have the policy that was balancing a
            # moment ago. Keep the loop closed and retry the connection between
            # ticks; experience collected now is simply not recorded.
            wait_for_pace()
            snap, _, _ = sample()
            _, act = runner.act(snap, commands.tick())
            drive(act[0], act[1])
            if prepare_ahead:
                runner.prepare()
            if time.monotonic() - last_attempt >= RETRY_DELAY:
                context.guiUpdater.setTrainerStatus('standalone (no trainer)')
                connect()
            update_gui()
            return
        # Keep refreshing the screen while idle, so the phone is a usable
        # sensor readout on its own and not only while a trainer is attached.
        # Connection attempts stay on their own slower cadence.
        stop_wheels()
        sample()
        commands.tick()  # nothing follows it yet, but the page shows it
        update_gui()
        if time.monotonic() - last_attempt >= RETRY_DELAY:
            connect()
        else:
            time.sleep(IDLE_PERIOD)
        return
    try:
        if onboard:
            return loop_onboard()
        if pace == 'serial':
            return loop_trainer_serial()
        # Split the step so a stall can be attributed. wait_ms is time blocked
        # on the trainer, so it carries the network and the policy compute;
        # work_ms is purely local and should be a near-constant control period.
        # A spike in work_ms means the phone stalled (GC, CPU contention); a
        # spike only in wait_ms means the link or the trainer did.
        before_read = time.monotonic()
        if pipeline:
            # Never block: take the newest action that has arrived and keep
            # driving the last one otherwise. Anything older than the newest is
            # already superseded, so applying it would only add delay.
            act = None
            while True:
                frame = read_nowait()
                if frame is None:
                    break
                header, payload = frame
                if header.get('type') == 'weights_ready':
                    # Bootstrap: we came up without a policy, so the trainer is
                    # still driving. Once its weights land we can take over,
                    # but onboard mode is settled at handshake -- so drop the
                    # link and let the next connect negotiate it.
                    runner.install(payload, header.get('stamp'))
                    print('dreamerBridge: got a policy, reconnecting to '
                          'take over control')
                    stop_wheels()
                    disconnect()
                    return
                act = header
        else:
            act = read()
        after_read = time.monotonic()
        if act is not None:
            if act['type'] != 'act':
                raise ValueError(
                    'Expected an act message, got %r' % act['type'])
            # The trainer scores the observation we just sent, so the reward
            # that comes back with the next action is this state's.
            apply_overrides(act)
            note_reward(float(act.get('reward', 0.0)))
            if act['reset']:
                stop_wheels()
                step = 0
                end_episode()
            else:
                drive(act['left'], act['right'])
            last_act = time.monotonic()
        elif time.monotonic() - last_act > ACTION_TIMEOUT:
            # The trainer has gone quiet. Coast rather than keep driving.
            stop_wheels()
        wait_for_tick()
        before_write = time.monotonic()
        snap, snap_stamp, snap_age = sample()
        cmd = commands.tick()
        write(dict(type='obs', step=step, t=time.time(),
                   cmd=cmd, cmd_src=commands.source,
                   # True when this tick applied a new action rather than
                   # repeating the previous one.
                   fresh=act is not None,
                   wait_ms=(after_read - before_read) * 1e3,
                   work_ms=(before_write - after_read) * 1e3,
                   drive_ms=drive_ms,
                   sensors=snap,
                   **latency_block(before_write, snap_stamp, snap_age)))
        step += 1
    except Exception as e:
        print('dreamerBridge: lost the trainer at step %d: %s' % (step, e))
        context.guiUpdater.setTrainerStatus('disconnected (%s)' % e)
        if not (standalone and runner is not None and runner.ready):
            stop_wheels()
        disconnect()
    finally:
        update_gui()


def note_reward(reward):
    """Track the reward the trainer scored for each step, for the display.

    One step's reward jumps around at 25 Hz and the screen refreshes at 10, so
    the headline number is an exponential average over ~0.5 s of steps.
    """
    global reward_avg, ep_return, ep_steps, last_reward
    last_reward = reward
    reward_avg += REWARD_AVG_ALPHA * (reward - reward_avg)
    ep_return += reward
    ep_steps += 1
    gui = context.guiUpdater
    gui.setReward(reward)
    gui.setRewardAvg(reward_avg)
    gui.setEpisodeMean(ep_return / ep_steps)
    gui.setEpisodeReturn(ep_return)


def end_episode():
    global ep_return, ep_steps, last_ep_mean, last_ep_return
    if ep_steps:
        last_ep_mean, last_ep_return = ep_return / ep_steps, ep_return
        context.guiUpdater.setLastEpisodeReturn(ep_return)
        context.guiUpdater.setLastEpisodeMean(ep_return / ep_steps)
    ep_return = 0.0
    ep_steps = 0


def update_gui():
    """Mirror the current step onto the phone screen, at a human rate."""
    global last_report, last_gui, control_hz
    now = time.monotonic()
    if sock is None and player is None:
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
    global last_web
    if web is not None and now - last_web >= WEB_PERIOD:
        last_web = now
        web.publish(web_state())
    if now - last_gui < GUI_PERIOD:
        return
    last_gui = now
    gui = context.guiUpdater
    gui.setBatteryVoltage(sensors['battery_voltage'])
    gui.setChargerVoltage(sensors['charger_voltage'])
    gui.setCoilVoltage(sensors['coil_voltage'])
    gui.setThetaDeg(math.degrees(sensors['theta']))
    gui.setAngularVelocityDeg(math.degrees(sensors['angular_velocity']))
    gui.setWheelSpeedL(float(sensors['wheel_speed_l']))
    gui.setWheelSpeedR(float(sensors['wheel_speed_r']))
    gui.setWheelCountL(int(sensors['wheel_count_l']))
    gui.setWheelCountR(int(sensors['wheel_count_r']))
    gui.setActionL(float(last_command[0]))
    gui.setActionR(float(last_command[1]))
    gui.setLastAction('')
    gui.setStep(int(step))
    gui.setControlHz(control_hz)
    gui.setCommand(float(commands.current[0]), float(commands.current[1]),
                   commands.source, web_url)
    gui.displayValues()
    global web_status
    web_status = (str(gui.getTrainerStatus()), str(gui.getSerialNote()))


def web_state():
    """What the web page's mirror shows: this screen's numbers, as JSON."""
    measured = None
    if command_units:
        left = float(sensors['wheel_speed_l'])
        right = float(sensors['wheel_speed_r'])
        measured = [0.5 * (left + right) / command_units[0],
                    0.5 * (left - right) / command_units[1]]
    return dict(
        reward=last_reward, reward_avg=reward_avg,
        ep_mean=(ep_return / ep_steps) if ep_steps else 0.0,
        ep_return=ep_return, last_ep_mean=last_ep_mean,
        last_ep_return=last_ep_return,
        theta_deg=math.degrees(sensors['theta']),
        rate_deg=math.degrees(sensors['angular_velocity']),
        speed_l=float(sensors['wheel_speed_l']),
        speed_r=float(sensors['wheel_speed_r']),
        count_l=int(sensors['wheel_count_l']),
        count_r=int(sensors['wheel_count_r']),
        act_l=float(last_command[0]), act_r=float(last_command[1]),
        step=int(step), hz=control_hz,
        battery=float(sensors['battery_voltage']),
        charger=float(sensors['charger_voltage']),
        coil=float(sensors['coil_voltage']),
        status=web_status[0], serial=web_status[1],
        cmd=list(commands.current), cmd_src=commands.source,
        mode=commands.settings['mode'], measured=measured,
        grid=float(commands.settings.get('step') or 0.0))


def loop_onboard():
    """One control step with the policy running here.

    Order matters and is the whole point. The legacy path applies an action at
    the top of a tick, sleeps, then samples and ships -- so the reply to an
    observation lands a full period after it was taken. Here we pace first,
    then sample, decide and drive back to back, so the gap between reading the
    IMU and moving the wheels is just the network forward pass (~2-4ms on this
    phone) instead of 20ms.

    Everything that is not on that path -- shipping the transition, taking
    resets, installing new weights -- happens afterwards, inside the slack of
    the same tick.
    """
    global step, last_act, seq
    before_tick = time.monotonic()
    if pace == 'serial' and presample_ms and t_dequeued:
        # Sleep-poll until the decision point, decide, then wait for the link.
        due = t_dequeued + (REPLY_MS - presample_ms) / 1e3
        while time.monotonic() < due:
            if ControlLatencyTrace.serialIdleUs() >= 0:
                break  # the reply came early; decide now
            time.sleep(0.0005)
        tick = time.monotonic()
        snap, snap_stamp, snap_age = sample()
        cmd = commands.tick()
        obs, act = runner.act(snap, cmd)
        wait_for_serial()
    else:
        wait_for_pace()
        tick = time.monotonic()
        snap, snap_stamp, snap_age = sample()
        cmd = commands.tick()
        obs, act = runner.act(snap, cmd)
    drive(act[0], act[1])
    applied = time.monotonic()
    last_act = applied
    seq += 1

    write(dict(type='obs', step=step, seq=seq, t=time.time(), fresh=True,
               pace=pace, cycle_ms=cycle_ms, lead_ms=wake_lead_ms,
               serial_wait_ms=(tick - before_tick) * 1e3,
               # The action we already applied. The trainer records this rather
               # than one of its own: what the world model learns must be what
               # the wheels actually did.
               act=act,
               # The command the policy just followed; the trainer scores
               # the step against it and records it as an observation.
               cmd=cmd, cmd_src=commands.source,
               is_first=runner.was_first,
               # Same split as the legacy path, but here work_ms is the real
               # decision cost, not a sleep: it is sample -> policy -> wheels.
               wait_ms=(tick - before_tick) * 1e3,
               work_ms=(applied - tick) * 1e3,
               policy_ms=runner.last_ms,
               warm_ms=runner.last_warm_ms,
               prepare_ms=runner.last_prepare_ms,
               prepared=runner.prepared,
               drive_ms=drive_ms,
               policy_stamp=runner.stamp,
               weights_installed=weights_installed,
               build_ms=receiver.last_build_ms if receiver else 0.0,
               sensors=snap,
               **latency_block(tick, snap_stamp, snap_age)))
    step += 1
    drain_control()
    # The GRU half of the next step needs only the carry we already have. Do
    # it now, in the ~80 ms the RP2040 is busy, so that when the reply lands
    # only the observation-dependent half is between the sensors and the
    # wheels. drain_control() ran first: a reset there invalidates it.
    if prepare_ahead:
        runner.prepare()


def loop_trainer_serial():
    """One control step, trainer-driven, paced on the serial link.

    The microcontroller is free the instant its reply lands, so that is when
    the observation is taken and shipped. The trainer's action normally comes
    back within ~10 ms, before the firmware's next USB poll; we hold the free
    link for it up to ACT_WAIT_MS and otherwise repeat the last action rather
    than let the microcontroller idle. Sensor-to-torque is therefore the
    network round trip plus the policy, and every action is applied.
    """
    global step, last_act, seq
    if max_hz > 0:
        # The link is free long before the slot; ship the observation
        # TRAINER_PRESAMPLE_MS ahead of it, so it is fresh and the action is
        # back in time. The second wait_for_serial below holds to the slot.
        serial_wait_ms = wait_for_serial(trainer_presample_ms)
    elif trainer_presample_ms and t_dequeued:
        # Ship the observation before the link is free, so the action is back
        # when it is. If the reply comes early, ship now.
        due = t_dequeued + (REPLY_MS - trainer_presample_ms) / 1e3
        t0 = time.monotonic()
        while time.monotonic() < due:
            if ControlLatencyTrace.serialIdleUs() >= 0:
                break
            time.sleep(0.0005)
        serial_wait_ms = (time.monotonic() - t0) * 1e3
    else:
        serial_wait_ms = wait_for_serial()
    tick = time.monotonic()
    snap, snap_stamp, snap_age = sample()
    seq += 1
    cmd = commands.tick()
    write(dict(type='obs', step=step, seq=seq, t=time.time(), fresh=True,
               pace=pace, cycle_ms=cycle_ms, cmd=cmd, cmd_src=commands.source,
               serial_wait_ms=serial_wait_ms,
               wait_ms=last_wait_ms, work_ms=last_work_ms,
               drive_ms=drive_ms, sensors=snap,
               **latency_block(tick, snap_stamp, snap_age)))
    step += 1

    # Wait for the action to this observation. A trainer that echoes `seq`
    # lets us tell it from a late reply to the previous one; one that does not
    # is taken at its word.
    act = None
    deadline = tick + ACT_WAIT_MS / 1e3
    while True:
        frame = read_nowait()
        while frame is not None:
            header, payload = frame
            kind = header.get('type')
            if kind == 'weights_ready':
                runner.install(payload, header.get('stamp'))
                print('dreamerBridge: got a policy, reconnecting to '
                      'take over control')
                stop_wheels()
                disconnect()
                return
            elif kind == 'act' and header.get('seq', seq) == seq:
                act = header
            frame = read_nowait()
        if act is not None or time.monotonic() >= deadline:
            break
        time.sleep(0.0005)
    got = time.monotonic()
    if trainer_presample_ms or max_hz > 0:
        wait_for_serial()

    if act is not None:
        apply_overrides(act)
        note_reward(float(act.get('reward', 0.0)))
        last_act = got
        if act['reset']:
            stop_wheels()
            step = 0
            end_episode()
        else:
            drive(act['left'], act['right'])
    elif time.monotonic() - last_act > ACTION_TIMEOUT:
        # The trainer has gone quiet. Coast rather than keep driving.
        stop_wheels()
    else:
        # Late: repeat the previous command so the link is not left idle.
        drive(*last_command)
    record_split((got - tick) * 1e3, (time.monotonic() - got) * 1e3)


def record_split(wait, work):
    """Keep this cycle's timing for the next frame: the observation goes out
    before the action that answers it is known."""
    global last_wait_ms, last_work_ms
    last_wait_ms, last_work_ms = wait, work


def drain_control():
    """Take whatever the trainer has sent, without blocking the control loop.

    Resets and weight updates are not latency critical, so they are handled
    after the wheels have already been driven for this tick. A weight push
    reaches us already parsed (bridge_link.Receiver did that on its own
    thread), so installing it is an assignment.
    """
    global step, weights_installed
    budget = time.monotonic() + 0.004
    while time.monotonic() < budget:
        frame = read_nowait()
        if frame is None:
            break
        header, payload = frame
        kind = header.get('type')
        if kind == 'weights_ready':
            stamp = header.get('stamp')
            runner.install(payload, stamp)
            weights_installed += 1
            context.guiUpdater.setTrainerStatus('policy %s' % stamp)
        elif kind in ('act', 'ctrl'):
            apply_overrides(header)
            note_reward(float(header.get('reward', 0.0)))
            if header.get('reset'):
                end_episode()
                # The trainer owns episode boundaries; it is the side that
                # knows the length and the termination rule. Only the timing
                # is relaxed -- a reset one tick late costs nothing, unlike an
                # action one tick late.
                stop_wheels()
                runner.reset()
                step = 0


def weights_dir():
    return os.path.join(str(context.getFilesDir()), 'dreamer_policy')


def policies_dir():
    """Saved policies, one <name>.npz + <name>.json each: what the policy
    list shows. Each training run keeps its latest weights here under the
    run's name; tools/save_policy.sh adds others."""
    return os.path.join(str(context.getExternalFilesDir(None)), 'policies')


def boot_policy():
    """The weights to start on: the last ones pushed, wherever they went."""
    try:
        with open(os.path.join(weights_dir(), 'boot.json')) as f:
            path = json.load(f)['path']
        if os.path.exists(path):
            return path
    except Exception:  # noqa: BLE001 -- no record yet: the old location
        pass
    return os.path.join(weights_dir(), 'policy.npz')


def weight_saver(run, settings):
    """What the receiver thread does with each weight push.

    A trainer that names its run gets its weights kept in the policy list as
    <run>.npz, overwritten by every push, so the latest weights of every run
    stay available to replay without saving anything by hand. The sidecar
    carries the rate and reward settings the player needs. Without a name
    (an older trainer) they go where they always did.
    """
    from policy_runner import PolicyRunner
    if not run:
        path = os.path.join(weights_dir(), 'policy.npz')
        return lambda blob, header: PolicyRunner.build_blob(blob, path)
    name = ''.join(c if c.isalnum() or c in '-_.' else '_' for c in run)
    path = os.path.join(policies_dir(), name + '.npz')
    side = dict(settings or {}, name=name, run=run, job=run, hz=max_hz)

    def save(blob, header):
        policy = PolicyRunner.build_blob(blob, path)
        stamp = str(header.get('stamp', ''))
        side['stamp'] = stamp
        try:
            side['published'] = time.strftime(
                '%Y-%m-%d %H:%M', time.localtime(int(stamp) / 1e9))
        except ValueError:
            pass
        write_json(os.path.splitext(path)[0] + '.json', side)
        write_json(os.path.join(weights_dir(), 'boot.json'), dict(path=path))
        return policy

    return save


def write_json(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)


def wait_for_pace():
    """Block until it is time to decide: see PACE."""
    if pace == 'serial':
        wait_for_serial()
    else:
        wait_for_tick()


def wait_for_serial(lead_ms=0.0):
    """Block until the RP2040 has answered the previous command, then until it
    is about to poll USB again, then, under a max_hz cap, until `lead_ms`
    before the next decision slot. Returns how long that took, in ms."""
    global serial_ok, last_serial_warning, serial_misses
    t0 = time.monotonic()
    if no_base:
        # No RP2040 to wait for: hold to the max_hz grid alone.
        if max_hz > 0 and next_slot:
            wait_for_slot(next_slot - lead_ms / 1e3)
        return (time.monotonic() - t0) * 1e3
    deadline = t0 + SERIAL_WAIT_MAX_MS / 1e3
    warmed = False
    while True:
        idle_us = ControlLatencyTrace.serialIdleUs()
        if idle_us >= 0:
            serial_ok = True
            break
        if warm_ms and not warmed and not max_hz and onboard and \
                runner is not None and runner.ready and t_dequeued and \
                time.monotonic() - t_dequeued >= (REPLY_MS - warm_ms) / 1e3:
            warmed = True
            for _ in range(warm_n):
                runner.warm()
            continue
        if time.monotonic() >= deadline:
            serial_ok = False
            serial_misses += 1
            context.guiUpdater.setSerialNote('%d missed replies, last %s' % (
                serial_misses, time.strftime('%H:%M:%S')))
            if time.monotonic() - last_serial_warning > 5.0:
                last_serial_warning = time.monotonic()
                print('dreamerBridge: no reply from the RP2040 for %.0f ms; '
                      'carrying on without it' % SERIAL_WAIT_MAX_MS)
            return (time.monotonic() - t0) * 1e3
        near = spin_before_ms and t_dequeued and \
            time.monotonic() - t_dequeued >= (REPLY_MS - spin_before_ms) / 1e3
        if not spin_wait and not near:
            time.sleep(0.001)
    remaining = wake_lead_ms / 1e3 - idle_us / 1e6
    if remaining > 0:
        if spin_wait:
            until = time.monotonic() + remaining
            while time.monotonic() < until:
                ControlLatencyTrace.serialIdleUs()
        else:
            time.sleep(remaining)
    if max_hz > 0 and next_slot:
        wait_for_slot(next_slot - lead_ms / 1e3)
    return (time.monotonic() - t0) * 1e3


def wait_for_slot(target):
    """Sleep until `target`, warming the policy core just before it.

    With the fast firmware the RP2040 answers in ~8 ms, so under a 25 Hz cap
    the loop idles ~30 ms per cycle and the core cools: measured 2026-10-06,
    6.1 ms per policy step against 3.35 ms warm. Warm it WARM_MS before the
    slot, the same trick the 81 ms firmware got from timing off its reply.
    """
    if warm_ms and onboard and runner is not None and runner.ready:
        remaining = target - warm_ms / 1e3 - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        if time.monotonic() < target:
            for _ in range(warm_n):
                runner.warm()
    remaining = target - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)


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


def apply_overrides(header):
    """Per-frame knobs the trainer may set, mostly for experiments."""
    global slew_max, zero_mode, wake_lead_ms, spin_wait
    if 'spin' in header:
        spin_wait = bool(header['spin'])
    if 'warm' in header:
        global warm_ms
        warm_ms = float(header['warm'])
    if 'warm_n' in header:
        global warm_n
        warm_n = int(header['warm_n'])
    if 'prepare' in header:
        global prepare_ahead
        prepare_ahead = bool(header['prepare'])
    if 'spin_before' in header:
        global spin_before_ms
        spin_before_ms = float(header['spin_before'])
    if 'presample' in header:
        global presample_ms
        presample_ms = float(header['presample'])
    if 'trainer_presample' in header:
        global trainer_presample_ms
        trainer_presample_ms = float(header['trainer_presample'])
    if 'cpus' in header:
        # Pin the control thread. On the Pixel 3a CPUs 6-7 are the A75s.
        try:
            os.sched_setaffinity(0, set(int(c) for c in header['cpus']))
            print('dreamerBridge: control thread pinned to %s' % header['cpus'])
        except Exception as e:  # noqa: BLE001
            print('dreamerBridge: could not pin: %s' % e)
    if 'prio' in header:
        try:
            from android.os import Process
            Process.setThreadPriority(int(header['prio']))
            print('dreamerBridge: control thread priority %s' % header['prio'])
        except Exception as e:  # noqa: BLE001
            print('dreamerBridge: could not set priority: %s' % e)
    if 'bench' in header and header['bench'] and runner is not None:
        # The same benchmark as at startup, but with the sensors live.
        print('dreamerBridge: policy timing (live)  %s' % runner.benchmark())
    if 'async_motor' in header:
        ControlLatencyTrace.useAsyncMotor(bool(header['async_motor']))
    if 'slew' in header:
        slew_max = float(header['slew'])
    if 'zero' in header:
        zero_mode = str(header['zero'])
    if 'lead' in header:
        wake_lead_ms = float(header['lead'])


def zero_fix(value, previous):
    """(value, brake) for one wheel under ZERO_MODE."""
    if abs(value) >= DEAD_ZONE:
        return value, False
    if zero_mode == 'brake':
        return 0.0, True
    sign = 1.0 if previous >= 0 else -1.0
    return sign * MIN_DRIVE, False


def drive(left, right):
    """Hand an action to the wheels, and time what that costs us.

    This only reaches the serial writer's mailbox; the wheels move whenever the
    writer wins its round trip with the RP2040. `drive_ms` is therefore a floor
    on the actuation delay, not the whole of it -- see `ser_service_ms`.
    """
    global drive_ms, last_drive, cycle_ms, last_command, t_drive, t_dequeued
    # The policy acts in [-1, 1]; the trainer may cap the motors below full
    # power (env.robot.command_scale) without shrinking the policy's range.
    left, right = float(left) * cmd_scale, float(right) * cmd_scale
    requested = (left, right)
    t0 = time.monotonic()
    lb = rb = False
    if zero_mode != 'coast':
        left, lb = zero_fix(left, last_command[0])
        right, rb = zero_fix(right, last_command[1])
    if not no_base:
        context.outputs.setWheelOutput(left, right, lb, rb, float(slew_max))
    last_command = requested
    drive_ms = (time.monotonic() - t0) * 1e3
    cycle_ms = (t0 - last_drive) * 1e3 if last_drive else 0.0
    last_drive = t0
    if max_hz > 0:
        # Decisions sit on a fixed grid, so drive-to-drive is the period and
        # not the period plus the decision's own cost. Fallen a whole period
        # behind (a stall), start the grid again rather than burst.
        global next_slot
        period = 1.0 / max_hz
        if next_slot and t0 - next_slot < period:
            next_slot += period
        else:
            next_slot = t0 + period
    t_drive = t0
    t_dequeued = t0 + 0.0005  # the writer picks it up in ~0.4 ms
    # The screen is refreshed by update_gui() at its own rate; a string format
    # and a UI-thread hop do not belong between the wheels and the next step.


def stop_wheels():
    if no_base:
        return
    if context is not None and context.outputs is not None:
        context.outputs.setWheelOutput(0.0, 0.0, True, True)
        context.guiUpdater.setLastAction('stopped')


def trainer_settings():
    """The trainer address and rate cap: trainer.json if pushed, else the build."""
    settings = dict(ip=BuildConfig.IP, port=BuildConfig.PORT, max_hz=MAX_HZ)
    path = os.path.join(str(context.getExternalFilesDir(None)), 'trainer.json')
    try:
        with open(path) as f:
            settings.update(json.load(f))
    except FileNotFoundError:
        pass
    except Exception as e:
        print('dreamerBridge: ignoring %s: %s' % (path, e))
    return settings


def connect():
    """Try once to reach the trainer, without blocking the loop for long."""
    global sock, last_attempt, next_tick, pipeline, last_act, max_hz
    last_attempt = time.monotonic()
    settings = trainer_settings()
    address = (str(settings['ip']), int(settings['port']))
    try:
        candidate = socket.create_connection(address, timeout=CONNECT_TIMEOUT)
        # Reads block for much longer than the connect handshake may.
        candidate.settimeout(RECV_TIMEOUT)
        candidate.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock = candidate
        buffer.clear()
        write(dict(type='hello', protocol=PROTOCOL, control_hz=CONTROL_HZ,
                   robot_id=ROBOT_ID, onboard=bool(runner and runner.ready),
                   policy_stamp=(runner.stamp if runner else None),
                   pace=PACE, max_hz=float(settings['max_hz']),
                   # We reassemble weight pushes sent in slices.
                   wchunk=True))
        hello = read()
        if hello['type'] != 'hello' or hello['protocol'] != PROTOCOL:
            raise ValueError('Unexpected handshake %r' % hello)
        pipeline = bool(hello.get('pipeline', True))
        # Both ends must agree. We only offer it if the weights actually
        # loaded; the trainer only accepts if it is configured to record our
        # actions rather than send its own.
        global onboard, standalone, pace
        pace = hello.get('pace', PACE)
        global cmd_scale
        cmd_scale = float(hello.get('cmd_scale', 1.0))
        # A command-task trainer says how to pick commands nobody steers.
        if hello.get('command'):
            commands.configure(hello['command'])
        note_command_units(hello.get('settings'))
        max_hz = float(hello.get('max_hz', settings['max_hz']))
        if pace not in ('serial', 'clock'):
            raise ValueError('Unknown pacing %r' % pace)
        onboard = bool(hello.get('onboard', False)) and runner.ready
        if onboard:
            standalone = True
            runner.reset()
        next_tick = 0.0
        last_act = time.monotonic()
        # cycle_ms is drive-to-drive; the first one of a session would
        # otherwise measure back to whatever the previous session did last.
        global last_drive, next_slot
        last_drive = 0.0
        next_slot = 0.0
        mode = ('pipelined' if pipeline else 'lock-step') + ', ' + pace + ' paced'
        if pace == 'serial' and max_hz > 0:
            mode += ' <= %g Hz' % max_hz
        global receiver
        receiver = bridge_link.Receiver(
            sock, build=weight_saver(hello.get('run'), hello.get('settings')),
            initial=bytes(buffer), timeout=RECV_TIMEOUT)
        buffer.clear()
        if no_base:
            mode += ', no base'
        print('dreamerBridge: connected to %s:%d (%s)' % (address + (mode,)))
        context.guiUpdater.setTrainerStatus(
            'connected to %s:%d (%s)' % (address + (mode,)))
    except Exception as e:
        print('dreamerBridge: cannot reach %s:%d: %s' % (address + (e,)))
        context.guiUpdater.setTrainerStatus('no trainer at %s:%d' % address)
        disconnect()


def disconnect():
    global sock, receiver
    if receiver is not None:
        receiver.close()
        receiver = None
    try:
        sock and sock.close()
    except OSError:
        pass
    sock = None
    buffer.clear()


def write(header, blob=b''):
    header = dict(header, blob_len=len(blob))
    payload = json.dumps(header).encode('utf-8')
    sock.sendall(struct.pack('>I', len(payload)) + payload + blob)


def read():
    """Block for the next complete frame. Returns the header only."""
    if receiver is not None:
        return receiver.get()[0]
    while True:
        frame = parse()
        if frame is not None:
            return frame[0]
        fill(block=True)


def read_nowait():
    """The next complete frame if it is already here, else None.

    Returns (header, blob). A weight update spans many recv() calls, so this
    keeps pulling until the socket is drained rather than taking one 64KB
    bite per control tick -- at 50Hz that would stretch a 5MB update over
    nearly two seconds.

    Once connected the receiver thread owns the socket, and this only takes
    what it has finished.
    """
    if receiver is not None:
        return receiver.get_nowait()
    frame = parse()
    if frame is not None:
        return frame
    while fill(block=False):
        frame = parse()
        if frame is not None:
            return frame
    return None


def parse():
    """Pull one frame out of the buffer, leaving a partial one in place.

    Returns (header, blob) or None. The blob used to be skipped; it now
    carries policy weight updates, which are megabytes, so a frame is only
    complete once all of it has arrived.
    """
    if len(buffer) < 4:
        return None
    length, = struct.unpack('>I', buffer[:4])
    if len(buffer) < 4 + length:
        return None
    header = json.loads(bytes(buffer[4:4 + length]).decode('utf-8'))
    blob_len = header.get('blob_len', 0)
    total = 4 + length + blob_len
    if len(buffer) < total:
        return None
    blob = bytes(buffer[4 + length:total]) if blob_len else b''
    del buffer[:total]
    return header, blob


def fill(block):
    """Read whatever the socket has; True if any bytes arrived."""
    sock.settimeout(RECV_TIMEOUT if block else 0.0)
    try:
        chunk = sock.recv(65536)
    except (BlockingIOError, InterruptedError):
        return False
    except socket.timeout:
        raise ConnectionError('Trainer went quiet')
    finally:
        sock.settimeout(RECV_TIMEOUT)
    if not chunk:
        raise ConnectionError('Trainer closed the connection')
    buffer.extend(chunk)
    return True
