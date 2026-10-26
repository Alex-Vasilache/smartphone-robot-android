"""Phone side of the DreamerV3 bridge.

Two ways to run, settled at handshake. Onboard (what the dreamerv3 robot
tools use, see loop_onboard): the policy runs here on weights the trainer
pushes in the background, and every observation is streamed back to train
on. Remote: the policy runs on the training computer; each control step this
client applies an action to the wheels, waits out the control period, then
reports the proprio sensors it collected. Either way the phone owns the
control clock.

In remote mode the handshake reply picks how tightly the two ends are coupled:

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
`tools/robot/dev/fake_robot_phone.py` there is the same protocol without a robot, useful
for testing the computer side on its own.

Wire format, both directions: a 4-byte big-endian header length, a UTF-8 JSON
header, then `blob_len` bytes of binary payload. The blob is unused while the
observation is proprio only; it is where camera frames go.

The trainer address defaults to `config.json` at the repository root (copy
`config.template.json`), compiled into `BuildConfig.IP`/`PORT`. A
`trainer.json` pushed with adb overrides it without a rebuild, and is reread
on every connection attempt:

    adb push trainer.json /sdcard/Android/data/jp.oist.abcvlib.dreamerBridge/files/

with `{"ip": "<learner address>", "port": 3000, "max_hz": 25}`; every key is
optional.
"""

import json
import math
import os
import socket
import struct
import sys
import threading
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
import dashboard
import webui

PROTOCOL = 3
CONTROL_HZ = 50.0
# The GUI is for a human watching the robot, so it does not need to keep up
# with the control loop. Above ~10 Hz the runOnUiThread hop starts costing
# more than the control step itself and shows up as jitter at the trainer.
GUI_PERIOD = 0.1
# The web page (webui.py) gets its own, faster refresh: it is pushed over a
# socket on another thread, so it costs the loop only building a small dict.
# Every control step at 65 Hz: at 25 Hz the dashboard's robot drawing was a
# visible beat behind the robot, ~20 ms of it waiting for the next frame.
WEB_PERIOD = 0.015
RECV_TIMEOUT = 30.0
# Connecting must fail fast. The control loop is single threaded, so a blocking
# connect freezes sensing and the display with it -- which looks exactly like a
# stale sensor. A refused connection returns immediately; this only bounds the
# case where the trainer's host swallows the SYN.
CONNECT_TIMEOUT = 0.5
# A refused connection costs nothing, and after a dashboard restart this is
# most of the time the robot waits for the next session.
RETRY_DELAY = 0.5
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
# Sign of the wheel *speed* that is forward (robot.py forward_sign) until a
# trainer says otherwise (its command_speed carries the sign): the robot's
# positive wheel speed is backward (e1302).
FORWARD_SIGN = -1.0
# Direct drive changes a motor output by at most this much per second, so a
# flick of the stick from full forward to full back takes 0.5 s. Reversing
# at once from full speed (2026-10-15) left the base answering in ~95 ms
# with the battery and encoders reading nothing until it was power-cycled.
DIRECT_RAMP = 4.0
# Every motor output, from any source, is kept within CURRENT_GAP of the
# output that would hold the wheel at its present speed. Motor current goes
# with that gap (drive voltage minus back-EMF), so this caps it at what a
# start from rest draws: from rest any output is allowed, but a full-speed
# wheel cannot be thrown into full reverse (a gap of ~2), which twice
# faulted the base on 2026-10-15 -- once in direct drive and once while
# training e1460 -- leaving it answering slowly with the battery and encoders
# reading 0 until power-cycled. Braking is still allowed at once down to
# zero output from full speed; reversing then follows the wheel down.
CURRENT_GAP = 1.0
# Wheel speed (mm/s) at full output, wheels in the air (2026-10-15); a
# positive output reads a negative speed.
WHEEL_SPEED_MAX = 5700.0
direct_out = (0.0, 0.0)
direct_at = 0.0
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
# diverges with a ~50ms time constant. See docs/robot/ONDEVICE_POLICY.md in dreamerv3.
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
# its <name>.json sidecar (tools/robot/save_policy.sh) gives the rate, episode length
# and reward settings, so the screen scores steps the way training did.
player_policy = None
player_eval = True
player = None      # the sidecar dict once loaded
# How to score steps when no trainer does (idle, standalone, direct drive):
# the last trainer's settings, kept in reward_settings.json across restarts,
# else these -- robot.py's defaults with robot_command_taper_sum on top.
DEFAULT_REWARD = dict(
    task='command', length=500, theta_zero=0.0, theta_lo=-0.122,
    theta_hi=0.182, theta_sigma=0.05, speed_scale=1e-4, drift_penalty=0.1,
    drift_clip=1.0, wheel_penalty=0.0, rate_penalty=0.05,
    action_rate_penalty=0.0, command_speed=4900.0, command_turn=4900.0,
    command_sigma=0.4, command_reward='taper_sum', forward_sign=-1.0,
    bumper_lo=-0.120, bumper_hi=0.161, bumper_margin=0.0175)
reward_settings = dict(DEFAULT_REWARD)
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
# Share of each motor output that is the command's own feed-forward drive
# (command_motors) rather than the policy's action; the trainer's hello or
# the player's sidecar sets it. 0: the policy alone, as before.
command_bias = 0.0
# Friction compensation of policy outputs (friction_comp in the trainer's
# hello or the player's sidecar). Measured 2026-10-15: a still wheel starts
# only at ~0.3 output and then jumps to ~1400 mm/s; a turning one keeps
# turning down to ~0.12. So an action a becomes sign(a) * (FRICTION_KINETIC
# + (1 - FRICTION_KINETIC) * |a|), and a wheel that is still while the action
# asks it to move gets FRICTION_BREAKAWAY for one step to unstick it -- not
# two in a row, so a wheel held against something is not driven there.
friction_comp = False
FRICTION_KINETIC = 0.12
FRICTION_BREAKAWAY = 0.32
FRICTION_STILL = 100.0   # mm/s: below this a wheel counts as stuck
FRICTION_ZERO = 0.02     # actions smaller than this stay zero
kicked = [False, False]
weights_installed = 0  # weight pushes swapped in this session
# Training from scratch, asked for on the dashboard (webui.py) and offered by
# a trainer under tools/robot/demo_supervisor.py ('restart' in its hello).
# The web thread sets the event; the control loop sends the request.
trainer_restart = False
restart_asked = threading.Event()
# Wheels held still until the trainer's next weights land: from a restart
# request, and whenever a trainer says the weights we hold are not its run's
# ('hold' in its hello). Otherwise a fresh run would start out showing the
# skill of the run before it.
hold_weights = False
# Pausing the robot (wheels held) or the training (learner idle) from the
# dashboard, offered by a trainer that sends 'pause' in its hello. The trainer
# owns both and answers every request with the state; the robot one is
# applied here the moment it is asked for, so the wheels stop at once.
can_pause = False
robot_paused = False
train_paused = False
pause_lock = threading.Lock()
pause_asked = {}   # web thread -> control loop: {'robot'/'training': bool}
# This run's reward curve, weight updates and learner metrics, for the web
# dashboard (dashboard.py); the live numbers come from web_state.
history = dashboard.RunHistory()
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
# After the base stops answering (USB dropped, RP2040 reset, I2C stuck), the
# wheels are held at zero until it has answered cleanly for BASE_HOLD_S, and
# every observation says base_ok False until then so the trainer can end the
# episode instead of learning from frozen sensors (2026-10-16).
BASE_HOLD_S = 2.0
base_ok_from = 0.0        # monotonic time from which the base counts as healthy
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
    web = webui.WebUI(commands, history=history.snapshot,
                      on_restart=request_restart, on_pause=request_pause)
    web_url = web.start() or ''
    load_reward_settings()
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
    global command_bias, friction_comp
    command_bias = float(player.get('command_bias', 0.0))
    friction_comp = bool(player.get('friction_comp', False))
    # Nobody needs random targets on a finished policy: it balances until the
    # joystick says otherwise.
    commands.set_mode('manual')
    note_command_units(player)
    pace = PACE
    print('dreamerBridge: player running %s at %g Hz (%s actions)' % (
        player['name'], max_hz, runner.mode))


def reward_settings_path():
    return os.path.join(
        str(context.getExternalFilesDir(None)), 'reward_settings.json')


def load_reward_settings():
    global reward_settings
    try:
        with open(reward_settings_path()) as f:
            reward_settings = dict(DEFAULT_REWARD, **json.load(f))
    except Exception:  # noqa: BLE001 -- no trainer has connected yet
        reward_settings = dict(DEFAULT_REWARD)
    note_command_units(reward_settings)


def save_reward_settings(settings):
    """Keep a trainer's reward settings, to score with after it is gone."""
    global reward_settings
    if not settings or 'theta_zero' not in settings:
        return
    reward_settings = dict(DEFAULT_REWARD, **settings)
    try:
        tmp = reward_settings_path() + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(reward_settings, f)
        os.replace(tmp, reward_settings_path())
    except Exception as e:  # noqa: BLE001 -- display only
        print('dreamerBridge: could not save reward settings: %s' % e)


def score_locally(snap, act, cmd):
    """Show the training reward for a step no trainer scored."""
    note_reward(player_reward(snap, act, cmd))
    p = player if player is not None else reward_settings
    if ep_steps >= int(p.get('length', 500)):
        end_episode()


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
    p = player if player is not None else reward_settings
    if 'theta_zero' not in p:
        return 0.0
    offset = float(sensors['theta']) - p['theta_zero']
    reach = p['theta_hi'] if offset > 0 else abs(p['theta_lo'])
    linear = 1.0 - min(1.0, abs(offset) / max(reach, 1e-6))
    bonus = math.exp(-(offset / p['theta_sigma']) ** 2)
    left = float(sensors['wheel_speed_l'])
    right = float(sensors['wheel_speed_r'])
    balance = 0.5 * linear + 0.5 * bonus
    if p.get('task') == 'command' and command_units:
        # _command_reward: tracking is a Gaussian on the 2D (forward, turn)
        # error; only the tilt-rate penalty is kept. Turn is read from the
        # wheels whatever the run's turn_source.
        speed, turn = command_units
        sigma = float(p.get('command_sigma', 0.3))
        forward = 0.5 * (left + right) / speed
        spin = 0.5 * (left - right) / turn
        # As robot.py: no wheel asked for more than full forward gives it.
        k = max(1.0, abs(cmd[0]) + abs(cmd[1] * turn / speed))
        err = math.hypot(forward - cmd[0] / k, spin - cmd[1] / k)
        track = math.exp(-(err / sigma) ** 2)
        mode = p.get('command_reward', 'sum')
        if mode == 'balance':
            score = balance
        elif mode == 'bumper_track':
            margin = float(p.get('bumper_margin', 0.0175))
            near = (offset >= float(p.get('bumper_hi', 0.161)) - margin or
                    offset <= float(p.get('bumper_lo', -0.120)) + margin)
            score = -float(p.get('bumper_penalty', 1.0)) if near else track
        elif mode == 'bumper':
            margin = float(p.get('bumper_margin', 0.0175))
            near = (offset >= float(p.get('bumper_hi', 0.161)) - margin or
                    offset <= float(p.get('bumper_lo', -0.120)) + margin)
            still = abs(cmd[0]) < 1e-3 and abs(cmd[1]) < 1e-3
            score = -1.0 if near else (
                0.5 * balance + 0.5 * track if still else track)
        elif mode == 'gated':
            score = balance * (0.5 + 0.5 * track)
        elif mode == 'product':
            score = balance * track
        elif mode == 'taper_sum':
            w = 0.1 + 0.8 * min(1.0, math.hypot(cmd[0], cmd[1]))
            score = (1.0 - w) * balance + w * track
        else:
            # 'sum', and 'curriculum' as if its ramp were done; sidecars
            # without command_reward (e1299 and older) are 'sum'.
            score = (float(p.get('command_balance_weight', 0.5)) * balance
                     + float(p.get('command_track_weight', 0.5)) * track)
        reward = score - p['rate_penalty'] * abs(
            float(sensors['angular_velocity']))
    else:
        left *= p['speed_scale']
        right *= p['speed_scale']
        clip = p['drift_clip']
        reward = (balance
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
    policy_drive(act[0], act[1])
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
    block['base_ok'] = base_ok()
    return block


def loop():
    """One control step, or one reconnect attempt if we are not connected."""
    global step, next_tick, last_act
    global direct_out, direct_at
    if commands.settings['mode'] == 'direct':
        return loop_direct()
    direct_out, direct_at = (0.0, 0.0), 0.0
    if player is not None:
        return loop_player()
    if sock is None:
        if standalone and runner is not None and runner.ready:
            # No trainer, but we still have the policy that was balancing a
            # moment ago. Keep the loop closed and retry the connection between
            # ticks; experience collected now is simply not recorded.
            wait_for_pace()
            snap, _, _ = sample()
            cmd = commands.tick()
            _, act = runner.act(snap, cmd)
            if hold_weights or robot_paused:
                act = [0.0, 0.0]
                stop_wheels()
            else:
                policy_drive(act[0], act[1])
            score_locally(snap, act, cmd)
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
        snap, _, _ = sample()
        score_locally(snap, (0.0, 0.0), commands.tick())
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
                if header.get('type') == 'stats':
                    history.stats(header.get('stats'))
                    continue
                if header.get('type') == 'pause':
                    apply_pause(header)
                    continue
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
            while act.get('type') == 'pause':
                apply_pause(act)
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
                policy_drive(act['left'], act['right'])
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
        send_pause()
    except Exception as e:
        print('dreamerBridge: lost the trainer at step %d: %s' % (step, e))
        context.guiUpdater.setTrainerStatus('disconnected (%s)' % e)
        if not (standalone and runner is not None and runner.ready):
            stop_wheels()
        disconnect()
    finally:
        update_gui()


def command_motors(f, t):
    """The motor outputs that drive the robot as command (f, t) asks.

    Toward the wheel speeds the trainer would ask for the same command; a
    motor output and its wheel speed have opposite signs (measured
    2026-10-15: output -1 read about +4700), hence the minus. The pair is
    scaled so the busier motor is at most 1, keeping the heading.
    """
    sign = math.copysign(1.0, command_units[0]) if command_units \
        else FORWARD_SIGN
    left, right = -(sign * f + t), -(sign * f - t)
    n = max(1.0, abs(left), abs(right))
    return left / n, right / n


def compensate(action, speed, wheel):
    """One wheel's output for `action` under friction compensation."""
    a = float(action)
    if abs(a) < FRICTION_ZERO:
        kicked[wheel] = False
        return 0.0
    out = math.copysign(FRICTION_KINETIC + (1 - FRICTION_KINETIC) * abs(a), a)
    still = abs(float(speed)) < FRICTION_STILL
    if still and not kicked[wheel] and abs(out) < FRICTION_BREAKAWAY:
        kicked[wheel] = True
        return math.copysign(FRICTION_BREAKAWAY, a)
    kicked[wheel] = False
    return out


def policy_drive(left, right):
    """Drive a policy's action: mixed with the command's own drive, then
    compensated for friction, as the run asks."""
    if command_bias:
        bl, br = command_motors(*commands.current)
        left = (1.0 - command_bias) * float(left) + command_bias * bl
        right = (1.0 - command_bias) * float(right) + command_bias * br
    if friction_comp:
        left = compensate(left, sensors['wheel_speed_l'], 0)
        right = compensate(right, sensors['wheel_speed_r'], 1)
    w = commands.blend
    if w > 0:
        # 'blend' mode: the stick's own motor outputs, as in direct drive (not
        # multiplied by command_scale), take share w of the policy's.
        sl, sr = command_motors(*commands.current)
        left = (1.0 - w) * float(left) * cmd_scale + w * sl
        right = (1.0 - w) * float(right) * cmd_scale + w * sr
        drive(left, right, scale=1.0)
        return
    drive(left, right)


def loop_direct():
    """One step of the joystick driving the motors itself, no policy.

    For checking what a command asks is possible at all: full forward is both
    motors at full output, full turn is +-1 on the two, and in between the
    pair is scaled so the busier motor is at most 1 (the heading is kept).
    The outputs are command_motors(): what the trainer would ask for.
    The output is not multiplied by the trainer's command_scale. A trainer
    link is dropped on entry and not retried until the mode changes back.
    """
    if sock is not None:
        print('dreamerBridge: direct drive, dropping the trainer')
        stop_wheels()
        disconnect()
    context.guiUpdater.setTrainerStatus('direct drive (no policy)')
    wait_for_pace()
    snap, _, _ = sample()
    f, t = commands.tick()
    left, right = command_motors(f, t)
    global direct_out, direct_at
    # Ramp from the start too: direct_out is zeroed in any other mode.
    now = time.monotonic()
    room = DIRECT_RAMP * min(now - direct_at, 0.1) if direct_at else 0.0
    direct_at = now
    direct_out = tuple(
        old + max(-room, min(room, new - old))
        for old, new in zip(direct_out, (left, right)))
    drive(direct_out[0], direct_out[1], scale=1.0)
    score_locally(snap, direct_out, (f, t))
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
    history.reward(reward)
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
    if sock is None and player is None and \
            commands.settings['mode'] != 'direct':
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
        run=history.run, t0=history.t0, now=time.time(),
        policy=str(runner.stamp) if runner is not None else None,
        weights_installed=weights_installed, stats=history.latest,
        can_restart=bool(trainer_restart and sock is not None and onboard),
        holding=hold_weights,
        can_pause=bool(can_pause and sock is not None),
        robot_paused=robot_paused, train_paused=train_paused,
        train_started=history.train_started, waiting=history.waiting,
        grid=0.0 if commands.settings['mode'] == 'blend'
        else float(commands.settings.get('step') or 0.0))


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
    if hold_weights or robot_paused:
        # Recorded as what it is: a step with the wheels held.
        act = [0.0, 0.0]
        stop_wheels()
    else:
        policy_drive(act[0], act[1])
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
    if restart_asked.is_set():
        send_restart()
    send_pause()
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
    send_pause()

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
            if kind == 'stats':
                history.stats(header.get('stats'))
            elif kind == 'pause':
                apply_pause(header)
            elif kind == 'weights_ready':
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
            policy_drive(act['left'], act['right'])
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
    global step, weights_installed, hold_weights
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
            history.weights(stamp)
            hold_weights = False
            context.guiUpdater.setTrainerStatus('policy %s' % stamp)
        elif kind == 'stats':
            # The learner's latest metrics, forwarded by the trainer for the
            # web dashboard only.
            history.stats(header.get('stats'))
        elif kind == 'pause':
            apply_pause(header)
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


def request_restart():
    """From the web page's thread: ask for training from scratch. Only a
    trainer that offered it is asked; the control loop sends it."""
    if not (trainer_restart and sock is not None and onboard):
        raise ValueError('this trainer cannot restart training')
    restart_asked.set()


def send_restart():
    """Ask the trainer for a fresh run and stop acting on this one's weights.

    The link stays up: the supervisor ends this session, which drops it, and
    only then do we reconnect -- to the next session, not back to this one.
    """
    global hold_weights, reward_avg
    restart_asked.clear()
    write(dict(type='restart'))
    hold_weights = True
    end_episode()
    reward_avg = 0.0
    print('dreamerBridge: asked the trainer for a fresh run')
    context.guiUpdater.setTrainerStatus('restarting training')


def request_pause(robot=None, training=None):
    """From the web page's thread: pause (True) or start (False) the robot
    and/or the training. The control loop sends it; the robot is held or let
    go here at once rather than a round trip later."""
    global robot_paused
    if not (can_pause and sock is not None):
        raise ValueError('this trainer cannot pause')
    with pause_lock:
        if robot is not None:
            pause_asked['robot'] = robot_paused = bool(robot)
        if training is not None:
            pause_asked['training'] = bool(training)


def send_pause():
    """Pass the dashboard's pause requests on to the trainer."""
    if not pause_asked:
        return
    with pause_lock:
        asked = dict(pause_asked)
        pause_asked.clear()
    write(dict(type='pause', **asked))


def apply_pause(state):
    """The trainer's pause state, from its hello or an answer to a request.
    A trainer that does not offer pausing (state None) pauses nothing."""
    global robot_paused, train_paused
    state = state or {}
    was = (robot_paused, train_paused)
    robot_paused = bool(state.get('robot', False))
    train_paused = bool(state.get('training', False))
    history.training(state.get('train_started_ago'),
                     state.get('waiting', False))
    if (robot_paused, train_paused) != was:
        print('dreamerBridge: robot %s, training %s' % (
            'paused' if robot_paused else 'running',
            'paused' if train_paused else 'running'))


def weights_dir():
    return os.path.join(str(context.getFilesDir()), 'dreamer_policy')


def policies_dir():
    """Saved policies, one <name>.npz + <name>.json each: what the policy
    list shows. Each training run keeps its latest weights here under the
    run's name; tools/robot/save_policy.sh adds others."""
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


def base_lost():
    """The base is not answering: hold the wheels from now until it has
    answered cleanly for BASE_HOLD_S."""
    global base_ok_from
    base_ok_from = time.monotonic() + BASE_HOLD_S


def base_ok():
    return no_base or time.monotonic() >= base_ok_from


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
            base_lost()
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


def limit_current(output, speed):
    """`output`, moved to within CURRENT_GAP of the wheel's own speed."""
    held = -float(speed) / WHEEL_SPEED_MAX
    return max(held - CURRENT_GAP, min(held + CURRENT_GAP, output))


def drive(left, right, scale=None):
    """Hand an action to the wheels, and time what that costs us.

    This only reaches the serial writer's mailbox; the wheels move whenever the
    writer wins its round trip with the RP2040. `drive_ms` is therefore a floor
    on the actuation delay, not the whole of it -- see `ser_service_ms`.
    """
    global drive_ms, last_drive, cycle_ms, last_command, t_drive, t_dequeued
    # The policy acts in [-1, 1]; the trainer may cap the motors below full
    # power (env.robot.command_scale) without shrinking the policy's range.
    scale = cmd_scale if scale is None else scale
    left, right = float(left) * scale, float(right) * scale
    if not base_ok():
        # Just back from an outage: whatever the policy wants, the first thing
        # the base gets is a stop, until it has answered for BASE_HOLD_S.
        left, right = 0.0, 0.0
    # A non-finite output would pass every clamp below as a full one.
    if not (math.isfinite(left) and math.isfinite(right)):
        print('dreamerBridge: non-finite wheel command (%r, %r); stopping'
              % (left, right))
        left, right = 0.0, 0.0
    if not no_base:
        left = limit_current(left, sensors['wheel_speed_l'])
        right = limit_current(right, sensors['wheel_speed_r'])
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
        global command_bias, friction_comp
        command_bias = float(hello.get('command_bias', 0.0))
        friction_comp = bool(hello.get('friction_comp', False))
        # A command-task trainer says how to pick commands nobody steers.
        if hello.get('command'):
            commands.configure(hello['command'])
        note_command_units(hello.get('settings'))
        save_reward_settings(hello.get('settings'))
        global trainer_restart, hold_weights, weights_installed
        if (hello.get('run') or 'robot') != history.run:
            weights_installed = 0   # the dashboard counts them per run
        history.start(hello.get('run'))
        end_episode()  # the idle steps scored here are not this run's
        trainer_restart = bool(hello.get('restart', False))
        hold_weights = bool(hello.get('hold', False))
        restart_asked.clear()
        global can_pause
        can_pause = hello.get('pause') is not None
        with pause_lock:
            pause_asked.clear()
        apply_pause(hello.get('pause'))
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
