"""Phone-side wrapper around NumpyPolicy: sensors in, wheel commands out.

`numpy_policy.py` is a faithful port of the network. This is the part that
knows about the robot: it reproduces the observation scaling from
`embodied/envs/robot.py:_obs`, owns the RSSM carry across an episode, and
accepts weight updates pushed down the existing bridge socket.

The scaling constants are **not** duplicated here -- they ride in the exported
`.npz` manifest (`tools/robot/dev/export_policy.py`), because a policy fed
inputs scaled differently from how it was trained is wrong in a way nothing downstream can
detect. One source of truth.

Copied verbatim into the app alongside `numpy_policy.py`; numpy is the only
import beyond the standard library.
"""

import os
import tempfile
import threading
import time

import numpy as np

from numpy_policy import NumpyPolicy, expand_delta, is_delta  # noqa: E402  (flat layout on the phone)

__all__ = ['PolicyRunner']


def travel_limit(act, pos, lo, hi, state=None):
  """Scale wheel commands that push the robot further the way it has already
  travelled: none below `lo` of leaky travel (`pos`, the mean of the two
  wheels' leaky distances, phone mm over ~pos_tau s), linearly to 0 at `hi`.
  Reaching `hi` latches: pushing on that way stays blocked until the travel
  is back under `lo` (state, a 1-list holding the blocked direction, keeps
  the latch between calls); a proportional fade alone let a policy that kept
  pushing settle into a steady crawl at hi (e30111, 0.08 m/s). Commands that
  brake or reverse pass whole, so short full-speed catches stay full while
  sustained driving stops. hi <= 0: off. A wheel command a turns its wheel
  backwards (travel -a), so it pushes on when -a has the sign of pos. The
  trainer (embodied/envs/robot.py) and the phone (deploy/policy_runner.py)
  each keep a copy, so change both."""
  if hi <= 0:
    return [float(a) for a in act]
  state = state if state is not None else [0.0]
  side = float(np.sign(pos))
  if state[0] and (abs(pos) < lo or side != state[0]):
    state[0] = 0.0
  if abs(pos) >= hi:
    state[0] = side
  if state[0]:
    k = 0.0
  elif abs(pos) <= lo:
    return [float(a) for a in act]
  else:
    k = min(1.0, max(0.0, (hi - abs(pos)) / max(hi - lo, 1e-6)))
  return [float(a) * k if -float(a) * pos > 0 else float(a) for a in act]


class Fatigue:
  """Fades a wheel command the policy holds: each wheel's output is scaled by
  1 - held/steps, so a command kept within `tol` of where it started, in the
  same direction and beyond `dead`, reaches 0 after `steps` control steps.
  Any larger change (or a reversal) restarts that wheel at full strength.
  Short bursts to catch a fall pass nearly whole; steady driving cannot last.
  steps 0: off. The trainer (embodied/envs/robot.py) and the phone
  (deploy/policy_runner.py) each keep a copy, so change both."""

  def __init__(self, steps=0, tol=0.15, dead=0.05):
    self.set(steps, tol, dead)

  def set(self, steps, tol=0.15, dead=0.05):
    self.steps, self.tol, self.dead = int(steps), float(tol), float(dead)
    self.reset()

  def reset(self):
    self.ref = [None, None]
    self.held = [0, 0]

  def __call__(self, act):
    if self.steps <= 0:
      return [float(a) for a in act]
    out = []
    for i, a in enumerate(act[:2]):
      a, ref = float(a), self.ref[i]
      if (ref is not None and abs(a) > self.dead and a * ref > 0
          and abs(a - ref) <= self.tol):
        self.held[i] += 1
      else:
        self.ref[i], self.held[i] = a, 0
      out.append(a * max(0.0, 1.0 - self.held[i] / self.steps))
    return out


class PolicyRunner:

  def __init__(self, path=None):
    self.policy = None
    self.carry = None
    self.stamp = None
    self.is_first = True
    # Whether the observation of the most recent act() opened an episode. The
    # trainer needs this flag attached to that transition, and act() has
    # already cleared is_first by the time the caller writes the message.
    self.was_first = False
    self.steps = 0
    self.last_ms = 0.0
    self.last_warm_ms = 0.0
    self.last_prepare_ms = 0.0
    self.errors = 0
    self._pre = None      # (carry, is_first, deter) from prepare(), if valid
    # 'train' samples the action, as while collecting; 'eval' takes the
    # distribution's mode, for running a finished policy (the player).
    self.mode = 'train'
    self.prepared = 0     # steps that used a prepared half
    # Emotion (deploy/emotion.py), for weights exported with emotion=True:
    # every FEEL_EVERY steps from the rewards since, empowerment on a thread
    # of its own every EMPOWER_S so the control tick never waits for it.
    self.emotion = None
    self.emotion_state = None
    self._feel = [0.0, 0, False]     # discounted reward sum, steps, a reset
    self._emo_lock = threading.Lock()
    self._emo_thread = None
    # The leaky wheel distance of a policy that observes 'position'; kept
    # across episodes and weight swaps, as the trainer keeps its own.
    self.pos = np.zeros(2, np.float64)
    self.prev_dist = None
    # Set from the trainer's pause frames; the app applies it to each action.
    self.fatigue = Fatigue()
    self.travel = (0.0, 0.0)   # (lo, hi) from the trainer; see limit()
    self._travel_latch = [0.0]
    if path and os.path.exists(path):
      self.load_path(path, stamp='initial')

  # -- weights ------------------------------------------------------------

  @property
  def ready(self):
    return self.policy is not None

  def load_path(self, path, stamp):
    return self.install(NumpyPolicy(path), stamp)

  def install(self, policy, stamp):
    """Swap in an already-parsed policy. Cheap: safe on the control thread.

    Only swap once the new weights have parsed. A half-loaded policy on a
    balancing robot is worse than slightly stale weights. The RSSM carry is
    kept across the swap -- same shapes, and resetting it mid-episode would
    hand the new weights a state they never saw at that point of an episode.
    """
    self.policy = policy
    self.stamp = stamp
    self._pre = None
    if self.carry is None:
      self.carry = policy.initial()
    self._install_emotion(policy)
    return policy.meta

  FEEL_EVERY = 4
  EMPOWER_S = 0.5

  def _install_emotion(self, policy):
    if 'emotion' not in policy.meta:
      self.emotion = None
      return
    try:
      from emotion import Emotion  # noqa: E402  (flat layout on the phone)
    except ImportError:
      return
    with self._emo_lock:
      if self.emotion is None:
        self.emotion = Emotion(policy, gamma=policy.meta['emotion']['gamma'],
                               every=0, tau=650 // self.FEEL_EVERY)
      else:
        self.emotion.p = policy   # keep the running normalisation
    if self._emo_thread is None:
      self._emo_thread = threading.Thread(target=self._empower_loop, daemon=True)
      self._emo_thread.start()

  def _empower_loop(self):
    while True:
      time.sleep(self.EMPOWER_S)
      emo, last = self.emotion, getattr(self.policy, 'last_post', None)
      if emo is None or last is None:
        continue
      try:
        value = emo._empowerment(last[0], last[1])
        with self._emo_lock:
          emo.empowerment = value
      except Exception as e:  # noqa: BLE001 -- the face is never worth a crash
        print('dreamerBridge: empowerment failed: %s' % e)

  def feel(self, reward):
    """After each step, with the reward it scored: updates emotion_state."""
    emo = self.emotion
    if emo is None or getattr(self.policy, 'last_post', None) is None:
      return
    acc = self._feel
    acc[0] += (emo.gamma ** acc[1]) * float(reward)
    acc[1] += 1
    acc[2] = acc[2] or self.was_first
    if acc[1] < self.FEEL_EVERY:
      return
    deter, stoch, logit = self.policy.last_post
    try:
      with self._emo_lock:
        sig = emo.step(deter, logit, stoch, acc[0], is_first=acc[2],
                       steps=acc[1])
      self.emotion_state = dict(
          valence=sig['face_valence'], arousal=sig['arousal'],
          dominance=sig['dominance'], td=sig['td'], value=sig['value'],
          surprise=sig['surprise'], unexpected=sig['unexpected'],
          novelty=sig['novelty'], empowerment=sig['empowerment'])
    except Exception as e:  # noqa: BLE001
      print('dreamerBridge: emotion failed: %s' % e)
      self.emotion = None
    self._feel = [0.0, 0, False]

  @staticmethod
  def build_blob(blob, path):
    """Write pushed weights to `path` and parse them; the slow half of an
    update.

    Touches nothing the control loop reads, so it can run on any thread (see
    bridge_link.Receiver); `install` the result on the control thread.

    A delta push (export.encode with a base) is rebuilt first from the push
    stored at `path`, which it must name as its base; otherwise this raises
    and the trainer, told so, sends the next push whole.
    """
    if is_delta(blob):
      blob = expand_delta(blob, path)
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(suffix='.npz', dir=directory)
    with os.fdopen(fd, 'wb') as f:
      f.write(blob)
    os.replace(tmp, path)
    return NumpyPolicy(path)

  def load_blob(self, blob, stamp, directory):
    """Install weights pushed by the trainer. Returns True if they took."""
    try:
      self.install(self.build_blob(
          blob, os.path.join(directory, 'policy.npz')), stamp)
      return True
    except Exception as e:  # noqa: BLE001 -- never let this stop the robot
      self.errors += 1
      print('dreamerBridge: bad weight update %s: %s' % (stamp, e))
      return False

  # -- acting -------------------------------------------------------------

  def reset(self):
    """Start a new episode: drop the RSSM carry and flag the next step."""
    if self.policy is not None:
      self.carry = self.policy.initial()
    self.is_first = True
    self._pre = None
    self.fatigue.reset()

  def observe(self, sensors, command=None, track=True):
    """Sensor dict -> the observation the network expects.

    Mirrors `robot.py:_obs`. Kept as its own method so the same scaling can be
    checked against the trainer's without running the network. `command` is
    (forward, turn) in [-1, 1], used only by a policy trained on the command
    task (its manifest lists a 'command' key); None is a centred stick.
    """
    s = self.policy.meta['obs_scale']
    theta = float(sensors['theta'])
    clip = s['obs_clip']
    orientation = np.clip(np.array([
        (theta - s['theta_zero']) / s['obs_theta_scale'],
        float(sensors['angular_velocity']) / s['obs_rate_scale'],
    ], np.float32), -clip, clip)
    wheels = np.clip(np.array([
        float(sensors['wheel_speed_l']), float(sensors['wheel_speed_r']),
    ], np.float32) * s['obs_wheel_scale'], -clip, clip)
    obs = {'orientation': orientation, 'wheels': wheels}
    if 'command' in self.policy.meta['obs_keys']:
      obs['command'] = np.clip(np.asarray(
          command if command is not None else (0.0, 0.0), np.float32), -1, 1)
    if 'position' in self.policy.meta['obs_keys'] and not track:
      # warm() and benchmark() pass made-up sensors with no wheel distances:
      # tracking those threw away the next real step's travel as a counter
      # jump and leaked an extra step, so the phone's position (its policy's
      # input and the travel limiter's) read ~1/3 of the trainer's (e30112).
      obs['position'] = np.clip(
          self.pos * s['obs_pos_scale'], -clip, clip).astype(np.float32)
    elif 'position' in self.policy.meta['obs_keys']:
      # Mirrors robot.py:track_position, one step per observation.
      dist = np.array([float(sensors.get('wheel_distance_l', 0.0)),
                       float(sensors.get('wheel_distance_r', 0.0))])
      step = (np.zeros(2) if self.prev_dist is None
              else dist - self.prev_dist)
      step[np.abs(step) > s['pos_jump']] = 0.0
      self.pos = self.pos * np.exp(-1.0 / (s['pos_tau'] * s['hz'])) + step
      self.prev_dist = dist
      obs['position'] = np.clip(
          self.pos * s['obs_pos_scale'], -clip, clip).astype(np.float32)
    return obs

  def limit(self, act):
    """The trainer's limits on the action act() chose: fatigue, then travel."""
    act = self.fatigue(act)
    lo, hi = self.travel
    if hi > 0:
      act = travel_limit(act, float(self.pos.mean()), lo, hi, self._travel_latch)
    return act

  def applied(self, act):
    """The wheels got `act`, not the action act() chose (clipped by the
    trainer's cap, held by a pause or a brake, or the trainer's own drive).
    The next step's RSSM input is what really happened, as in replay."""
    if self.carry is not None:
      self.carry = dict(self.carry, prevact=np.asarray(act, np.float32))
      self._pre = None

  def prepare(self):
    """Run the observation-independent half of the next step now.

    Call it in the slack after an action has been applied. `act` then only
    has the encode/posterior/actor half left when the observation lands --
    roughly half the time and half the weights, on a core that is cold by
    then anyway.
    """
    if self.policy is None:
      return
    t0 = time.monotonic()
    self._pre = (self.carry, self.is_first,
                 self.policy.precompute(self.carry, self.is_first))
    self.last_prepare_ms = (time.monotonic() - t0) * 1e3

  def act(self, sensors, command=None):
    """One policy step. Returns (obs, action) with action as a 2-list.

    The observation is returned too because the trainer needs exactly the one
    the action was computed from -- recomputing it there from the same sensors
    would be the same numbers today and a silent divergence the first time
    either side's scaling changes.
    """
    t0 = time.monotonic()
    obs = self.observe(sensors, command)
    self.was_first = self.is_first
    pre = self._pre
    self._pre = None
    if pre is not None and pre[0] is self.carry and pre[1] == self.is_first:
      deter = pre[2]
      self.prepared += 1
    else:
      deter = self.policy.precompute(self.carry, self.is_first)
    act, self.carry = self.policy.finish(obs, self.carry, deter, mode=self.mode)
    self.is_first = False
    self.steps += 1
    self.last_ms = (time.monotonic() - t0) * 1e3
    return obs, [float(act[0]), float(act[1])]

  def warm(self):
    """A throwaway step on the current carry, to bring the core up to speed
    before the real one. Nothing is kept: the carry is not advanced and the
    result is dropped. Only the RNG moves, which is harmless."""
    t0 = time.monotonic()
    sensors = dict(theta=0.0, angular_velocity=0.0,
                   wheel_speed_l=0.0, wheel_speed_r=0.0)
    pre = self._pre
    deter = pre[2] if pre is not None else self.policy.precompute(self.carry)
    self.policy.finish(self.observe(sensors, track=False), self.carry, deter)
    self.last_warm_ms = (time.monotonic() - t0) * 1e3

  # -- diagnostics --------------------------------------------------------

  def benchmark(self, n=30):
    """Time each stage on this device and return a one-line summary.

    Run at startup because the phone is 50-100x slower than the workstation
    the port was written on, and which stage dominates is not predictable from
    there: the first on-robot measurement (2026-09-07) came in at 16.7ms per
    step against a 20ms control period, which is the difference between a 50Hz
    robot and a 28Hz one.
    """
    import numpy as _np
    sensors = dict(theta=0.02, angular_velocity=0.1,
                   wheel_speed_l=10.0, wheel_speed_r=-10.0)
    obs = self.observe(sensors, track=False)
    carry = self.policy.initial()
    p = self.policy
    # Warm up: first touch of each array pays page-in and any lazy dispatch.
    for _ in range(5):
      p.step(obs, carry, is_first=False)

    def timeit(fn):
      t0 = time.monotonic()
      for _ in range(n):
        fn()
      return (time.monotonic() - t0) / n * 1e3

    tokens = p._encode(obs)
    deter = p._core(carry['deter'], carry['stoch'], _np.zeros(2, _np.float32))
    stoch, _ = p._obs_stoch(deter, tokens)
    parts = [
        ('encode', lambda: p._encode(obs)),
        ('core', lambda: p._core(
            carry['deter'], carry['stoch'], _np.zeros(2, _np.float32))),
        ('stoch', lambda: p._obs_stoch(deter, tokens)),
        ('actor', lambda: p._actor(deter, stoch, 'train')),
        ('total', lambda: p.step(obs, carry, is_first=False)),
    ]
    return '  '.join(f'{name} {timeit(fn):.2f}ms' for name, fn in parts)
