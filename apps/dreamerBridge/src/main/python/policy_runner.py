"""Phone-side wrapper around NumpyPolicy: sensors in, wheel commands out.

`numpy_policy.py` is a faithful port of the network. This is the part that
knows about the robot: it reproduces the observation scaling from
`embodied/envs/robot.py:_obs`, owns the RSSM carry across an episode, and
accepts weight updates pushed down the existing bridge socket.

The scaling constants are **not** duplicated here -- they ride in the exported
`.npz` manifest (`tools/export_policy.py`), because a policy fed inputs scaled
differently from how it was trained is wrong in a way nothing downstream can
detect. One source of truth.

Copied verbatim into the app alongside `numpy_policy.py`; numpy is the only
import beyond the standard library.
"""

import os
import tempfile
import time

import numpy as np

from numpy_policy import NumpyPolicy  # noqa: E402  (flat layout on the phone)

__all__ = ['PolicyRunner']


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
    return policy.meta

  @staticmethod
  def build_blob(blob, path):
    """Write pushed weights to `path` and parse them; the slow half of an
    update.

    Touches nothing the control loop reads, so it can run on any thread (see
    bridge_link.Receiver); `install` the result on the control thread.
    """
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

  def observe(self, sensors):
    """Sensor dict -> the observation the network expects.

    Mirrors `robot.py:_obs`. Kept as its own method so the same scaling can be
    checked against the trainer's without running the network.
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
    return {'orientation': orientation, 'wheels': wheels}

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

  def act(self, sensors):
    """One policy step. Returns (obs, action) with action as a 2-list.

    The observation is returned too because the trainer needs exactly the one
    the action was computed from -- recomputing it there from the same sensors
    would be the same numbers today and a silent divergence the first time
    either side's scaling changes.
    """
    t0 = time.monotonic()
    obs = self.observe(sensors)
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
    self.policy.finish(self.observe(sensors), self.carry, deter)
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
    obs = self.observe(sensors)
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
