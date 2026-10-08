"""The command the policy follows: (forward, turn), each in [-1, 1].

Only a policy trained on the command task (env.robot task 'command' in the
dreamerv3 repo) reads it; the others ignore it. It is chosen here, on the
phone, because the phone runs the policy: the command must be in the
observation the instant the policy acts, and it goes to the trainer with every
observation so the reward there is scored against the same one.

Where it comes from, in order:

* the joystick on the web page (webui.py), while someone is touching it --
  each move posts a command and it holds for JOYSTICK_HOLD seconds;
* otherwise, by mode:
  - 'auto': a random command, held for hold_min..hold_max seconds. Zero with
    p_zero, one axis only with p_axis, both axes otherwise. This is what
    training runs on; nobody can steer for an hour.
  - 'manual': zero, which is "balance in place".

The trainer's handshake sets the sampler (`command` in its hello), the web
page can switch the mode at any time, and the player starts in 'manual'.
"""

import random
import threading
import time

# A joystick that stops posting is let go after this long, so a phone that
# drops off the WiFi mid-drive cannot leave the robot driving.
JOYSTICK_HOLD = 0.5

DEFAULTS = dict(mode='manual', hold_min=2.0, hold_max=5.0, p_zero=0.3,
                p_axis=0.4)


class CommandSource:

    def __init__(self, seed=None):
        self._lock = threading.Lock()
        self._rng = random.Random(seed)
        self.settings = dict(DEFAULTS)
        self._auto = (0.0, 0.0)
        self._auto_until = 0.0
        self._stick = (0.0, 0.0)
        self._stick_at = -1e9
        self.current = (0.0, 0.0)
        self.source = 'idle'

    def configure(self, settings):
        """Sampler settings from the trainer's hello; missing keys keep theirs."""
        with self._lock:
            for key in DEFAULTS:
                if settings and key in settings:
                    self.settings[key] = settings[key]
            self._auto_until = 0.0

    def set_mode(self, mode):
        if mode not in ('auto', 'manual'):
            raise ValueError('mode must be auto or manual, not %r' % mode)
        with self._lock:
            self.settings['mode'] = mode
            self._auto_until = 0.0

    def joystick(self, forward, turn):
        """A joystick position, from the web server's thread."""
        forward = max(-1.0, min(1.0, float(forward)))
        turn = max(-1.0, min(1.0, float(turn)))
        with self._lock:
            self._stick = (forward, turn)
            self._stick_at = time.monotonic()

    def release(self):
        with self._lock:
            self._stick_at = -1e9

    def tick(self):
        """The command for this control step, as [forward, turn]."""
        now = time.monotonic()
        with self._lock:
            if now - self._stick_at < JOYSTICK_HOLD:
                cmd, src = self._stick, 'joystick'
            elif self.settings['mode'] == 'auto':
                if now >= self._auto_until:
                    self._auto = self._sample()
                    s = self.settings
                    self._auto_until = now + self._rng.uniform(
                        float(s['hold_min']), float(s['hold_max']))
                cmd, src = self._auto, 'auto'
            else:
                cmd, src = (0.0, 0.0), 'idle'
        self.current, self.source = cmd, src
        return [cmd[0], cmd[1]]

    def _sample(self):
        s, u = self.settings, self._rng.uniform
        r = self._rng.random()
        if r < float(s['p_zero']):
            return (0.0, 0.0)
        if r < float(s['p_zero']) + float(s['p_axis']):
            if self._rng.random() < 0.5:
                return (u(-1.0, 1.0), 0.0)
            return (0.0, u(-1.0, 1.0))
        return (u(-1.0, 1.0), u(-1.0, 1.0))
