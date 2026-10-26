"""What the web dashboard (/dash) needs beyond the live state: this run's past.

The page streams the live state over the WebSocket, but it may be opened an
hour into a run and must still draw the whole reward curve. So the phone
keeps, per training run, the reward averaged over each second, every weight
install with how old those weights were, and the learner's metrics as the
trainer forwards them. Small: a two-hour run is ~7200 reward bins.

Written by the control loop, read by the web server's threads, so every
access takes the lock and `snapshot` hands out copies.
"""

import threading
import time

MAX_POINTS = 6 * 3600   # six hours of one-second bins


class RunHistory:

    def __init__(self):
        self._lock = threading.Lock()
        self.run = None
        self.t0 = time.time()
        self.latest = {}       # the learner's last forwarded metrics
        # When training started (this clock), for a run started paused; the
        # page draws that moment as minute 0. None: since the run started,
        # or, with `waiting`, not yet (the page counts back from now).
        self.train_started = None
        self.waiting = False
        self._reset()

    def _reset(self):
        self._rewards = []     # [seconds since t0, mean reward over the bin]
        self._weights = []     # [seconds since t0, age of the weights in s]
        self._stats = []       # [seconds since t0, {metric: value}]
        self._bin = None       # [bin index, sum, count]

    def start(self, run):
        """A trainer connected. A new run starts a new history; a reconnect
        of the same run keeps the one it has."""
        with self._lock:
            run = run or 'robot'
            if run != self.run:
                self.run = run
                self.t0 = time.time()
                self.latest = {}
                self.train_started = None
                self.waiting = False
                self._reset()

    def reward(self, value):
        now = time.time() - self.t0
        index = int(now)
        with self._lock:
            if self._bin is not None and self._bin[0] != index:
                i, total, count = self._bin
                self._append(self._rewards, [i + 0.5, total / count])
                self._bin = None
            if self._bin is None:
                self._bin = [index, 0.0, 0]
            self._bin[1] += float(value)
            self._bin[2] += 1

    def weights(self, stamp):
        """Weights swapped in; their age from the learner's publish stamp
        (nanoseconds, the learner's clock) when it is one."""
        now = time.time()
        try:
            age = now - int(stamp) / 1e9
        except (TypeError, ValueError):
            age = None
        with self._lock:
            self._append(self._weights, [now - self.t0, age])

    def stats(self, values):
        if not isinstance(values, dict):
            return
        with self._lock:
            self.latest = dict(values)
            self._append(self._stats, [time.time() - self.t0, dict(values)])

    def training(self, started_ago, waiting):
        """The trainer's pause state: how long ago training started (an age,
        so the trainer's clock need not agree with this one), and whether a
        run started paused is still waiting for it."""
        with self._lock:
            self.train_started = (None if started_ago is None
                                  else time.time() - float(started_ago))
            self.waiting = bool(waiting)

    def snapshot(self):
        with self._lock:
            return dict(
                run=self.run, t0=self.t0, now=time.time(),
                train_started=self.train_started, waiting=self.waiting,
                reward=list(self._rewards), weights=list(self._weights),
                stats=list(self._stats))

    @staticmethod
    def _append(series, point):
        series.append(point)
        if len(series) > MAX_POINTS:
            del series[:len(series) - MAX_POINTS]
