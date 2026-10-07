"""Reads the trainer's socket on a thread of its own, so the control loop never does.

The control loop used to pull frames itself, in the slack after each tick, and
a weight push is megabytes: copying it out of the socket, writing it to disk and
parsing it into a NumpyPolicy all happened between two control steps. Here a
receiver thread does all of that, and the control loop only takes finished
frames off a queue -- a weight update reaches it as one `weights_ready` frame
holding a policy already parsed, which `PolicyRunner.install` swaps in for the
cost of an assignment.

Pushes arrive either whole (`weights`, the old trainer) or in slices
(`wchunk`, with `offset` and `total`), which let the trainer slip its control
frames in between; this side reassembles either.

Pure standard library plus whatever `build` needs, so the same file runs in the
app and in the dreamerv3 repo's fake phone (`dreamerv3/deploy/bridge_link.py`
there is a verbatim copy).
"""

import collections
import json
import socket
import struct
import threading
import time


class Receiver:

  def __init__(self, sock, build=None, initial=b'', timeout=30.0):
    """`build(blob)` turns a pushed weight blob into whatever the control loop
    installs; it runs on the receiver thread. None passes the blob through."""
    self.sock = sock
    self.build = build
    self.timeout = timeout
    self.error = None
    self.weights_received = 0
    self.last_build_ms = 0.0
    self._buffer = bytearray(initial)
    self._inbox = collections.deque()
    self._ready = threading.Condition()
    self._closed = False
    self._partial = None   # [stamp, bytearray, total] while a push is in flight
    self._thread = threading.Thread(
        target=self._run, name='bridge_receiver', daemon=True)
    self._thread.start()

  def get_nowait(self):
    """The oldest finished frame as (header, payload), or None."""
    try:
      return self._inbox.popleft()
    except IndexError:
      if self.error is not None:
        raise self.error
      return None

  def get(self, timeout=None):
    """Block for the next finished frame."""
    timeout = self.timeout if timeout is None else timeout
    deadline = time.monotonic() + timeout
    with self._ready:
      while not self._inbox:
        if self.error is not None:
          raise self.error
        remaining = deadline - time.monotonic()
        if remaining <= 0:
          raise ConnectionError('Trainer went quiet')
        self._ready.wait(remaining)
      return self._inbox.popleft()

  def close(self):
    self._closed = True

  def _put(self, frame):
    with self._ready:
      self._inbox.append(frame)
      self._ready.notify()

  def _run(self):
    try:
      self._drain()
      while not self._closed:
        try:
          chunk = self.sock.recv(1 << 16)
        except socket.timeout:
          raise ConnectionError('Trainer went quiet')
        if not chunk:
          raise ConnectionError('Trainer closed the connection')
        self._buffer.extend(chunk)
        self._drain()
    except BaseException as e:  # noqa: BLE001 -- handed to the control loop
      if not self._closed:
        self.error = e if isinstance(e, Exception) else ConnectionError(repr(e))
      with self._ready:
        self._ready.notify_all()

  def _drain(self):
    while True:
      frame = parse(self._buffer)
      if frame is None:
        return
      header, blob = frame
      kind = header.get('type')
      if kind == 'weights':
        self._weights(header, blob)
      elif kind == 'wchunk':
        self._chunk(header, blob)
      else:
        self._put((header, blob))

  def _chunk(self, header, blob):
    stamp, offset, total = header.get('stamp'), header['offset'], header['total']
    if offset == 0 or self._partial is None or self._partial[0] != stamp:
      # A new push, or the remains of one we lost track of: start over.
      self._partial = [stamp, bytearray(), total]
    part = self._partial
    if len(part[1]) != offset:
      self._partial = None  # a gap; wait for the next push from its start
      return
    part[1].extend(blob)
    if len(part[1]) >= total:
      self._partial = None
      self._weights(dict(header, type='weights'), bytes(part[1]))

  def _weights(self, header, blob):
    t0 = time.monotonic()
    try:
      payload = self.build(blob) if self.build else blob
    except Exception as e:  # noqa: BLE001 -- a bad push must not end the link
      print('dreamerBridge: bad weight update %s: %s' % (header.get('stamp'), e))
      return
    self.last_build_ms = (time.monotonic() - t0) * 1e3
    self.weights_received += 1
    self._put((dict(header, type='weights_ready', build_ms=self.last_build_ms),
               payload))


def parse(buffer):
  """Pull one frame out of `buffer`, leaving a partial one in place."""
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
