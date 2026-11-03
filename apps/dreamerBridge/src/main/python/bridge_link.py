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
frames in between; this side reassembles either. A trainer that offers
`wstreams` in its hello sends them instead over connections of their own
(`WeightStreams`), one part per connection, so a long-distance link carries a
push several times faster and the control link never waits behind one.

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
    """`build(blob, header)` turns a pushed weight blob into whatever the
    control loop installs; it runs on the receiver thread. None passes the
    blob through."""
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

  def deliver_weights(self, header, blob):
    """A whole push that came some other way (WeightStreams): build it and
    hand it to the control loop as if it had come over this socket."""
    self._weights(header, blob)

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
      payload = self.build(blob, header) if self.build else blob
    except Exception as e:  # noqa: BLE001 -- a bad push must not end the link
      print('dreamerBridge: bad weight update %s: %s' % (header.get('stamp'), e))
      # The control loop tells the trainer, which then sends the next push
      # whole instead of as a delta on weights we do not hold.
      self._put((dict(header, type='weights_failed', error=str(e)[:200]), b''))
      return
    self.last_build_ms = (time.monotonic() - t0) * 1e3
    self.weights_received += 1
    self._put((dict(header, type='weights_ready', build_ms=self.last_build_ms,
                    ready_at=time.monotonic()), payload))


class WeightStreams:
  """The connections a trainer pushes weights over (its hello's `wstreams`).

  One thread per address connects, greets with the session token, and reads
  `wpart` frames; the parts of one push are joined here and the whole goes to
  `deliver(header, blob)` once every part has landed. A lost connection is
  retried every few seconds until `close`; the trainer pushes over the ones
  that are up (or over the control link while none are).
  """

  RETRY_S = 5.0

  def __init__(self, addresses, session, deliver, connect_timeout=10.0,
               protocol=3):
    self.addresses = list(addresses)
    self.session = session
    self.deliver = deliver
    self.connect_timeout = connect_timeout
    self.protocol = protocol
    self.up = 0
    self.parts_received = 0
    self._lock = threading.Lock()
    self._push = None    # [(stamp, parts, total), bytearray, set(parts), t0]
    self._socks = set()
    self._closed = False
    for i, address in enumerate(self.addresses):
      threading.Thread(target=self._run, args=(i, address),
                       name='bridge_wstream%d' % i, daemon=True).start()

  @property
  def receiving(self):
    return self._push is not None

  def close(self):
    self._closed = True
    with self._lock:
      socks = list(self._socks)
    for sock in socks:
      try:
        sock.shutdown(socket.SHUT_RDWR)
      except OSError:
        pass
      try:
        sock.close()
      except OSError:
        pass

  def _run(self, index, address):
    while not self._closed:
      sock = None
      try:
        sock = socket.create_connection(address, timeout=self.connect_timeout)
        sock.settimeout(None)  # quiet between pushes for as long as it likes
        with self._lock:
          self._socks.add(sock)
        hello = json.dumps(dict(
            type='wstream', protocol=self.protocol, session=self.session,
            index=index, blob_len=0)).encode('utf-8')
        sock.sendall(struct.pack('>I', len(hello)) + hello)
        self.up += 1
        try:
          self._read(sock)
        finally:
          self.up -= 1
      except Exception as e:  # noqa: BLE001 -- retried below
        if not self._closed:
          print('dreamerBridge: weight stream %d to %s:%d: %s'
                % ((index,) + tuple(address) + (e,)))
      finally:
        if sock is not None:
          with self._lock:
            self._socks.discard(sock)
          try:
            sock.close()
          except OSError:
            pass
      deadline = time.monotonic() + self.RETRY_S
      while not self._closed and time.monotonic() < deadline:
        time.sleep(0.2)

  def _read(self, sock):
    buffer = bytearray()
    while not self._closed:
      chunk = sock.recv(1 << 16)
      if not chunk:
        raise ConnectionError('trainer closed the stream')
      buffer.extend(chunk)
      while True:
        frame = parse(buffer)
        if frame is None:
          break
        if frame[0].get('type') == 'wpart':
          self._part(*frame)

  def _part(self, header, blob):
    key = (header.get('stamp'), int(header['parts']), int(header['total']))
    offset = int(header['offset'])
    with self._lock:
      if self._push is None or self._push[0] != key:
        # A new push replaces whatever was half-joined: only the newest
        # weights matter, and a retry of the same push may split differently.
        self._push = [key, bytearray(key[2]), set(), time.monotonic()]
      push = self._push
      push[1][offset:offset + len(blob)] = blob
      push[2].add(int(header['part']))
      self.parts_received += 1
      if len(push[2]) < key[1]:
        return
      self._push = None
    # recv_s: first part to last, the part of a push the network decides.
    self.deliver(dict(type='weights', stamp=key[0], parts=key[1], bytes=key[2],
                      recv_s=round(time.monotonic() - push[3], 3)),
                 bytes(push[1]))


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
