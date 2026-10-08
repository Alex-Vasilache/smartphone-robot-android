"""A web page on the robot phone: a joystick, and a live mirror of its screen.

Open http://<robot phone's WiFi address>:8080 from another phone or a computer
on the same network; the robot's screen shows the address. Two views of one
page: / is the joystick (with a few live numbers), /mirror the robot's screen.

    GET  /, /mirror   the page
    GET  /ws          a WebSocket: state pushed at WEB_HZ, commands sent back
    GET  /state       the screen's numbers as JSON (fallback: polling)
    POST /cmd         {"forward": f, "turn": t} in [-1, 1], or {"release": true}
    POST /mode        {"mode": "auto" | "manual"}

The page talks over one WebSocket. Polling plain HTTP was measured at ~110 ms
(p50) from a posted command to the page seeing it, up to ~250 ms: a fresh TCP
connection and a thread per request, a 10 Hz state refresh and a 10 Hz poll on
top of it. On one held-open socket the stick goes out on every move and the
state comes back the moment the control loop publishes it. The HTTP
endpoints stay as the page's fallback.

The server runs on threads of its own and only ever touches the CommandSource
(behind its lock) and the state dict the control loop hands to `publish`, so
it never holds the control loop up: one broadcaster thread does the sending,
and a client too slow to take a frame within SEND_TIMEOUT is dropped.
"""

import base64
import hashlib
import json
import pkgutil
import socket
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 8080
WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
SEND_TIMEOUT = 1.0


def local_address():
    """This phone's address on the WiFi, by asking the routing table."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(('10.255.255.255', 1))  # nothing is sent
        return probe.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        probe.close()


def ws_frame(payload, opcode=0x1):
    """One unmasked, unfragmented server frame."""
    n = len(payload)
    if n < 126:
        head = struct.pack('!BB', 0x80 | opcode, n)
    elif n < 1 << 16:
        head = struct.pack('!BBH', 0x80 | opcode, 126, n)
    else:
        head = struct.pack('!BBQ', 0x80 | opcode, 127, n)
    return head + payload


def ws_read(rfile):
    """(opcode, payload) of the next client frame; None at end of stream.

    Clients always mask. Fragmented messages are not reassembled: the page
    only sends a few dozen bytes at a time, which browsers never fragment.
    """
    head = rfile.read(2)
    if len(head) < 2:
        return None
    opcode, n = head[0] & 0x0F, head[1] & 0x7F
    if n == 126:
        n, = struct.unpack('!H', rfile.read(2))
    elif n == 127:
        n, = struct.unpack('!Q', rfile.read(8))
    mask = rfile.read(4) if head[1] & 0x80 else b'\0\0\0\0'
    data = rfile.read(n)
    if len(data) < n:
        return None
    # Unmask as one big integer XOR rather than byte by byte in Python: the
    # reader thread holds the GIL for less time with the control loop waiting.
    key = int.from_bytes((mask * (n // 4 + 1))[:n], 'little')
    return opcode, (int.from_bytes(data, 'little') ^ key).to_bytes(n, 'little')


class _Client:

    def __init__(self, sock):
        self.sock = sock
        self.lock = threading.Lock()

    def send(self, frame):
        with self.lock:
            self.sock.sendall(frame)


class WebUI:

    def __init__(self, commands, port=PORT):
        self.commands = commands
        self.port = port
        self.state = {}   # the latest published, never mutated
        self.page = pkgutil.get_data('webpages', 'index.html')
        self._server = None
        self._clients = set()
        self._cond = threading.Condition()
        self._version = 0

    def publish(self, state):
        """Hand the page a new state. Cheap: the broadcaster does the sending."""
        with self._cond:
            self.state = state
            self._version += 1
            self._cond.notify()

    def _broadcast(self):
        seen = 0
        while True:
            with self._cond:
                while self._version == seen or not self._clients:
                    self._cond.wait()
                seen = self._version
                state = self.state
                clients = list(self._clients)
            frame = ws_frame(json.dumps(state).encode())
            for client in clients:
                try:
                    client.send(frame)
                except OSError:
                    self._drop(client)

    def _drop(self, client):
        with self._cond:
            self._clients.discard(client)
        try:
            client.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def handle(self, message):
        """One message from the page, over either transport."""
        kind = message.get('t')
        if kind == 'cmd':
            self.commands.joystick(message['forward'], message['turn'])
        elif kind == 'release':
            self.commands.release()
        elif kind == 'mode':
            self.commands.set_mode(message['mode'])
        else:
            raise ValueError('unknown message %r' % kind)

    def start(self):
        ui = self

        class Handler(BaseHTTPRequestHandler):

            def log_message(self, *args):
                pass  # one line per poll would bury the bridge's own log

            def _send(self, code, body, kind):
                self.send_response(code)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _json(self, data, code=200):
                self._send(code, json.dumps(data).encode(), 'application/json')

            def do_GET(self):
                path = self.path.split('?')[0]
                if path == '/ws':
                    self._websocket()
                elif path in ('/', '/mirror', '/drive'):
                    self._send(200, ui.page, 'text/html; charset=utf-8')
                elif path == '/state':
                    self._json(ui.state)
                elif path == '/favicon.ico':
                    self._send(204, b'', 'image/x-icon')
                else:
                    self._send(404, b'not found', 'text/plain')

            def do_POST(self):
                try:
                    n = int(self.headers.get('Content-Length', 0))
                    body = json.loads(self.rfile.read(n) or b'{}')
                    path = self.path.split('?')[0]
                    if path == '/cmd':
                        ui.handle(dict(body, t='release' if body.get('release')
                                       else 'cmd'))
                    elif path == '/mode':
                        ui.handle(dict(body, t='mode'))
                    else:
                        return self._send(404, b'not found', 'text/plain')
                    self._json(dict(ok=True))
                except Exception as e:  # noqa: BLE001 -- a bad post is the page's problem
                    self._json(dict(ok=False, error=str(e)), 400)

            def _websocket(self):
                key = self.headers.get('Sec-WebSocket-Key')
                if not key or 'websocket' not in self.headers.get('Upgrade', '').lower():
                    return self._send(400, b'websocket only', 'text/plain')
                accept = base64.b64encode(hashlib.sha1(
                    (key + WS_GUID).encode()).digest()).decode()
                self.send_response(101)
                self.send_header('Upgrade', 'websocket')
                self.send_header('Connection', 'Upgrade')
                self.send_header('Sec-WebSocket-Accept', accept)
                self.end_headers()
                self.wfile.flush()
                sock = self.connection
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(SEND_TIMEOUT)
                client = _Client(sock)
                with ui._cond:
                    ui._clients.add(client)
                    ui._cond.notify()
                try:
                    client.send(ws_frame(json.dumps(ui.state).encode()))
                    while True:
                        try:
                            frame = ws_read(self.rfile)
                        except socket.timeout:
                            continue  # idle page; the timeout is for sends
                        if frame is None:
                            break
                        opcode, data = frame
                        if opcode == 0x8:  # close
                            break
                        if opcode == 0x9:  # ping
                            client.send(ws_frame(data, 0xA))
                        elif opcode == 0x1:
                            try:
                                ui.handle(json.loads(data))
                            except Exception as e:  # noqa: BLE001
                                print('dreamerBridge: bad web message: %s' % e)
                except OSError:
                    pass
                finally:
                    ui._drop(client)
                    self.close_connection = True

        class Server(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        try:
            self._server = Server(('0.0.0.0', self.port), Handler)
        except OSError as e:
            print('dreamerBridge: web page unavailable on port %d: %s'
                  % (self.port, e))
            return None
        threading.Thread(target=self._server.serve_forever, name='webui',
                         daemon=True).start()
        threading.Thread(target=self._broadcast, name='webui_push',
                         daemon=True).start()
        url = 'http://%s:%d' % (local_address(), self.port)
        print('dreamerBridge: joystick at %s, screen mirror at %s/mirror'
              % (url, url))
        return url
