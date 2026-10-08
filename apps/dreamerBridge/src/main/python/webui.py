"""A web page on the robot phone: a joystick, and a live mirror of its screen.

Open http://<robot phone's WiFi address>:8080 from another phone or a computer
on the same network; the robot's screen shows the address. Two views of one
page: / is the joystick (with a few live numbers), /mirror the robot's screen.

    GET  /, /mirror   the page
    GET  /state       the screen's numbers as JSON, refreshed at 10 Hz
    POST /cmd         {"forward": f, "turn": t} in [-1, 1], or {"release": true}
    POST /mode        {"mode": "auto" | "manual"}

Plain HTTP polling rather than a websocket: it needs nothing beyond the
standard library, and a post every 50-100 ms is well inside what one control
step can spare. The server runs on threads of its own and only ever touches
the CommandSource (behind its lock) and a dict the control loop replaces
whole, so it never holds the control loop up.
"""

import json
import pkgutil
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 8080


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


class WebUI:

    def __init__(self, commands, port=PORT):
        self.commands = commands
        self.port = port
        self.state = {}   # replaced whole by the control loop, never mutated
        self.page = pkgutil.get_data('webpages', 'index.html')
        self._server = None

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
                if path in ('/', '/mirror', '/drive'):
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
                        if body.get('release'):
                            ui.commands.release()
                        else:
                            ui.commands.joystick(body['forward'], body['turn'])
                    elif path == '/mode':
                        ui.commands.set_mode(body['mode'])
                    else:
                        return self._send(404, b'not found', 'text/plain')
                    self._json(dict(ok=True))
                except Exception as e:  # noqa: BLE001 -- a bad post is the page's problem
                    self._json(dict(ok=False, error=str(e)), 400)

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
        url = 'http://%s:%d' % (local_address(), self.port)
        print('dreamerBridge: joystick at %s, screen mirror at %s/mirror'
              % (url, url))
        return url
