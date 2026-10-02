#!/usr/bin/env python3
"""Wi-Fi bridge between the tuning dashboard and the Teensy (run on the Pi).

Serves tools/pid_tune_dashboard.html and pipes raw protocol bytes between the
browser (WebSocket) and the Teensy (USB serial) without parsing them.

    python3 tune_bridge.py [--port /dev/ttyACM0] [--bind 127.0.0.1] [--http-port 8000] [--sim]

Then open http://<pi-ip>:8000/. Owns the serial port, so drone_ctl.py can't run
at the same time. Sends a KILL burst whenever the active client disconnects.
"""

import argparse
import base64
import glob
import hashlib
import math
import os
import queue
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
except ImportError:
    serial = None

# Only enough protocol to synthesise a kill. Must match teensy_fc.ino.
UP_HDR = b"\xA5\x5A"
UP_LEN = 14
FLAG_KILL = 0x02

WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

DASHBOARD_CANDIDATES = [
    os.path.join("..", "tools", "pid_tune_dashboard.html"),
    os.path.join("tools", "pid_tune_dashboard.html"),
    "pid_tune_dashboard.html",
]


def crc8(data: bytes) -> int:
    c = 0x00
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
    return c


def build_kill(seq: int) -> bytes:
    body = struct.pack("<BhhhHH", FLAG_KILL, 0, 0, 0, 0, seq & 0xFFFF)
    return UP_HDR + body + bytes([crc8(body)])


def find_port():
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/serial/by-id/*Teensy*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


def find_dashboard(explicit=None):
    if explicit:
        return explicit if os.path.isfile(explicit) else None
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in DASHBOARD_CANDIDATES:
        path = os.path.normpath(os.path.join(here, rel))
        if os.path.isfile(path):
            return path
    return None


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 1))  # no packet sent; just picks a route
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


class SerialHub:
    """Owns the serial port: one writer lock, one reader thread, one client."""

    def __init__(self, port, sim=False):
        self.sim = sim
        self.ser = None
        self.lock = threading.Lock()
        self.client = None
        self.client_lock = threading.Lock()
        self.running = True
        self.tx_bytes = 0
        self.rx_bytes = 0
        self._kill_seq = 0
        if not sim:
            if serial is None:
                raise RuntimeError("pyserial not installed: pip3 install pyserial")
            self.ser = serial.Serial(port, 500000, timeout=0.05)
            time.sleep(0.2)  # Teensy resets when the port opens
            self.ser.reset_input_buffer()
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def _read_loop(self):
        fake = _FakeTeensy() if self.sim else None
        while self.running:
            if fake is not None:
                chunk = fake.poll()
                if not chunk:
                    time.sleep(0.005)
                    continue
            else:
                try:
                    chunk = self.ser.read(4096)
                except Exception as e:
                    print(f"[serial] read error: {e}", file=sys.stderr)
                    time.sleep(0.5)
                    continue
                if not chunk:
                    continue
            self.rx_bytes += len(chunk)
            with self.client_lock:
                c = self.client
            if c is not None:
                c.send_binary(chunk)

    def write(self, data: bytes):
        self.tx_bytes += len(data)
        if self.ser is None:
            return
        with self.lock:
            try:
                self.ser.write(data)
            except Exception as e:
                print(f"[serial] write error: {e}", file=sys.stderr)

    def send_kill_burst(self, n=8):
        for _ in range(n):
            self.write(build_kill(self._kill_seq))
            self._kill_seq = (self._kill_seq + 1) & 0xFFFF
            time.sleep(0.005)

    def attach(self, session):
        # Newest wins, so a dead tab can't lock you out of the aircraft
        with self.client_lock:
            old = self.client
            self.client = session
        if old is not None and old is not session:
            print("[ws] evicting previous client")
            old.close(1000, "superseded by a new connection")

    def detach(self, session):
        with self.client_lock:
            if self.client is session:
                self.client = None
                return True
        return False

    def close(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass


class _FakeTeensy:
    """Plausible 59-byte telemetry frames at 20 Hz for --sim."""

    def __init__(self):
        self.t0 = time.time()
        self.next = 0.0
        self.seq = 0

    def poll(self):
        now = time.time()
        if now < self.next:
            return b""
        self.next = now + 0.05
        t = now - self.t0
        roll = 6.0 * math.sin(t * 0.9)
        pitch = 4.0 * math.sin(t * 0.6 + 1.0)
        yaw = 20.0 * math.sin(t * 0.2)
        body = struct.pack(
            "<BhhhHHHHHBH9f",
            0, int(roll * 100), int(pitch * 100), int(yaw * 100),
            1060, 1060, 1060, 1060, 100, 0, self.seq & 0xFFFF,
            0.20, 0.30, 0.05, 0.20, 0.30, 0.05, 0.30, 0.05, 0.00015,
        )
        self.seq += 1
        return b"\x5A\xA5" + body + bytes([crc8(body)])


class WSSession:
    """Minimal RFC 6455 server session: one peer, binary frames, no extensions."""

    def __init__(self, sock, hub):
        self.sock = sock
        self.hub = hub
        self.alive = True
        self.txq = queue.Queue(maxsize=256)
        self.send_lock = threading.Lock()
        self.writer = threading.Thread(target=self._write_loop, daemon=True)
        self.writer.start()

    def send_binary(self, payload: bytes):
        if not self.alive:
            return
        try:
            self.txq.put_nowait(payload)
        except queue.Full:
            # Client isn't draining: drop stale telemetry rather than add latency
            try:
                while True:
                    self.txq.get_nowait()
            except queue.Empty:
                pass
            try:
                self.txq.put_nowait(payload)
            except queue.Full:
                pass

    def _write_loop(self):
        while self.alive:
            try:
                payload = self.txq.get(timeout=0.25)
            except queue.Empty:
                continue
            if payload is None:
                break
            self._send_frame(0x2, payload)

    def _send_frame(self, opcode: int, payload: bytes):
        n = len(payload)
        if n < 126:
            hdr = struct.pack("!BB", 0x80 | opcode, n)
        elif n < (1 << 16):
            hdr = struct.pack("!BBH", 0x80 | opcode, 126, n)
        else:
            hdr = struct.pack("!BBQ", 0x80 | opcode, 127, n)
        try:
            with self.send_lock:
                self.sock.sendall(hdr + payload)
        except Exception:
            self.alive = False

    def close(self, code=1000, reason=""):
        if not self.alive:
            return
        self._send_frame(0x8, struct.pack("!H", code) + reason.encode("utf-8")[:123])
        self.alive = False
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def read_loop(self):
        frag_op = None
        frag = bytearray()
        while self.alive:
            hdr = self._recv_exact(2)
            if hdr is None:
                break
            fin = bool(hdr[0] & 0x80)
            opcode = hdr[0] & 0x0F
            masked = bool(hdr[1] & 0x80)
            length = hdr[1] & 0x7F
            if length == 126:
                ext = self._recv_exact(2)
                if ext is None:
                    break
                length = struct.unpack("!H", ext)[0]
            elif length == 127:
                ext = self._recv_exact(8)
                if ext is None:
                    break
                length = struct.unpack("!Q", ext)[0]
            # Client frames must be masked; anything huge is a bad peer
            if not masked or length > (1 << 20):
                break
            mask = self._recv_exact(4)
            if mask is None:
                break
            payload = self._recv_exact(length) if length else b""
            if payload is None:
                break
            if length:
                payload = bytes(payload[i] ^ mask[i & 3] for i in range(length))

            if opcode == 0x8:  # close
                break
            if opcode == 0x9:  # ping
                self._send_frame(0xA, payload)
                continue
            if opcode == 0xA:  # pong
                continue
            if opcode == 0x0:  # continuation
                if frag_op is None:
                    break
                frag += payload
                if fin:
                    self._dispatch(frag_op, bytes(frag))
                    frag_op, frag = None, bytearray()
                continue
            if opcode in (0x1, 0x2):
                if not fin:
                    frag_op, frag = opcode, bytearray(payload)
                    continue
                self._dispatch(opcode, payload)
                continue
            break

        self.alive = False
        try:
            self.txq.put_nowait(None)
        except queue.Full:
            pass

    def _dispatch(self, opcode, payload):
        if opcode == 0x2:  # binary passes straight through; text is ignored
            self.hub.write(payload)


def ws_accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1(key.encode("ascii") + WS_GUID).digest()).decode("ascii")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "TeensyQuadBridge/1.0"

    def log_message(self, fmt, *args):
        if self.path != "/favicon.ico":
            sys.stderr.write("[http] %s %s\n" % (self.command, self.path))

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/ws":
            self.handle_websocket()
        elif path in ("/", "/index.html", "/pid_tune_dashboard.html"):
            self.serve_dashboard()
        elif path == "/health":
            hub = self.server.hub
            body = ('{"ok":true,"client":%s,"tx":%d,"rx":%d,"sim":%s}' % (
                "true" if hub.client else "false", hub.tx_bytes, hub.rx_bytes,
                "true" if hub.sim else "false")).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404, "Not found")

    def serve_dashboard(self):
        path = self.server.dashboard_path
        if not path:
            self.send_error(500, "pid_tune_dashboard.html not found. Pass --dashboard /path/to/it.")
            return
        try:
            with open(path, "rb") as f:
                body = f.read()
        except OSError as e:
            self.send_error(500, f"Cannot read dashboard: {e}")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def handle_websocket(self):
        if self.headers.get("Upgrade", "").lower() != "websocket":
            self.send_error(400, "Expected a WebSocket upgrade")
            return
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self.send_error(400, "Missing Sec-WebSocket-Key")
            return
        self.wfile.write((
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {ws_accept_key(key)}\r\n"
            "\r\n"
        ).encode("ascii"))
        self.wfile.flush()

        sock = self.connection
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # no Nagle on a control link
        except Exception:
            pass
        sock.settimeout(None)

        hub = self.server.hub
        session = WSSession(sock, hub)
        hub.attach(session)
        peer = self.client_address[0]
        print(f"[ws] client connected from {peer}")
        try:
            session.read_loop()
        finally:
            was_active = hub.detach(session)
            session.alive = False
            print(f"[ws] client {peer} disconnected")
            # Only the active client kills; an evicted one must not disarm its replacement
            if was_active:
                print("[ws] sending KILL burst")
                hub.send_kill_burst()
            self.close_connection = True


class BridgeServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    ap = argparse.ArgumentParser(description="Wireless bridge from a browser to the Teensy flight controller")
    ap.add_argument("--port", default=None, help="serial device (default: auto-detect)")
    ap.add_argument("--bind", default="0.0.0.0", help="listen address (127.0.0.1 for SSH tunnelling)")
    ap.add_argument("--http-port", type=int, default=8000, help="HTTP/WebSocket port (default: 8000)")
    ap.add_argument("--dashboard", default=None, help="path to pid_tune_dashboard.html")
    ap.add_argument("--sim", action="store_true", help="fake Teensy, no hardware needed")
    args = ap.parse_args()

    port = args.port or find_port()
    if not args.sim and port is None:
        sys.exit("No serial port found. Plug in the Teensy, pass --port, or use --sim.")

    dashboard = find_dashboard(args.dashboard)
    if dashboard is None:
        print("[warn] pid_tune_dashboard.html not found; pass --dashboard /path/to/it", file=sys.stderr)

    try:
        hub = SerialHub(port, sim=args.sim)
    except Exception as e:
        sys.exit(f"Could not open {port}: {e}\nIs drone_ctl.py already running? Only one can own the port.")

    httpd = BridgeServer((args.bind, args.http_port), Handler)
    httpd.hub = hub
    httpd.dashboard_path = dashboard

    shown = lan_ip() if args.bind == "0.0.0.0" else args.bind
    print("=" * 62)
    print("  Teensy quad -- wireless tuning bridge")
    print("=" * 62)
    print(f"  serial     : {'SIMULATED (no hardware)' if args.sim else port}")
    print(f"  dashboard  : {dashboard or '(not found -- serve it yourself)'}")
    print(f"  open       : http://{shown}:{args.http_port}/")
    print(f"  websocket  : ws://{shown}:{args.http_port}/ws")
    print()
    print("  Props off until you've run the bench checklist in README.md.")
    print("  Ctrl-C here disarms and exits.")
    print("=" * 62)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down -- sending KILL.")
    finally:
        hub.send_kill_burst()
        hub.close()
        httpd.shutdown()


if __name__ == "__main__":
    main()
