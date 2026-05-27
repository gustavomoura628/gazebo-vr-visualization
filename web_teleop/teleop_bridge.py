#!/usr/bin/env python3
"""Quest teleop bridge — a single node that puppets the go2w sim from a browser.

Runs INSIDE the sim container (so it's on the same ROS graph; the container is
--net=host, so its HTTP port is reachable from the Quest over WiFi). One port
serves everything:

  GET /            -> the web UI (web_teleop/index.html)
  GET /stream      -> MJPEG video of /oakd/rgb/preview/image_raw
  GET /cmd?vx=&vy=&vyaw=  -> set the desired body velocity (browser sends ~20 Hz)
  GET /stop        -> zero the command (sends STOPMOVE)

It republishes the latest commanded velocity as Unitree sport-mode MOVE (api
1008) on /api/sport/request at a steady rate, which (a) drives the robot and
(b) dodges the ROS 2 discovery race that drops a single one-shot MOVE. If no
fresh command arrives within CMD_TIMEOUT (browser closed / tab backgrounded),
it falls back to STOPMOVE as a watchdog.

Run (inside the container):
    source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
    python3 teleop_bridge.py            # serves on 0.0.0.0:8080
"""
import io
import json
import os
import ssl
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import Image
from unitree_api.msg import Request

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
HTTP_PORT = 8080      # plain HTTP — fine for the 2D page
HTTPS_PORT = 8443     # HTTPS (self-signed) — REQUIRED for the WebXR/VR page
CERT_PATH = "/tmp/teleop_cert.pem"
KEY_PATH = "/tmp/teleop_key.pem"
CAMERA_TOPIC = "/oakd/rgb/preview/image_raw"
SPORT_REQUEST_TOPIC = "/api/sport/request"

API_MOVE = 1008
API_STOPMOVE = 1003

# Velocity caps (m/s, m/s, rad/s). Reverse is deliberately limited: the TB4
# stand-in has a backup-limit safety reflex that wedges motion_control if you
# drive backward for long. Forward/turn are the main teleop axes.
VX_MAX = 0.5
VX_MIN = -0.15
VY_MAX = 0.30
VYAW_MAX = 1.20

CMD_TIMEOUT = 0.4      # s without a fresh /cmd -> stop (watchdog)
PUBLISH_HZ = 10.0      # MOVE republish rate
HEARTBEAT_S = 0.3      # resend MOVE at least this often even if unchanged
STREAM_FPS = 30.0

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(WEB_DIR, "index.html")

_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript",
    ".css": "text/css",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".ico": "image/x-icon",
}

# ---------------------------------------------------------------------------
# JPEG encoder: prefer Pillow, fall back to OpenCV. One of these must be present
# in the container (Pillow is the lighter dep).
# ---------------------------------------------------------------------------
_encode = None
try:
    from PIL import Image as _PILImage

    def _encode(w, h, rgb_bytes):  # noqa: E306
        buf = io.BytesIO()
        _PILImage.frombytes("RGB", (w, h), rgb_bytes).save(buf, "JPEG", quality=80)
        return buf.getvalue()
    _ENCODER = "pillow"
except Exception:  # pragma: no cover
    pass

if _encode is None:
    try:
        import cv2
        import numpy as np

        def _encode(w, h, rgb_bytes):  # noqa: E306
            arr = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape(h, w, 3)
            ok, jpg = cv2.imencode(".jpg", arr[:, :, ::-1],
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
            return jpg.tobytes()
        _ENCODER = "opencv"
    except Exception:  # pragma: no cover
        _ENCODER = None


# ---------------------------------------------------------------------------
# Shared state between the ROS node and the HTTP server threads
# ---------------------------------------------------------------------------
class Shared:
    def __init__(self):
        self.frame_lock = threading.Lock()
        self.jpeg = None              # latest encoded JPEG bytes
        self.cmd_lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.cmd_t = 0.0              # monotonic time of last /cmd


SHARED = Shared()


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------
class TeleopBridge(Node):
    def __init__(self):
        super().__init__("quest_teleop_bridge")

        # The ros_gz camera bridge publishers don't reliably match a plain
        # RELIABLE subscription; BEST_EFFORT connects to anything.
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.create_subscription(Image, CAMERA_TOPIC, self._on_image, cam_qos)
        self.pub = self.create_publisher(Request, SPORT_REQUEST_TOPIC, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        self._last_pub = (None, None, None)
        self._last_pub_t = 0.0
        self._was_moving = False
        self._frames = 0

        if _ENCODER is None:
            self.get_logger().error(
                "No JPEG encoder available (need Pillow or OpenCV). "
                "Video stream will be blank. Install: pip install pillow")
        else:
            self.get_logger().info(f"JPEG encoder: {_ENCODER}")
        self.get_logger().info(
            f"Quest teleop bridge up. HTTP on :{HTTP_PORT}, camera {CAMERA_TOPIC}")

    # --- camera ---------------------------------------------------------
    def _on_image(self, msg: Image):
        if _encode is None:
            return
        w, h, step = msg.width, msg.height, msg.step
        data = bytes(msg.data)
        # Only rgb8/bgr8 8-bit handled; repack rows if stride has padding.
        rowbytes = w * 3
        if step and step != rowbytes:
            data = b"".join(data[i * step:i * step + rowbytes] for i in range(h))
        if msg.encoding == "bgr8":
            # swap B<->R via PIL-friendly route: handled by encoder expecting RGB,
            # so flip here cheaply using bytearray slicing.
            ba = bytearray(data)
            ba[0::3], ba[2::3] = ba[2::3], ba[0::3]
            data = bytes(ba)
        try:
            jpg = _encode(w, h, data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"encode failed: {e}")
            return
        with SHARED.frame_lock:
            SHARED.jpeg = jpg
        self._frames += 1

    # --- command republisher (the MOVE pump) ---------------------------
    def _tick(self):
        with SHARED.cmd_lock:
            vx, vy, vyaw, t = SHARED.vx, SHARED.vy, SHARED.vyaw, SHARED.cmd_t
        fresh = (time.monotonic() - t) < CMD_TIMEOUT
        moving = fresh and (abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(vyaw) > 1e-3)

        now = time.monotonic()
        if moving:
            cur = (round(vx, 3), round(vy, 3), round(vyaw, 3))
            # Resend on change, or as a heartbeat, so motion stays smooth and
            # we don't spam the adapter log every tick when holding steady.
            if cur != self._last_pub or (now - self._last_pub_t) > HEARTBEAT_S:
                self._publish(API_MOVE, {"x": vx, "y": vy, "z": vyaw})
                self._last_pub = cur
                self._last_pub_t = now
            self._was_moving = True
        else:
            # On transition to idle, send STOPMOVE (twice for robustness), then quiet.
            if self._was_moving:
                self._publish(API_STOPMOVE, {})
                self._publish(API_STOPMOVE, {})
                self._was_moving = False
                self._last_pub = (None, None, None)

    def _publish(self, api_id, payload):
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        req.header.identity.api_id = api_id
        req.header.policy.noreply = True
        req.parameter = json.dumps(payload)
        self.pub.publish(req)


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):  # silence per-request logging
        pass

    def _send(self, code, ctype, body, extra_headers=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        if extra_headers:
            for k, v in extra_headers.items():
                self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            try:
                with open(HTML_PATH, "rb") as f:
                    body = f.read()
            except FileNotFoundError:
                body = b"<h1>index.html not found next to teleop_bridge.py</h1>"
            self._send(200, "text/html; charset=utf-8", body)
            return

        if path == "/cmd":
            q = parse_qs(parsed.query)
            try:
                vx = _clamp(float(q.get("vx", ["0"])[0]), VX_MIN, VX_MAX)
                vy = _clamp(float(q.get("vy", ["0"])[0]), -VY_MAX, VY_MAX)
                vyaw = _clamp(float(q.get("vyaw", ["0"])[0]), -VYAW_MAX, VYAW_MAX)
            except ValueError:
                self._send(400, "text/plain", b"bad params")
                return
            with SHARED.cmd_lock:
                SHARED.vx, SHARED.vy, SHARED.vyaw = vx, vy, vyaw
                SHARED.cmd_t = time.monotonic()
            self._send(200, "text/plain", b"ok")
            return

        if path == "/stop":
            with SHARED.cmd_lock:
                SHARED.vx = SHARED.vy = SHARED.vyaw = 0.0
                SHARED.cmd_t = time.monotonic()
            self._send(200, "text/plain", b"stopped")
            return

        if path == "/stream":
            self._stream_mjpeg()
            return

        # Static files from web_teleop/ (e.g. /vr.html, /three.min.js).
        if self._serve_static(path):
            return

        self._send(404, "text/plain", b"not found")

    def _serve_static(self, path):
        rel = path.lstrip("/")
        if not rel or ".." in rel or rel.startswith("/"):
            return False
        full = os.path.realpath(os.path.join(WEB_DIR, rel))
        if not full.startswith(WEB_DIR + os.sep) or not os.path.isfile(full):
            return False
        ext = os.path.splitext(full)[1].lower()
        ctype = _CONTENT_TYPES.get(ext, "application/octet-stream")
        with open(full, "rb") as f:
            body = f.read()
        self._send(200, ctype, body)
        return True

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        period = 1.0 / STREAM_FPS
        try:
            while True:
                with SHARED.frame_lock:
                    jpg = SHARED.jpeg
                if jpg is not None:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass


def _serve_http():
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()


def _ensure_cert():
    """Generate a self-signed cert if missing. WebXR needs a secure context
    (HTTPS) when the Quest connects over the LAN; the user clicks through the
    self-signed warning once."""
    if os.path.exists(CERT_PATH) and os.path.exists(KEY_PATH):
        return True
    try:
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
             "-keyout", KEY_PATH, "-out", CERT_PATH, "-days", "365",
             "-subj", "/CN=go2w-teleop"],
            check=True, capture_output=True, timeout=30,
        )
        return True
    except Exception as e:  # noqa: BLE001
        print(f"[teleop] HTTPS disabled — cert generation failed: {e}")
        print("[teleop] The 2D page still works over HTTP; VR needs HTTPS.")
        return False


def _serve_https():
    if not _ensure_cert():
        return
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(CERT_PATH, KEY_PATH)
    srv = ThreadingHTTPServer(("0.0.0.0", HTTPS_PORT), Handler)
    srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
    srv.serve_forever()


def main():
    rclpy.init()
    node = TeleopBridge()
    threading.Thread(target=_serve_http, daemon=True).start()
    threading.Thread(target=_serve_https, daemon=True).start()
    node.get_logger().info(
        f"  2D page:  http://<laptop-ip>:{HTTP_PORT}/")
    node.get_logger().info(
        f"  VR page:  https://<laptop-ip>:{HTTPS_PORT}/vr.html  "
        f"(accept the self-signed cert warning)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
