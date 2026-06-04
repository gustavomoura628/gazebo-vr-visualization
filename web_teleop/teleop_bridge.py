#!/usr/bin/env python3
"""Quest teleop bridge — WebRTC edition.

Puppets the go2w sim from a browser / Quest headset. Runs INSIDE the sim
container (same ROS graph; --net=host so its ports are reachable over WiFi).

Why WebRTC (vs the old MJPEG):
  - Video goes over UDP, so packet loss on a flaky hotspot drops a frame
    instead of stalling the whole stream (TCP head-of-line blocking was the
    main cause of the choppiness).
  - Control travels on a SEPARATE, unreliable+unordered DataChannel — fully
    decoupled from video, lowest latency, never blocks behind a video frame.
  - Browser-side, control is sent on its own timer, not the render loop.

Transport summary:
  HTTPS (aiohttp, self-signed) on :8443  -> static pages + WebRTC signaling
    GET  /            -> 2D page
    GET  /vr.html     -> WebXR page
    GET  /three.min.js, /*.js etc (static)
    POST /offer       -> WebRTC offer/answer signaling
  WebRTC peer:
    video track       -> /oakd/rgb/preview/image_raw, paced ~30 fps
    datachannel "ctl" -> {vx,vy,vyaw} JSON, unreliable/unordered
  Plain HTTP on :8080 keeps a tiny GET /cmd fallback (fire-and-forget) and the
    2D page, for debugging without WebRTC.

Run (inside the container):
    source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
    python3 teleop_bridge.py
"""
import asyncio
import fractions
import json
import os
import ssl
import subprocess
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from unitree_api.msg import Request

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
HTTP_PORT = 8080
HTTPS_PORT = 8443
CERT_PATH = "/tmp/teleop_cert.pem"
KEY_PATH = "/tmp/teleop_key.pem"
CAMERA_TOPIC = "/oakd/rgb/preview/image_raw"
SPORT_REQUEST_TOPIC = "/api/sport/request"

# Two MID-360 stand-in lidars (matching the real robot: a top one mounted normally
# and a bottom one mounted upside-down, both tilted forward). Each cloud is
# transformed from its sensor frame to the common robot BODY frame (x fwd, y left,
# z up) before merging, so the browser just does a fixed body->VR remap (no
# per-source rotation there).
# These tilts MUST MATCH the xacro mount (top_lidar_pitch / bottom_lidar_pitch).
TOP_LIDAR_PITCH = 0.30      # rad, forward tilt
BOTTOM_LIDAR_PITCH = 0.30   # rad, forward tilt


def _rpy_R(roll, pitch, yaw):
    # Fixed-axis RPY -> R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Maps a vector from the
    # sensor frame into the robot body frame (same convention as URDF <origin rpy>).
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return (Rz @ Ry @ Rx).astype(np.float32)


# sensor-frame -> body-frame rotation per source key.
_SOURCE_ROT = {
    "lidar_top":    _rpy_R(0.0,     TOP_LIDAR_PITCH,    np.pi / 2),
    "lidar_bottom": _rpy_R(np.pi,   BOTTOM_LIDAR_PITCH, np.pi / 2),
}

# Sources = list of (ros_topic, key). Default: the two lidars. Single depth-camera
# fallback via env:  CLOUD_TOPIC=/oakd/rgb/preview/depth/points CLOUD_FRAME=optical
if os.environ.get("CLOUD_TOPIC"):
    LIDAR_SOURCES = [(os.environ["CLOUD_TOPIC"], os.environ.get("CLOUD_FRAME", "optical"))]
else:
    LIDAR_SOURCES = [("/lidar/points", "lidar_top"), ("/lidar/points2", "lidar_bottom")]
LIDAR_KEYS = [k for _, k in LIDAR_SOURCES]   # order: [top, bottom] for the 2-lidar case

LIDAR_PERIOD_MS = 100   # 10 Hz (the future throttle / prediction knob)
CLOUD_MAX_PTS = 850     # subsample target PER source (pre-cull); merged stays
                        # ~16 KB/msg — small enough for the unreliable datachannel.
MIN_RANGE = 0.15        # cull only point-blank noise (m). NOT a self-hit filter:
                        # self-returns are left visible so the top/bottom diagnostic
                        # modes can show them (the mount is bad because TB4 != dog).

API_MOVE = 1008
API_STOPMOVE = 1003

VX_MAX = 0.6
VX_MIN = -0.4
VY_MAX = 0.1
VYAW_MAX = 2.2

CMD_TIMEOUT = 0.4    # safety watchdog: stop if no control arrives for this long
PUBLISH_HZ = 10.0
HEARTBEAT_S = 0.3
VIDEO_FPS = 30

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8", ".js": "application/javascript",
    ".css": "text/css", ".png": "image/png", ".ico": "image/x-icon",
}


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Shared state (written by ROS thread + asyncio thread; guarded by locks)
# ---------------------------------------------------------------------------
class Shared:
    def __init__(self):
        self.frame_lock = threading.Lock()
        self.rgb = None  # HxWx3 uint8 RGB, latest camera frame
        self.cmd_lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.cmd_t = 0.0  # monotonic time of last control msg
        # rolling stats for the 1 Hz control log (reset by take_stats)
        self.n_msgs = 0
        self.n_zero = 0
        self.n_nonzero = 0
        # lidar: latest body-frame points per source (key -> np.float32 (N,3)),
        # merged on demand. + bytes-sent counter for the log.
        self.lidar_lock = threading.Lock()
        self.lidar_clouds = {}
        self.lidar_npts = 0
        self.lidar_bytes_sent = 0  # rolling, reset by take_stats

    def set_cmd(self, vx, vy, vyaw):
        with self.cmd_lock:
            self.vx = _clamp(float(vx), VX_MIN, VX_MAX)
            self.vy = _clamp(float(vy), -VY_MAX, VY_MAX)
            self.vyaw = _clamp(float(vyaw), -VYAW_MAX, VYAW_MAX)
            self.cmd_t = time.monotonic()
            self.n_msgs += 1
            if abs(self.vx) < 1e-3 and abs(self.vy) < 1e-3 and abs(self.vyaw) < 1e-3:
                self.n_zero += 1
            else:
                self.n_nonzero += 1

    def take_stats(self):
        with self.cmd_lock:
            s = (self.n_msgs, self.n_zero, self.n_nonzero, self.vx, self.vyaw)
            self.n_msgs = self.n_zero = self.n_nonzero = 0
        with self.lidar_lock:
            lb = self.lidar_bytes_sent
            self.lidar_bytes_sent = 0
        return s + (lb,)

    def set_lidar_cloud(self, key, arr):
        with self.lidar_lock:
            self.lidar_clouds[key] = arr

    def get_lidar(self):
        # Merge sources (in LIDAR_KEYS order: top first, bottom second) into one
        # blob: [uint32 n_top][float32 XYZ top...][float32 XYZ bottom...]. The
        # header lets the browser show top / bottom / both for diagnostics.
        with self.lidar_lock:
            clouds = [(k, self.lidar_clouds.get(k)) for k in LIDAR_KEYS]
        clouds = [(k, c) for k, c in clouds if c is not None and c.size]
        if not clouds:
            return None
        n_top = clouds[0][1].shape[0]
        allpts = np.concatenate([c for _, c in clouds], axis=0).astype("<f4")
        self.lidar_npts = allpts.shape[0]
        return np.uint32(n_top).tobytes() + allpts.tobytes()

    def add_lidar_bytes(self, n):
        with self.lidar_lock:
            self.lidar_bytes_sent += n


SHARED = Shared()


# ---------------------------------------------------------------------------
# ROS node: camera in, MOVE out
# ---------------------------------------------------------------------------
class TeleopNode(Node):
    def __init__(self):
        super().__init__("quest_teleop_bridge")
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=2,
        )
        self.create_subscription(Image, CAMERA_TOPIC, self._on_image, cam_qos)
        for topic, key in LIDAR_SOURCES:
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, k=key: self._on_cloud(msg, k), cam_qos)
        self.pub = self.create_publisher(Request, SPORT_REQUEST_TOPIC, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self._last_pub = (None, None, None)
        self._last_pub_t = 0.0
        self._was_moving = False
        self._log_ticks = 0
        self.get_logger().info("Quest teleop bridge (WebRTC) node up.")

    def _on_image(self, msg: Image):
        w, h, step = msg.width, msg.height, msg.step
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        rowbytes = w * 3
        try:
            if step and step != rowbytes:
                data = data.reshape(h, step)[:, :rowbytes]
            rgb = data.reshape(h, w, 3)
            if msg.encoding == "bgr8":
                rgb = rgb[:, :, ::-1]
            with SHARED.frame_lock:
                SHARED.rgb = np.ascontiguousarray(rgb)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"image convert failed: {e}")

    def _on_scan(self, msg: LaserScan):
        # LaserScan -> XYZ points in the robot/lidar frame (x fwd, y left, z up).
        # z=0 (single ring today); a real 3D lidar would fill z. Packed float32
        # XYZ triplets, little-endian, ready to ship straight to the browser.
        try:
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            n = ranges.size
            ang = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
            ok = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
            r, a = ranges[ok], ang[ok]
            x = r * np.cos(a)
            y = r * np.sin(a)
            z = np.zeros_like(x)
            pts = np.stack([x, y, z], axis=1).astype("<f4")
            SHARED.set_lidar(pts.tobytes(), pts.shape[0])
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"scan convert failed: {e}")

    def _on_cloud(self, msg: PointCloud2, key: str):
        # PointCloud2 -> latest body-frame float32 XYZ for this source.
        # Each source is transformed to the common robot body frame so the merged
        # cloud is coherent: lidars via their mount rotation (_SOURCE_ROT), the
        # depth-camera fallback via the optical-axis remap.
        try:
            step = msg.point_step
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, step)
            sr = max(1, raw.shape[0] // CLOUD_MAX_PTS)   # uniform subsample for now
            raw = raw[::sr]                              # (blue-noise replaces this in Step 2)
            xyz = raw[:, 0:12].copy().view(np.float32).reshape(-1, 3)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            # cull self-hits: drop points closer than MIN_RANGE (the lidar sees the
            # robot's own body, which would otherwise sit right at the viewer).
            xyz = xyz[(xyz * xyz).sum(axis=1) > (MIN_RANGE * MIN_RANGE)]
            if key == "optical":                         # depth camera
                ox, oy, oz = xyz[:, 0], xyz[:, 1], xyz[:, 2]
                xyz = np.stack([oz, -ox, -oy], axis=1)   # optical -> body frame
            elif key in _SOURCE_ROT:                     # lidar: sensor -> body
                xyz = xyz @ _SOURCE_ROT[key].T
            SHARED.set_lidar_cloud(key, xyz.astype(np.float32))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cloud convert failed ({key}): {e}")

    def _tick(self):
        with SHARED.cmd_lock:
            vx, vy, vyaw, t = SHARED.vx, SHARED.vy, SHARED.vyaw, SHARED.cmd_t
        fresh = (time.monotonic() - t) < CMD_TIMEOUT
        moving = fresh and (abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(vyaw) > 1e-3)
        now = time.monotonic()
        if moving:
            cur = (round(vx, 3), round(vy, 3), round(vyaw, 3))
            if cur != self._last_pub or (now - self._last_pub_t) > HEARTBEAT_S:
                self._publish(API_MOVE, {"x": vx, "y": vy, "z": vyaw})
                self._last_pub, self._last_pub_t = cur, now
            self._was_moving = True
        elif self._was_moving:
            self._publish(API_STOPMOVE, {})
            self._publish(API_STOPMOVE, {})
            self._was_moving = False
            self._last_pub = (None, None, None)

        # ~1 Hz control log: surfaces command rate, zero/nonzero split, and the
        # number of connected WebRTC peers. >1 peer = two clients fighting for
        # control (look HERE before theorizing about stutter).
        self._log_ticks += 1
        if self._log_ticks >= int(PUBLISH_HZ):
            self._log_ticks = 0
            n, z, nz, lvx, lvyaw, lbytes = SHARED.take_stats()
            npeers = len(PCS)
            if n > 0 or npeers > 1 or lbytes > 0:
                flag = "  <-- MULTIPLE PEERS (control conflict?)" if npeers > 1 else ""
                kbit = lbytes * 8 / 1000.0
                self.get_logger().info(
                    f"[ctl] peers={npeers} {n}/s (zero {z}/nonzero {nz}) "
                    f"vx={lvx:.2f} vyaw={lvyaw:.2f} | lidar {kbit:.0f} kbit/s "
                    f"({SHARED.lidar_npts} pts){flag}")

    def _publish(self, api_id, payload):
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        req.header.identity.api_id = api_id
        req.header.policy.noreply = True
        req.parameter = json.dumps(payload)
        self.pub.publish(req)


# ---------------------------------------------------------------------------
# WebRTC video track: latest ROS frame, paced to VIDEO_FPS
# ---------------------------------------------------------------------------
class CameraTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self._n = 0
        self._t0 = None

    async def recv(self):
        if self._t0 is None:
            self._t0 = time.time()
        self._n += 1
        target = self._t0 + self._n / VIDEO_FPS
        delay = target - time.time()
        if delay > 0:
            await asyncio.sleep(delay)
        with SHARED.frame_lock:
            rgb = SHARED.rgb
        if rgb is None:
            rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        frame = VideoFrame.from_ndarray(rgb, format="rgb24")
        frame.pts = self._n
        frame.time_base = fractions.Fraction(1, VIDEO_FPS)
        return frame


# ---------------------------------------------------------------------------
# aiohttp app: static + signaling
# ---------------------------------------------------------------------------
PCS = set()


def _static_response(path):
    rel = path.lstrip("/") or "index.html"
    if ".." in rel:
        return web.Response(status=403, text="nope")
    full = os.path.realpath(os.path.join(WEB_DIR, rel))
    if not full.startswith(WEB_DIR + os.sep) or not os.path.isfile(full):
        return web.Response(status=404, text="not found")
    ext = os.path.splitext(full)[1].lower()
    with open(full, "rb") as f:
        body = f.read()
    return web.Response(body=body, content_type=_CONTENT_TYPES.get(ext, "application/octet-stream").split(";")[0])


async def index(request):
    return _static_response("/index.html")


async def static_file(request):
    return _static_response(request.path)


async def cmd_http(request):
    """Fire-and-forget HTTP control fallback: /cmd?vx=&vy=&vyaw="""
    q = request.rel_url.query
    try:
        SHARED.set_cmd(q.get("vx", 0), q.get("vy", 0), q.get("vyaw", 0))
    except (TypeError, ValueError):
        return web.Response(status=400, text="bad")
    return web.Response(text="ok")


async def stop_http(request):
    SHARED.set_cmd(0, 0, 0)
    return web.Response(text="stopped")


async def offer(request):
    params = await request.json()
    pc = RTCPeerConnection()
    PCS.add(pc)

    @pc.on("datachannel")
    def on_datachannel(channel):
        if channel.label == "ctl":
            @channel.on("message")
            def on_message(message):
                try:
                    d = json.loads(message)
                    SHARED.set_cmd(d.get("vx", 0), d.get("vy", 0), d.get("vyaw", 0))
                except Exception:  # noqa: BLE001
                    pass
        elif channel.label == "lidar":
            # Server -> browser: push the latest scan blob every LIDAR_PERIOD_MS.
            async def lidar_loop():
                while channel.readyState == "open":
                    blob = SHARED.get_lidar()
                    if blob:
                        try:
                            channel.send(blob)
                            SHARED.add_lidar_bytes(len(blob))
                        except Exception:  # noqa: BLE001
                            pass  # skip this frame (e.g. transient/too-big); keep streaming
                    await asyncio.sleep(LIDAR_PERIOD_MS / 1000.0)
            asyncio.ensure_future(lidar_loop())

    @pc.on("connectionstatechange")
    async def on_state():
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            PCS.discard(pc)
            SHARED.set_cmd(0, 0, 0)  # safety: stop if peer drops

    pc.addTrack(CameraTrack())
    await pc.setRemoteDescription(
        RTCSessionDescription(sdp=params["sdp"], type=params["type"]))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.json_response(
        {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})


def make_app():
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)
    app.router.add_get("/cmd", cmd_http)
    app.router.add_get("/stop", stop_http)
    app.router.add_get("/{name}", static_file)
    return app


# ---------------------------------------------------------------------------
# cert + startup
# ---------------------------------------------------------------------------
def ensure_cert():
    if os.path.exists(CERT_PATH) and os.path.exists(KEY_PATH):
        return True
    try:
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
             "-keyout", KEY_PATH, "-out", CERT_PATH, "-days", "365",
             "-subj", "/CN=go2w-teleop"],
            check=True, capture_output=True, timeout=30)
        return True
    except Exception as e:  # noqa: BLE001
        print(f"[teleop] cert gen failed, HTTPS/VR disabled: {e}")
        return False


async def run_servers():
    app = make_app()
    runner = web.AppRunner(app)
    await runner.setup()
    # plain HTTP (2D + fallback)
    await web.TCPSite(runner, "0.0.0.0", HTTP_PORT).start()
    # HTTPS (required for WebXR secure context)
    if ensure_cert():
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(CERT_PATH, KEY_PATH)
        await web.TCPSite(runner, "0.0.0.0", HTTPS_PORT, ssl_context=ctx).start()
        print(f"[teleop] HTTPS :{HTTPS_PORT}  (VR: https://<ip>:{HTTPS_PORT}/vr.html)")
    print(f"[teleop] HTTP  :{HTTP_PORT}  (2D: http://<ip>:{HTTP_PORT}/)")
    while True:
        await asyncio.sleep(3600)


def main():
    rclpy.init()
    node = TeleopNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    try:
        asyncio.run(run_servers())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
