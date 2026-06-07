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

# --- MTU fix: shrink every WebRTC payload to survive a reduced-MTU path -------
# WebRTC/aiortc sizes packets for a 1500-byte LAN and does NO path-MTU discovery.
# A Tailscale/WireGuard tunnel (or most VPNs, or IPv6's guaranteed minimum) is
# ~1280, so full-size media packets are SILENTLY DROPPED while the tiny STUN/DTLS
# handshake packets fit -- you get "ICE connected, but no image and no lidar".
# PROVEN against a live 1280 Tailscale path: forcing the connection onto the
# Tailscale candidates gives conn=connected but video_frames=0 and a stuck lidar
# DataChannel buffer; over a normal LAN the SAME build works.
#
# Budget over IPv6 (the worst case; header is 40 B and routers never fragment):
#   usable UDP packet = 1280 - 40 (IPv6) - 8 (UDP) = 1232 bytes.
#   RTP video : payload + 12 (RTP) + ~16 (header exts) + ~16 (SRTP/GCM tag) <= 1232
#               -> payload <= ~1188.
#   SCTP data : payload + ~28 (SCTP common+DATA chunk) + ~29 (DTLS rec + GCM) <= 1232
#               -> payload <= ~1175.
# So 1200 (the previous guess) is actually ~12 B TOO BIG over IPv6 -- which is why
# the prior "fix" still failed on the second machine. We cap at 1000 (worst-case wire
# ~1092/1105 << 1280, ~180 B margin -- robust on a lossy link) and make it
# overridable via env so any future nudge is a bridge RESTART, never a rebuild.
# GENERAL: helps every low-MTU path; on a normal LAN it just sends a few % more
# packets. Must run before any track / DataChannel is created.
import os as _os
_RTC_MTU = int(_os.environ.get("RTC_MTU", "1000"))

# (1) RTP video packetizers (VP8 + H264). Module globals read at packetize time.
import aiortc.codecs.vpx as _vpx
import aiortc.codecs.h264 as _h264
_vpx.PACKET_MAX = _RTC_MTU
_h264.PACKET_MAX = _RTC_MTU

# (2) SCTP DataChannel (lidar + control). aiortc's USERDATA_MAX_LENGTH defaults to
# 1200 and was NEVER touched before -> every lidar message (16-39 KB = many chunks)
# had oversized chunks dropped, reliable retransmits looped forever, the send buffer
# stuck (buffered=108600 in the logs) -> no lidar at all. This is the missing half.
import aiortc.rtcsctptransport as _sctp
_sctp.USERDATA_MAX_LENGTH = _RTC_MTU

# (3) DTLS handshake. aiortc never calls set_ciphertext_mtu, so OpenSSL may fragment
# the Certificate flight to its own (larger) default. DTLS actually completes on the
# 1280 path in practice, but pin it too for safety/robustness on tighter paths.
# _do_handshake runs right after self._ssl exists (connect/accept state already set).
import aiortc.rtcdtlstransport as _dtls
_orig_do_handshake = _dtls.RTCDtlsTransport._do_handshake
async def _do_handshake_capped_mtu(self):
    try:
        if self._ssl is not None and hasattr(self._ssl, "set_ciphertext_mtu"):
            self._ssl.set_ciphertext_mtu(_RTC_MTU)
    except Exception as _e:  # noqa: BLE001 — other caps still apply if this fails
        print(f"[mtu] could not pin DTLS MTU ({_e!r}); continuing", flush=True)
    return await _orig_do_handshake(self)
_dtls.RTCDtlsTransport._do_handshake = _do_handshake_capped_mtu
print(f"[mtu] caps applied: RTP/SCTP/DTLS payload <= {_RTC_MTU} (RTC_MTU env)", flush=True)
# -----------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
HTTP_PORT = 8080
HTTPS_PORT = 8443
CERT_PATH = "/tmp/teleop_cert.pem"
KEY_PATH = "/tmp/teleop_key.pem"
CAMERA_TOPIC = "/oakd/rgb/preview/image_raw"
SPORT_REQUEST_TOPIC = "/api/sport/request"

# Two MID-360 stand-in lidars. Each cloud is rotated from its sensor frame into the
# robot BODY frame (x fwd, y left, z up) by its mount rotation, so the browser just
# does a fixed body->VR remap (no per-source rotation there).
# These rotations MUST MATCH the mount rpy in the Dockerfile xacro patch:
#   top    rpy = (0,      +PITCH, 0)  -> aims forward + DOWN
#   bottom rpy = (pi,     -PITCH, 0)  -> upside-down, aims forward + UP
# (yaw is 0 so "forward" is actually forward; verified by the forward-ray check.)
LIDAR_PITCH = 0.30  # rad, forward tilt; MUST MATCH the xacro

def _rpy_R(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return (Rz @ Ry @ Rx).astype(np.float32)  # sensor-vector -> body-vector

_SOURCE_ROT = {
    "lidar_top":    _rpy_R(0.0,     LIDAR_PITCH, 0.0),
    "lidar_bottom": _rpy_R(np.pi,  -LIDAR_PITCH, 0.0),
}

# Mount POSITIONS in shell_link frame (MUST MATCH the xacro origins). Without these
# each cloud would be drawn as if its sensor sat at the common origin, so the two
# sensors (12 cm apart in height) would each report the floor at a different z -> two
# ground planes. Translating by the mount offset puts both in one frame -> one floor.
_PLATE_Z   = 0.25257   # tower_sensor_plate_z_offset
_LIDAR_X   = 0.10      # forward to the plate's front edge
_LIDAR_GAP = 0.06      # +above / -below the plate
_SOURCE_T = {
    "lidar_top":    np.array([_LIDAR_X, 0.0, _PLATE_Z + _LIDAR_GAP], np.float32),
    "lidar_bottom": np.array([_LIDAR_X, 0.0, _PLATE_Z - _LIDAR_GAP], np.float32),
}

# Debug: cast a forward reference ray from each lidar (dense line of points along
# the sensor +x axis). After transform it shows, in VR, exactly where each lidar
# aims — top should point forward+down, bottom forward+up. Retired now that
# orientation is verified; set MARKER_RAY=1 to bring it back for re-checks.
MARKER_RAY = 0
_MARKER_PTS = np.stack([np.linspace(0.2, 3.0, 24),
                        np.zeros(24), np.zeros(24)], axis=1).astype(np.float32)

# Sources = list of (ros_topic, key). Default: the two lidars. Single depth-camera
# fallback via env:  CLOUD_TOPIC=/oakd/rgb/preview/depth/points CLOUD_FRAME=optical
if os.environ.get("CLOUD_TOPIC"):
    LIDAR_SOURCES = [(os.environ["CLOUD_TOPIC"], os.environ.get("CLOUD_FRAME", "optical"))]
else:
    LIDAR_SOURCES = [("/lidar/points", "lidar_top"), ("/lidar/points2", "lidar_bottom")]
LIDAR_KEYS = [k for _, k in LIDAR_SOURCES]   # order: [top, bottom] for the 2-lidar case

LIDAR_PERIOD_MS = 100   # 10 Hz (the future throttle / prediction knob)
CLOUD_MAX_PTS = 1500    # subsample target PER source (pre-cull)

# Self-hit cull: drop returns that land inside the robot's OWN body. The Create3
# base (r~0.17) is the widest part, and the standoffs / front OAK-D camera / sensor
# plate (r=0.137) all fit inside that radius, so ONE bounding cylinder about the
# body z-axis (shell_link frame), floor -> plate-top, covers the whole robot. Simple
# and robust; the only cost is a thin dead zone right against the body (which can't
# be seen into anyway). Replaces the old point-blank MIN_RANGE filter, so a close
# wall stays visible but the robot itself doesn't. (r_max^2, z_lo, z_hi).
_SELF_CYLS = (
    (0.18 ** 2, -0.10, 0.27),
)

def _cull_self(xyz):
    """Keep only points OUTSIDE the robot body (xyz already in shell_link frame)."""
    r2 = xyz[:, 0] ** 2 + xyz[:, 1] ** 2
    z = xyz[:, 2]
    inside = np.zeros(len(xyz), dtype=bool)
    for r2max, zlo, zhi in _SELF_CYLS:
        inside |= (r2 < r2max) & (z >= zlo) & (z <= zhi)
    return xyz[~inside]

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
            rgb = np.array(rgb, dtype=np.uint8)      # guaranteed-writable contiguous copy
            _stamp_frame(rgb, int(time.time() * 1000))   # capture-time watermark (in place)
            with SHARED.frame_lock:
                SHARED.rgb = rgb
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
            # forward-ray debug marker (in the SENSOR frame, before transform) so it
            # ends up showing exactly where this lidar aims.
            if MARKER_RAY and key in _SOURCE_ROT:
                xyz = np.concatenate([_MARKER_PTS, xyz], axis=0)
            if key == "optical":                         # depth camera
                ox, oy, oz = xyz[:, 0], xyz[:, 1], xyz[:, 2]
                xyz = np.stack([oz, -ox, -oy], axis=1)   # optical -> body frame
            elif key in _SOURCE_ROT:                     # lidar: sensor -> body
                xyz = xyz @ _SOURCE_ROT[key].T + _SOURCE_T[key]
            # now in shell_link frame: cull returns that hit the robot's own body.
            xyz = _cull_self(xyz)
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
# --- frame timestamp watermark (true glass-to-glass latency) ---
# A row of high-contrast blocks at top-left encodes the server CAPTURE time (low 24
# bits of unix ms). The browser decodes it at DISPLAY time and computes latency =
# (now + clock_offset) - stamp. Stamped at capture, so a stale/duplicated frame shows
# GROWING latency (exposes the fixed-cadence "fake fps"). Layout matched in rtc.js:
# 2 reference blocks (white, black) then 24 data bits MSB-first, square side `blk`.
STAMP_MARGIN = 4
STAMP_NBITS = 24
STAMP_NBLK = 2 + STAMP_NBITS

def _stamp_frame(rgb, t_ms):
    h, w = rgb.shape[:2]
    blk = max(6, int(w * 0.03))
    if STAMP_MARGIN + STAMP_NBLK * blk > w or STAMP_MARGIN + blk > h:
        return rgb                                   # frame too small to stamp
    val = int(t_ms) & ((1 << STAMP_NBITS) - 1)
    cols = [255, 0] + [255 if (val >> (STAMP_NBITS - 1 - i)) & 1 else 0
                       for i in range(STAMP_NBITS)]
    y0, y1 = STAMP_MARGIN, STAMP_MARGIN + blk
    for i, c in enumerate(cols):
        x0 = STAMP_MARGIN + i * blk
        rgb[y0:y1, x0:x0 + blk, :] = c
    return rgb


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


async def time_http(request):
    """Server clock in unix ms, for the browser's NTP-style offset estimation."""
    return web.json_response({"t": time.time() * 1000.0},
                             headers={"Cache-Control": "no-store"})


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


async def _diag_loop(pc, peer):
    """Per-peer WebRTC diagnostics. Makes the next connect attempt decisive:
       - dtlsState: 'connected' => DTLS made it (MTU handshake fix worked / wasn't
         needed). Stuck at 'connecting' => DTLS still blocked (handshake too big).
       - transport bytesSent climbing but the browser's remote-inbound
         packetsReceived ~0 / packetsLost high => media packets dropped (RTP too big
         for path MTU).
       - selected ICE pair shows whether we're on the IPv4 or IPv6 Tailscale path.
    Reads aiortc getStats (transport + outbound-rtp + remote-inbound-rtp)."""
    await asyncio.sleep(2.0)
    # one-shot: log the selected ICE pair (which path actually won)
    try:
        ice = pc.sctp.transport.transport._connection  # aioice Connection
        for comp, pair in getattr(ice, "_nominated", {}).items():
            lc, rc = pair.local_candidate, pair.remote_candidate
            print(f"[rtc {peer}] ICE pair comp{comp}: "
                  f"{lc.host}:{lc.port} ({lc.type}) -> {rc.host}:{rc.port} ({rc.type})",
                  flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[rtc {peer}] (could not read ICE pair: {e!r})", flush=True)
    while pc.connectionState not in ("closed", "failed"):
        try:
            report = await pc.getStats()
            dtls = vid = rin = None
            for s in report.values():
                t = getattr(s, "type", "")
                if t == "transport":
                    dtls = s
                elif t == "outbound-rtp" and getattr(s, "kind", "video") == "video":
                    vid = s
                elif t == "remote-inbound-rtp":
                    rin = s
            line = [f"[rtc {peer}] conn={pc.connectionState}"]
            if dtls is not None:
                line.append(f"dtls={getattr(dtls,'dtlsState','?')} "
                            f"tx={getattr(dtls,'bytesSent','?')}B "
                            f"rx={getattr(dtls,'bytesReceived','?')}B")
            if vid is not None:
                line.append(f"vid_sent={getattr(vid,'packetsSent','?')}pkt/"
                            f"{getattr(vid,'bytesSent','?')}B")
            if rin is not None:  # what the BROWSER reports back to us
                line.append(f"browser_recv_lost={getattr(rin,'packetsLost','?')} "
                            f"fracLost={getattr(rin,'fractionLost','?')} "
                            f"rtt={getattr(rin,'roundTripTime','?')}")
            print(" ".join(line), flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[rtc {peer}] stats err: {e!r}", flush=True)
        await asyncio.sleep(2.0)


async def offer(request):
    params = await request.json()
    pc = RTCPeerConnection()
    PCS.add(pc)
    peer = request.remote
    print(f"[rtc {peer}] offer received", flush=True)
    # per-connection state (so one client toggling the cloud doesn't affect others)
    pc_state = {"cloud_on": True}

    @pc.on("iceconnectionstatechange")
    async def _on_ice():
        print(f"[rtc {peer}] ICE -> {pc.iceConnectionState}", flush=True)

    @pc.on("datachannel")
    def on_datachannel(channel):
        if channel.label == "ctl":
            @channel.on("message")
            def on_message(message):
                try:
                    d = json.loads(message)
                    if "cloud" in d:                     # cloud on/off toggle
                        pc_state["cloud_on"] = bool(d["cloud"])
                    else:                                # drive command
                        SHARED.set_cmd(d.get("vx", 0), d.get("vy", 0), d.get("vyaw", 0))
                except Exception:  # noqa: BLE001
                    pass
        elif channel.label == "lidar":
            async def lidar_loop():
                sent = skipped = backpressured = 0
                tick = 0
                try:
                    while channel.readyState == "open":
                        tick += 1
                        # cloud off => send nothing, freeing the whole pipe for video
                        blob = SHARED.get_lidar() if pc_state["cloud_on"] else None
                        if blob:
                            # backpressure: don't queue faster than the channel drains,
                            # or the SCTP buffer bloats and the stream stalls for good.
                            if getattr(channel, "bufferedAmount", 0) > 2 * len(blob):
                                backpressured += 1
                            else:
                                try:
                                    channel.send(blob)
                                    SHARED.add_lidar_bytes(len(blob))
                                    sent += 1
                                except Exception as e:  # noqa: BLE001
                                    skipped += 1
                                    if skipped <= 5:
                                        print(f"[lidar-tx] send failed: {e!r}", flush=True)
                        # ~1 Hz heartbeat so we can SEE if/why the stream stalls
                        if tick % max(1, int(1000 / LIDAR_PERIOD_MS)) == 0:
                            print(f"[lidar-tx] state={channel.readyState} sent={sent} "
                                  f"skipped={skipped} backpressured={backpressured} "
                                  f"buffered={getattr(channel, 'bufferedAmount', '?')}", flush=True)
                            sent = skipped = backpressured = 0
                        await asyncio.sleep(LIDAR_PERIOD_MS / 1000.0)
                except Exception as e:  # noqa: BLE001 — loop dying == permanent freeze
                    print(f"[lidar-tx] loop CRASHED (permanent freeze): {e!r}", flush=True)
                print(f"[lidar-tx] loop exited; channel state={channel.readyState}", flush=True)
            asyncio.ensure_future(lidar_loop())

    @pc.on("connectionstatechange")
    async def on_state():
        print(f"[rtc {peer}] conn -> {pc.connectionState}", flush=True)
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            PCS.discard(pc)
            SHARED.set_cmd(0, 0, 0)  # safety: stop if peer drops

    pc.addTrack(CameraTrack())
    await pc.setRemoteDescription(
        RTCSessionDescription(sdp=params["sdp"], type=params["type"]))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    # DTLS state logging (the video sender + sctp share dtls transports). Attached
    # after setLocalDescription so the transports exist. Tells us exactly whether the
    # handshake completes -> isolates "DTLS blocked" from "media dropped".
    try:
        transports = [getattr(s, "transport", None) for s in pc.getSenders()]
        if pc.sctp is not None:
            transports.append(getattr(pc.sctp, "transport", None))
        for _t in {id(t): t for t in transports if t is not None}.values():
            @_t.on("statechange")
            def _on_dtls(tr=_t):
                print(f"[rtc {peer}] DTLS -> {tr.state}", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[rtc {peer}] (could not attach DTLS logging: {e!r})", flush=True)

    asyncio.ensure_future(_diag_loop(pc, peer))
    return web.json_response(
        {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})


def make_app():
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)
    app.router.add_get("/cmd", cmd_http)
    app.router.add_get("/stop", stop_http)
    app.router.add_get("/time", time_http)
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
