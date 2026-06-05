// Shared WebRTC helper for the teleop pages.
// - Pulls the sim camera as a WebRTC video track (UDP, loss-tolerant).
// - Opens an unreliable/unordered "ctl" DataChannel for control (UDP-like,
//   decoupled from video, never head-of-line-blocked).
//
// connectWebRTC(videoEl, opts) -> { pc, getChannel() }
//   opts.onLidar(ArrayBuffer)  // called with each packed float32 XYZ scan blob
// sendControl(getChannel, vx, vy, vyaw)  // fire-and-forget; HTTP fallback

async function connectWebRTC(videoEl, opts = {}) {
  const pc = new RTCPeerConnection({
    // local LAN only; no STUN/TURN needed
    iceServers: [],
  });

  // Control channel: unreliable + unordered => lowest latency, latest-wins.
  const ctl = pc.createDataChannel('ctl', { ordered: false, maxRetransmits: 0 });

  // Lidar channel: server -> browser point blobs. Unreliable/unordered; we only
  // ever want the freshest scan, so a dropped one is fine.
  const lidar = pc.createDataChannel('lidar', { ordered: false, maxRetransmits: 0 });
  lidar.binaryType = 'arraybuffer';
  lidar.onmessage = (e) => { if (opts.onLidar) opts.onLidar(e.data); };

  // We only receive video.
  pc.addTransceiver('video', { direction: 'recvonly' });

  pc.ontrack = (e) => {
    if (videoEl && e.streams && e.streams[0]) {
      videoEl.srcObject = e.streams[0];
      videoEl.play().catch(() => {});
    }
  };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);

  const resp = await fetch('/offer', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type }),
  });
  const answer = await resp.json();
  await pc.setRemoteDescription(answer);

  return { pc, getChannel: () => ctl };
}

// Toggle the server-side point-cloud stream for THIS connection. When off the bridge
// stops sending cloud blobs entirely, so the video stream gets the full pipe — handy
// for a clean video-latency test. Sent over the ctl channel as {cloud: bool}.
function setCloud(getChannel, on) {
  const ch = getChannel();
  if (!ch || ch.readyState !== 'open') return false;
  // The ctl channel is UNRELIABLE (maxRetransmits:0), so a single toggle can be
  // dropped and never take effect. Send it a few times so at least one arrives.
  const msg = JSON.stringify({ cloud: !!on });
  let n = 0;
  const fire = () => { try { ch.send(msg); } catch (e) { /* noop */ } if (++n < 5) setTimeout(fire, 60); };
  fire();
  return true;
}

// Poll WebRTC stats ~1 Hz and report live video instrumentation:
//   fps, vkbps (video bitrate), tkbps (TOTAL bandwidth incl. cloud+control),
//   rttMs (network round-trip), jbMs (jitter-buffer delay), loss, w/h (resolution).
// Total vs video bitrate makes the point cloud's bandwidth cost directly visible.
function startStats(pc, onStats, intervalMs = 1000) {
  let prev = null;
  const timer = setInterval(async () => {
    let v = null, tr = null, pair = null;
    try {
      const stats = await pc.getStats();
      stats.forEach(r => {
        if (r.type === 'inbound-rtp' && (r.kind === 'video' || r.mediaType === 'video')) v = r;
        else if (r.type === 'transport') tr = r;
        else if (r.type === 'candidate-pair' && r.nominated) pair = r;
      });
    } catch (e) { return; }
    if (!v) return;
    const now = {
      t: performance.now(),
      vbytes: v.bytesReceived || 0,
      tbytes: (tr && tr.bytesReceived) || v.bytesReceived || 0,
      frames: v.framesDecoded || 0,
      jbd: v.jitterBufferDelay || 0,
      jbc: v.jitterBufferEmittedCount || 0,
    };
    let vkbps = 0, tkbps = 0, fps = v.framesPerSecond || 0;
    if (prev) {
      const dt = (now.t - prev.t) / 1000;
      if (dt > 0) {
        vkbps = (now.vbytes - prev.vbytes) * 8 / 1000 / dt;
        tkbps = (now.tbytes - prev.tbytes) * 8 / 1000 / dt;
        if (!v.framesPerSecond) fps = (now.frames - prev.frames) / dt;
      }
    }
    const dC = now.jbc - (prev ? prev.jbc : 0);
    const jbMs = dC > 0 ? (now.jbd - (prev ? prev.jbd : 0)) / dC * 1000
                        : (now.jbc > 0 ? now.jbd / now.jbc * 1000 : 0);
    prev = now;
    onStats({
      fps: Math.round(fps),
      vkbps, tkbps,
      rttMs: pair && pair.currentRoundTripTime != null ? pair.currentRoundTripTime * 1000 : null,
      jbMs,
      loss: v.packetsLost || 0,
      w: v.frameWidth || 0, h: v.frameHeight || 0,
    });
  }, intervalMs);
  return () => clearInterval(timer);
}

// --- glass-to-glass latency: clock sync + frame-stamp decoder ---
// Must match the server watermark layout in teleop_bridge.py (_stamp_frame).
const STAMP_MARGIN = 4, STAMP_NBITS = 24, STAMP_NBLK = 2 + STAMP_NBITS;

// NTP-style: estimate (server_clock - client_clock) in ms. Returns the offset to ADD
// to Date.now() to get server time. Picks the lowest-RTT sample of a few probes.
async function syncClock(samples = 7) {
  let best = null;
  for (let i = 0; i < samples; i++) {
    const t0 = Date.now();
    let j;
    try { j = await (await fetch('/time', { cache: 'no-store' })).json(); }
    catch (e) { continue; }
    const t1 = Date.now(), rtt = t1 - t0;
    const offset = (j.t + rtt / 2) - t1;     // server time at t1 minus client t1
    if (!best || rtt < best.rtt) best = { rtt, offset };
  }
  return best ? best.offset : null;
}

// Decode the server timestamp watermarked into each video frame. Reports, via
// onMeter({latencyMs, fps}): true capture->display latency (ms, smoothed) and the
// real CONTENT fps = rate of UNIQUE frame stamps (duplicated/re-paced frames carry
// the same stamp, so this is the genuine fresh-frame rate, not the encoded cadence).
function startLatencyMeter(videoEl, getOffset, onMeter) {
  if (!('requestVideoFrameCallback' in HTMLVideoElement.prototype)) return () => {};
  const cvs = document.createElement('canvas');
  const ctx = cvs.getContext('2d', { willReadFrequently: true });
  let ema = null, stop = false, lastVal = null, fresh = 0, fpsT = performance.now(), fps = 0;
  const frame = () => {
    if (stop) return;
    const w = videoEl.videoWidth, h = videoEl.videoHeight, off = getOffset();
    if (w && h) {
      const blk = Math.max(6, Math.floor(w * 0.03));
      const sw = STAMP_MARGIN + STAMP_NBLK * blk, sh = STAMP_MARGIN + blk;
      if (sw <= w && sh <= h) {
        cvs.width = sw; cvs.height = sh;
        ctx.drawImage(videoEl, 0, 0, sw, sh, 0, 0, sw, sh);
        const px = ctx.getImageData(0, 0, sw, sh).data;
        const cy = STAMP_MARGIN + (blk >> 1);
        const samp = bi => { const cx = STAMP_MARGIN + bi * blk + (blk >> 1),
                             o = (cy * sw + cx) * 4; return (px[o] + px[o+1] + px[o+2]) / 3; };
        const wRef = samp(0), bRef = samp(1), thr = (wRef + bRef) / 2;
        if (wRef - bRef > 40) {                       // a valid stamp is present
          let val = 0;
          for (let i = 0; i < STAMP_NBITS; i++) val = (val << 1) | (samp(2 + i) > thr ? 1 : 0);
          if (val !== lastVal) { fresh++; lastVal = val; }   // unique => a fresh frame
          if (off != null) {
            const est = Date.now() + off, span = 1 << STAMP_NBITS;
            let full = Math.round(est / span) * span + val;
            while (full > est + span / 2) full -= span;
            while (full < est - span / 2) full += span;
            const lat = est - full;
            if (lat >= -50 && lat < 60000) ema = ema == null ? lat : ema * 0.8 + lat * 0.2;
          }
        }
      }
    }
    const tnow = performance.now();
    if (tnow - fpsT >= 1000) { fps = fresh * 1000 / (tnow - fpsT); fresh = 0; fpsT = tnow; }
    onMeter({ latencyMs: ema, fps });
    videoEl.requestVideoFrameCallback(frame);
  };
  videoEl.requestVideoFrameCallback(frame);
  return () => { stop = true; };
}

// Send a command. Uses the DataChannel when open; falls back to a fire-and-forget
// HTTP GET so control still works if the channel isn't up yet.
function sendControl(getChannel, vx, vy, vyaw) {
  const ch = getChannel();
  const payload = JSON.stringify({ vx, vy, vyaw });
  if (ch && ch.readyState === 'open') {
    try { ch.send(payload); return 'rtc'; } catch (e) { /* fall through */ }
  }
  fetch(`/cmd?vx=${vx}&vy=${vy}&vyaw=${vyaw}`).catch(() => {});
  return 'http';
}
