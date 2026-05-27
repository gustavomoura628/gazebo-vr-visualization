// Shared WebRTC helper for the teleop pages.
// - Pulls the sim camera as a WebRTC video track (UDP, loss-tolerant).
// - Opens an unreliable/unordered "ctl" DataChannel for control (UDP-like,
//   decoupled from video, never head-of-line-blocked).
//
// connectWebRTC(videoEl) -> { pc, getChannel() }
// sendControl(getChannel, vx, vy, vyaw)  // fire-and-forget; HTTP fallback

async function connectWebRTC(videoEl) {
  const pc = new RTCPeerConnection({
    // local LAN only; no STUN/TURN needed
    iceServers: [],
  });

  // Control channel: unreliable + unordered => lowest latency, latest-wins.
  const ctl = pc.createDataChannel('ctl', { ordered: false, maxRetransmits: 0 });

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
