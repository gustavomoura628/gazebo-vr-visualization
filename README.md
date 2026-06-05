# gazebo-vr-visualization

Quest VR teleoperation + live 3D point-cloud visualization for a simulated robot in
Ignition Gazebo. A **TurtleBot4** in Gazebo Fortress stands in for a **Unitree Go2W**,
carrying **two MID-360-style 3D lidars**; a WebRTC bridge streams the camera video and
the merged lidar cloud to a **WebXR** app you open in the Meta Quest browser, where you
see the cloud in true 1:1 metric scale around a to-scale model of the robot and drive it
with the thumbsticks.

Built on top of the upstream `go2w_people_search` simulation.

---

## How it works

```
Gazebo (gz-sim, OGRE2 on NVIDIA)
  ├─ 2× gpu_lidar (top + bottom, tilted)  ──gz→ROS bridge──▶ /lidar/points, /lidar/points2
  └─ OAK-D camera                         ──────────────────▶ image topic
                                                                   │
                        web_teleop/teleop_bridge.py (aiortc + aiohttp)
                          • merges + transforms both clouds into the robot body frame
                          • culls self-hits (points inside the robot's own body)
                          • subsamples, streams cloud over a WebRTC DataChannel
                          • streams camera as a WebRTC video track
                          • serves the WebXR page over HTTPS (WebXR needs a secure ctx)
                                                                   │
                                          Meta Quest browser  ◀────┘
                              web_teleop/vr.html (three.js / WebXR)
                                • instanced low-poly point cloud, lit, body-framed
                                • to-scale robot model + controller lasers
                                • thumbstick driving → control DataChannel → /cmd_vel
```

Coordinate frames: clouds are transformed into the robot **body frame** (x fwd, y left,
z up) in the bridge, so the VR app only applies one fixed body→VR remap. The viewpoint
sits at the robot, slightly back and above the sensor plate.

---

## Requirements

- **An NVIDIA GPU** (required). Gazebo renders the GPU lidars + camera with OGRE2 on
  the GPU via headless EGL; without a usable NVIDIA GL context it falls back to
  software (llvmpipe) and the sim crawls / sensors read zero. A modern discrete card
  is recommended — an old entry-level card (e.g. GTX 750 Ti) builds and runs but is
  too weak for acceptable frame rates.
- **Docker** + the **NVIDIA Container Toolkit** (for `--gpus all`).
- ROS 2 Humble + Ignition Gazebo Fortress — provided *inside* the Docker image, you
  don't install them on the host.
- A **Meta Quest** (or any WebXR headset) on the **same Wi-Fi** as the host.

### Setting up a fresh machine

These are the host prerequisites (everything else is in the image). Validated on
Ubuntu and Manjaro:

```bash
# 1. Docker + NVIDIA driver (use your distro's packages; verify the driver works):
nvidia-smi                      # must list your GPU

# 2. NVIDIA Container Toolkit (package name varies by distro):
#    Debian/Ubuntu:  sudo apt install nvidia-container-toolkit
#    Arch/Manjaro:   sudo pacman -S nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 3. Verify the GPU is reachable from a container:
docker run --rm --gpus all ubuntu nvidia-smi   # should print your GPU

# 4. Clone + run:
git clone https://github.com/gustavomoura628/gazebo-vr-visualization.git
cd gazebo-vr-visualization
./run_simulation.sh
```

**Sanity check that rendering is on the GPU** (not software): while the sim runs,
`nvidia-smi` should list **`ign gazebo server`** using GPU memory. If you instead see
`libEGL ... failed to create dri2 screen` / `llvmpipe` in the logs and the GPU is
idle, EGL didn't reach the NVIDIA card (`run_simulation.sh` pins it via
`__EGL_VENDOR_LIBRARY_FILENAMES`; ensure the toolkit step above succeeded).

---

## Quick start

```bash
# 1. Build the image and launch the simulation (sim + gz→ROS lidar bridges)
./run_simulation.sh

# 2. In another terminal, start the teleop/WebRTC bridge inside the running container
./run_teleop_bridge.sh
```

Then on the Quest browser open (replace with the host's LAN IP):

- **VR:**  `https://<host-ip>:8443/vr.html`  → tap **Enter VR**
- **2D preview:** `http://<host-ip>:8080/`

The page is served over **HTTPS with a self-signed cert** (WebXR requires a secure
context), so accept the browser's certificate warning the first time.

> Note: `run_simulation.sh` unlocks the Create3 base's full drive speed by setting
> `safety_override=full` on `/motion_control`. If the sim ever gets wedged, the reliable
> fix is a full clean restart (stop the container, relaunch, re-start the bridge).

---

## VR controls

| Input | Action |
|-------|--------|
| **Left stick** | drive forward / back |
| **Right stick** | turn |
| **A** (right) | cycle view: camera → both lidars → top → bottom |
| **X** (left) | cycle point shape: icosa / geodesic / dodeca / octa / cube |
| **Y** (left) | toggle flat / smooth shading |
| **B** (right) | toggle the robot model |
| Controller rays | point at things during demos |

---

## Repo layout

| Path | What |
|------|------|
| `Dockerfile` | Builds the sim image; injects the two 3D lidars + MID-360 FOV/range into the TurtleBot4 URDF and renders sensors on OGRE2 |
| `run_simulation.sh` | Build + run the sim container (NVIDIA GPU, X11 for the Gazebo GUI) |
| `run_teleop_bridge.sh` | Start the WebRTC/teleop bridge inside the running container |
| `web_teleop/teleop_bridge.py` | The bridge: ROS↔WebRTC, cloud transform/merge/cull, HTTPS signaling + static serving |
| `web_teleop/vr.html` | The WebXR app (three.js): point-cloud render, robot model, controls |
| `web_teleop/rtc.js` | Browser WebRTC setup (video track + data channels) |
| `packages/aog_simulation/launch/unitree_adapter_simulation.xml` | Launch: spawns the robot + the gz→ROS lidar point-cloud bridges |

See `TODO.md` and `WORKLOG.md` (in the parent project dir) for open work and a running
development log.

---

## Status

Working end-to-end: real GPU lidar in sim, two tilted 3D lidars merged into one
coherent body-frame cloud, single ground plane, self-body culling, in-headset driving,
and a live point-cloud visualization with a to-scale robot model.

Open directions (see `TODO.md`): blue-noise density sampling, a wrist-mounted VR
settings menu, lidar coloring (intensity / height / camera-fusion), bandwidth scaling
via range-image-over-codec streaming, and motion prediction on throttled points.
