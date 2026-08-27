# SOP — Driving the Husky in Isaac Sim 5.1 via ROS 2 (Jazzy)

# New here? Start with SETUP.md for prerequisites and installation, then come back here for how the pipeline works.

Standard operating procedure for commanding the Husky's wheels in Isaac Sim from a ROS 2 `/cmd_vel` publisher, using an OmniGraph action graph (ROS2 Subscribe Twist → Differential Controller → Articulation Controller).

Platform: Isaac Sim 5.1.0, ROS 2 Jazzy, Dell workstation (Ubuntu 24.04).

---

## Part A — One-time scene setup

Do this once per scene. If the graph and joint drives are already saved in your `.usd`, skip to Part B.

### A1. Wheel joint drives (THE critical step)

Each of the four wheel joints needs a **velocity drive** or the wheels will not respond to any command, no matter how the graph is wired.

For each joint in the `joints` folder:
- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

Set, under **Angular Drive**:
- **Stiffness** = `0`
- **Damping** = `15000`

Stiffness 0 + nonzero damping = pure velocity control (what a wheel needs). Damping 0 silently makes the wheel ignore all velocity commands.

### A2. Action graph nodes

Build these nodes in the Action Graph:

1. **On Playback Tick** — fires every frame while playing.
2. **ROS2 Context** — leave defaults.
3. **ROS2 Subscribe Twist** — set **Topic Name** = `cmd_vel`.
4. **Break 3-Vector** (×2) — one for linear velocity, one for angular velocity.
5. **Differential Controller** — converts linear/angular into wheel speeds.
6. **Articulation Controller** — drives the actual joints.

### A3. Wiring

```
On Playback Tick : Tick          -> ROS2 Subscribe Twist : Exec In
On Playback Tick : Delta Seconds -> Differential Controller : Dt   (see note*)
ROS2 Context     : Context       -> ROS2 Subscribe Twist : Context

ROS2 Subscribe Twist : Linear Velocity  -> Break 3-Vector (linear)  : Vector
ROS2 Subscribe Twist : Angular Velocity -> Break 3-Vector (angular) : Vector
ROS2 Subscribe Twist : Exec Out         -> Differential Controller  : Exec In

Break 3-Vector (linear)  : X -> Differential Controller : Desired Linear Velocity
Break 3-Vector (angular) : Z -> Differential Controller : Desired Angular Velocity

Differential Controller : Velocity Command -> Articulation Controller : Velocity Command
Differential Controller : Exec Out          -> Articulation Controller : Exec In
```

*Note on Dt: if the `deltaSeconds` output reads 0 and you get an "Invalid deltaTime 0.000000" error, disconnect the Dt wire and type a fixed value `0.016` into the Dt field instead. A constant frame time is fine for velocity control.

### A4. Differential Controller parameters

| Field | Value | Notes |
|---|---|---|
| Wheel Radius | `0.1651` | meters |
| Wheel Distance | `0.5708` | track width, meters |
| Max Linear Speed | `1.0` | MUST be > commanded speed; defaults to 0 which clamps output to 0 |
| Max Wheel Speed | `10.0` | rad/s |
| Max Angular Speed | `2.0` | |
| Max Acceleration | `10.0` | |
| Max Angular Acceleration | `10.0` | |
| Max Deceleration | `10.0` | |

The **Max** fields are hard ceilings and default to `0`. A max of 0 clamps all output to zero — a very common "wired correctly but nothing moves" cause.

### A5. Articulation Controller parameters

- **targetPrim** = `/husky` (if wheels don't drive, try `/husky/base_link`)
- **jointNames** = all four, in order:
  - `front_left_wheel_joint`
  - `rear_left_wheel_joint`
  - `front_right_wheel_joint`
  - `rear_right_wheel_joint`
- Feed the wheel velocities into **Velocity Command** (not Position Command, not Effort Command).

---

## Part B — Running it (the part you forgot)

### B1. Launch Isaac Sim (sourced correctly)

Always start Isaac Sim from a terminal that has ROS 2 and your workspace sourced, or the ROS bridge won't connect:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
# then launch Isaac Sim from this same terminal
```

### B2. Open the scene and PRESS PLAY

Open your saved `.usd`, then press the **Play** button (triangle, top-left toolbar). Nothing publishes or subscribes until the sim is actively **playing** (toolbar shows the pause icon = two bars).

### B3. Confirm the topic exists

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

You should see `/cmd_vel` in the list. If not, the graph isn't running the ROS side — recheck the sim is playing and the Subscribe Twist node exists.

### B4. Drive the wheels

**Drive straight forward** (0.5 m/s):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 20
```

**Turn in place** (rotate, no forward motion):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 20
```

**Drive in an arc** (forward + turn):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}" -r 20
```

**Stop** (either Ctrl+C the publisher, or send zeros):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 20
```

- `-r 20` republishes at 20 Hz. Leave the command running; the wheels drive as long as messages arrive.
- `linear.x` = forward speed (m/s). `angular.z` = yaw/turn rate (rad/s). For a skid-steer, those are the only two that matter.

---

## Troubleshooting quick reference

| Symptom | Cause | Fix |
|---|---|---|
| Wheels don't move, no error | Joint damping = 0 | Set Angular Drive Damping = 15000, Stiffness = 0 on all wheel joints |
| Wheels don't move, no error | Max Linear/Wheel Speed = 0 on Diff Controller | Set them above your commanded speed |
| "Invalid deltaTime 0.000000" error | `deltaSeconds` outputting 0 | Disconnect Dt wire, type `0.016` in Dt field |
| "shape mismatch (1,2) vs (1,4)" | Diff Controller outputs 2 values, jointNames lists 4 | List matching joints, or use array nodes to expand [L,R] → [L,L,R,R] |
| `/cmd_vel` not in topic list | Sim not playing, or bridge not sourced | Press Play; relaunch Isaac Sim from a ROS-sourced terminal |
| Only front wheels spin | Rear joints have damping 0, or missing from jointNames | Add rear joints to jointNames + set their damping |
| Publisher shows `RTPS_TRANSPORT_SHM Error ... port7001` | Shared-memory transport hiccup | Harmless — it falls back to UDP and still works |

---

## Key Husky parameters

- Wheel radius: `0.1651` m
- Wheel distance (track width): `0.5708` m
- Wheel joints: `front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, `rear_right_wheel_joint`

- File name: husky_isaaclab.usd (path:home,lab,downloads,husky)

- # Husky Isaac Sim — ROS 2 Digital Twin

Autonomous Clearpath Husky A200 in NVIDIA Isaac Sim, driven and sensed entirely through ROS 2. This repository documents the full simulation pipeline: velocity control, ground-truth localization, autonomous waypoint navigation, and a simulated ZED stereo camera.

**Platform:** Isaac Sim 5.1.0 · ROS 2 Jazzy · Ubuntu 24.04 · Clearpath Husky A200
**Scene file:** `husky_isaaclab.usd` (currently at `~/Downloads/husky/`)

---

## What this pipeline provides

Four ROS 2 interfaces connect an external control stack to the simulated robot:

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | in → sim | commanded linear & angular velocity |
| `/odom` | `nav_msgs/Odometry` | sim → out | ground-truth robot pose |
| `/zed/image` | `sensor_msgs/Image` | sim → out | ZED left-camera RGB view |
| `/navigation_status` | `std_msgs/String` | out | navigator state (optional) |

Everything below builds one of these interfaces. Once built and saved into the `.usd`, the scene setup is one-time; day-to-day use is just Part E (running it).

---

# PART A — Wheel drive: `/cmd_vel` → wheels

The OmniGraph chain that turns an incoming velocity command into wheel motion:
ROS2 Subscribe Twist → Break Vectors → Differential Controller → Articulation Controller.

### A1. Wheel joint drives (THE critical step)
Each of the four wheel joints needs a **velocity drive** or the wheels will not respond to any command, no matter how the graph is wired.

For each joint in the `joints` folder (`front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, `rear_right_wheel_joint`), set under **Angular Drive**:
- **Stiffness** = `0`
- **Damping** = `15000`

Stiffness 0 + nonzero damping = pure velocity control (what a wheel needs). Damping 0 silently makes the wheel ignore all velocity commands.

### A2. Action graph nodes
1. **On Playback Tick** — fires every frame while playing.
2. **ROS2 Context** — leave defaults.
3. **ROS2 Subscribe Twist** — set Topic Name = `cmd_vel`.
4. **Break 3-Vector (×2)** — one for linear velocity, one for angular velocity.
5. **Differential Controller** — converts linear/angular into wheel speeds.
6. **Articulation Controller** — drives the actual joints.

### A3. Wiring
```
On Playback Tick : Tick          -> ROS2 Subscribe Twist : Exec In
On Playback Tick : Delta Seconds -> Differential Controller : Dt   (see note*)
ROS2 Context     : Context       -> ROS2 Subscribe Twist : Context

ROS2 Subscribe Twist : Linear Velocity  -> Break 3-Vector (linear)  : Vector
ROS2 Subscribe Twist : Angular Velocity -> Break 3-Vector (angular) : Vector
ROS2 Subscribe Twist : Exec Out         -> Differential Controller  : Exec In

Break 3-Vector (linear)  : X -> Differential Controller : Desired Linear Velocity
Break 3-Vector (angular) : Z -> Differential Controller : Desired Angular Velocity

Differential Controller : Velocity Command -> [Array mapping, see A6] -> Articulation Controller : Velocity Command
Differential Controller : Exec Out          -> Articulation Controller : Exec In
```
*Note on Dt: if the `deltaSeconds` output reads 0 and you get an "Invalid deltaTime 0.000000" error, disconnect the Dt wire and type a fixed value `0.016` into the Dt field instead. A constant frame time is fine for velocity control.

### A4. Differential Controller parameters
| Field | Value | Notes |
|---|---|---|
| Wheel Radius | `0.1651` | meters |
| Wheel Distance | `0.5708` | track width, meters |
| Max Linear Speed | `1.0` | MUST be > commanded speed; defaults to 0 which clamps output to 0 |
| Max Wheel Speed | `10.0` | rad/s |
| Max Angular Speed | `2.0` | |
| Max Acceleration | `10.0` | |
| Max Angular Acceleration | `10.0` | |
| Max Deceleration | `10.0` | |

The **Max** fields are hard ceilings and default to `0`. A max of 0 clamps all output to zero — a very common "wired correctly but nothing moves" cause.

### A5. Articulation Controller parameters
- **targetPrim** = `/husky` (if wheels don't drive, try `/husky/base_link`)
- **jointNames** = all four, in order: `front_left_wheel_joint`, `rear_left_wheel_joint`, `front_right_wheel_joint`, `rear_right_wheel_joint`
- Feed the wheel velocities into **Velocity Command** (not Position Command, not Effort Command).

### A6. Array mapping (skid-steer: 2 values → 4 wheels)
The Differential Controller outputs **2** values `[left, right]`, but the Husky has **4** wheel joints. Without mapping, this throws `shape mismatch (1,2) vs (1,4)`.

Insert between the Differential Controller and the Articulation Controller:
- **Get Array Index** (×2): both take the Diff Controller's Velocity Command as `Array`; set `Index` = `0` (left) on one, `Index` = `1` (right) on the other.
- **Make Array** (4 inputs): wire so it produces `[left, left, right, right]` to match jointNames order `[FL, RL, FR, RR]` — Get Array Index[0] → input0 & input1, Get Array Index[1] → input2 & input3.
- **Make Array : Array output** → Articulation Controller : Velocity Command.

---

# PART B — Localization: sim → `/odom`

Publishes the robot's ground-truth pose as odometry so the navigator can localize. Two nodes, chained: Isaac Compute Odometry → ROS2 Publish Odometry (mirrors the producer→publisher pattern).

### B1. Add the nodes
- **Isaac Compute Odometry Node** — reads a prim's true world transform.
- **ROS2 Publish Odometry** — packages that pose into a ROS Odometry message.

### B2. Configure Isaac Compute Odometry
- **chassisPrim** → `/husky/base_link`
  **(NOT `/husky`** — the top prim isn't a rigid body / articulation root and will error with "not a valid rigid body or articulation root".)

### B3. Configure ROS2 Publish Odometry
- **odomFrameId** = `odom`
- **chassisFrameId** = `base_link`
- **topicName** = `odom`
- **Context** ← the same ROS2 Context node used by Subscribe Twist

### B4. Wiring
```
On Playback Tick : Tick -> Isaac Compute Odometry : Exec In
Isaac Compute Odometry : Exec Out -> ROS2 Publish Odometry : Exec In

Isaac Compute Odometry : Position         -> ROS2 Publish Odometry : Position
Isaac Compute Odometry : Orientation      -> ROS2 Publish Odometry : Orientation
Isaac Compute Odometry : Linear Velocity  -> ROS2 Publish Odometry : Linear Velocity
Isaac Compute Odometry : Angular Velocity -> ROS2 Publish Odometry : Angular Velocity
```

### B5. Verify
Sim playing:
```bash
ros2 topic list          # expect /odom
ros2 topic echo /odom --once | grep -A3 position   # position values, changing as robot moves
```

---

# PART C — ZED stereo camera: sim → `/zed/image`

Mounts a simulated ZED X on the robot and publishes its camera view over ROS 2. Uses the Stereolabs ZED X **USD model** (accurate geometry + stereo camera prims) but publishes through **Isaac's own ROS2 Camera Helper** — no ZED SDK required.

> Note: this is the ROS-topic ("Path A") approach. It gives the real ZED body/optics and a live camera feed over ROS. It does **not** use the ZED SDK's depth processing — that would require installing the ZED SDK and using Stereolabs' ZED Camera Helper node (a separate, heavier setup). Isaac's rendering is used for the image here.

### C1. Import and mount the ZED model
1. Obtain the ZED X USD from the Stereolabs Isaac Sim extension (`sl.sensor.camera`, file `ZED_X.usdc` in the extension's `data/usd` folder). Install the extension via Isaac's Extension Manager → Third-Party tab, or clone `github.com/stereolabs/zed-isaac-sim` and add its `exts` path.
2. Drag `ZED_X` into the scene. **Save.**
3. In the Stage tree, **drag `ZED_X` onto `base_link`** so it nests underneath (path becomes `/husky/base_link/ZED_X/...`). This rigidly attaches it to the robot. **Save.**
4. Position it: select `ZED_X`, in the Transform section set translate to the front of the robot (measure the real mount for twin fidelity; +X is forward). **Save.**

The model contains stereo camera prims `CameraLeft` and `CameraRight` (correctly placed for the real ZED's optics).

### C2. Add the render + publish nodes
Isaac's ROS2 Camera Helper needs a **render product** — pointing it at a raw camera prim alone will NOT publish. Chain: Create Render Product → ROS2 Camera Helper.

- **Isaac Create Render Product**
  - **cameraPrim** → full CameraLeft path, e.g. `/husky/base_link/ZED_X/base_link/ZED_X/CameraLeft`
  - **Exec In** ← On Playback Tick : Tick
- **ROS2 Camera Helper**
  - **renderProductPath** ← Create Render Product : renderProductPath **output** (wire it; don't type the camera path)
  - **topicName** = `/zed/image`
  - **type** = `rgb`
  - **frameId** = `zed_left_camera`
  - **Exec In** ← On Playback Tick : Tick

**Save.**

### C3. Verify and view
Sim playing:
```bash
ros2 topic list            # expect /zed/image
ros2 topic hz /zed/image   # expect ~25-60 Hz
ros2 run rqt_image_view rqt_image_view
```
In the rqt window, click the topic dropdown (top-left) and select `/zed/image`. If it's not listed, click the refresh button first. The camera view appears.
(Install viewer if needed: `sudo apt install ros-jazzy-rqt-image-view`)

> **GPU note:** camera rendering is GPU-heavy. If Isaac crashes with "Failed to create any GPU devices," lower the camera resolution / FPS, use only one camera, and save often. A driver/library version mismatch (after a background NVIDIA update) also causes this — fix with a reboot, then `nvidia-smi` to confirm the GPU is healthy.

---

# PART D — Autonomous waypoint navigation

The `husky_sim_nav` ROS 2 package drives the robot through a list of waypoints using a PD controller + state machine. It subscribes to `/odom` (Part B) and publishes to `/cmd_vel` (Part A) — closing the loop.

### D1. Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select husky_sim_nav
source install/setup.bash
```
Package dependencies: `rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs`.

### D2. Create a waypoints file
The node reads `waypoints.csv` from the current working directory. One waypoint per line: `x,y,theta` (meters, meters, radians), in the `/odom` frame (origin = robot's spawn pose). Lines starting with `#` are comments.

Simple first path (drive forward ~1.5 m):
```bash
cd ~
echo "2.0,0.0,0.0" > waypoints.csv
```

### D3. Run
```bash
# Isaac playing, /odom and /cmd_vel confirmed live
cd ~                                   # dir containing waypoints.csv
ros2 run husky_sim_nav waypoint_nav
```
Expected log: `Loaded N waypoints` → `Initial robot pose: ...` → state transitions (ALIGNING → MOVING → ...) → `All waypoints successfully navigated!`

### D4. Changing paths (no rebuild)
Waypoints are read at startup, not compiled in. To run a different path: edit/replace `waypoints.csv` and re-run the node. **Only rebuild when you change the Python code** (speeds, gains, tolerances).

### D5. Tuning notes
- Getting stuck aligning on a waypoint → loosen tolerances: `goal_final_theta_tolerance` (e.g. 0.20), `goal_xy_tolerance` (e.g. 0.15), `approach_heading_tolerance` (e.g. 0.3).
- Too slow → raise `max_linear_speed` (e.g. 0.5) and `max_angular_speed` (e.g. 0.8); keep them under the Diff Controller's Max ceilings.
- First waypoint should be a clear distance from the start (a too-close first point causes heading jitter).

---

# PART E — Daily use (running an existing scene)

Once the `.usd` and package are built, normal operation:

**1. Launch Isaac Sim (ROS-sourced):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
~/isaacsim/isaac-sim.sh
```
Open `husky_isaaclab.usd`, press **Play** (toolbar shows pause icon = two bars = playing).

**2. Verify interfaces (second terminal):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list      # expect /cmd_vel /odom /zed/image
```

**3. Drive it — pick one:**

Manual velocity:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 20
```

Keyboard teleop (install: `sudo apt install ros-jazzy-teleop-twist-keyboard`):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Keys: `i` forward, `,` back, `j`/`l` turn, `k` stop, `z`/`x` adjust speed. The teleop terminal must have keyboard focus.

Autonomous navigation:
```bash
cd ~   # dir with waypoints.csv
ros2 run husky_sim_nav waypoint_nav
```

**4. View the camera:**
```bash
ros2 run rqt_image_view rqt_image_view   # select /zed/image
```

---

## Troubleshooting quick reference

| Symptom | Cause | Fix |
|---|---|---|
| Wheels don't move, no error | Joint damping = 0 | Angular Drive: Damping 15000, Stiffness 0 on all wheel joints |
| Wheels don't move, no error | Diff Controller Max speeds = 0 | Set Max Linear/Wheel/etc. above commanded speed |
| "Invalid deltaTime 0.000000" | `deltaSeconds` outputs 0 | Disconnect Dt wire, type `0.016` in Dt field |
| "shape mismatch (1,2) vs (1,4)" | 2 controller outputs vs 4 joints | Add Get Array Index ×2 + Make Array to expand [L,R]→[L,L,R,R] (Part A6) |
| Robot drives with no command (phantom motion) | Controller ticked only on message; holds last output | Tick Differential Controller from On Playback Tick every frame |
| `/odom` not publishing | Compute Odometry chassisPrim wrong | Set chassisPrim = `/husky/base_link`, not `/husky` |
| `/zed/image` not publishing | renderProductPath points at raw camera | Add Isaac Create Render Product (cameraPrim = CameraLeft) → wire its output into Camera Helper renderProductPath |
| rqt shows blank | No topic selected | Click dropdown (top-left), refresh, select `/zed/image` |
| Topic missing entirely | Sim not playing, or bridge not sourced | Press Play; launch Isaac from a ROS-sourced terminal |
| "Failed to create any GPU devices" | GPU/driver mismatch (background NVIDIA update) or VRAM exhausted | Reboot; `nvidia-smi` to confirm; lower camera res/FPS |
| `RTPS_TRANSPORT_SHM Error ... port7001` | Shared-memory transport hiccup | Harmless — falls back to UDP |

---

## Key Husky parameters
- Wheel radius: `0.1651` m
- Wheel distance (track width): `0.5708` m
- Wheel joints: `front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, `rear_right_wheel_joint`
- Scene file: `husky_isaaclab.usd`

## Repository contents
| Path | Purpose |
|---|---|
| `usd/husky_isaaclab.usd` | Isaac Sim scene: drive + odometry + camera action graph |
| `husky_sim_nav/` | ROS 2 package — PD waypoint navigator |
| `waypoints/*.csv` | Demo paths (square, octagon, zigzag) |
| `README.md` | This document |

> **USD path dependency:** the `.usd` may reference meshes (URDF, ZED model) by absolute local paths. On a fresh machine these must be re-resolved. Keep referenced assets alongside the USD or document their locations.
