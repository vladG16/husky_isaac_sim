# SOP — Driving the Husky in Isaac Sim 5.1 via ROS 2 (Jazzy)

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
