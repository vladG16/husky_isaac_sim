# Setup & Prerequisites
 
This repository contains the **simulation logic and control code** — the Isaac Sim scene with its action graph, the ROS 2 navigator package, waypoint paths, and documentation. It does **not** redistribute the base robot model or the camera model; those come from their original sources (Clearpath and Stereolabs). This guide lists what to install and where to place the repository files.
 
> **Approach:** This is an "assemble-it" setup. You install the base dependencies from their official sources, clone this repo, place the USD scene, and re-resolve asset paths if needed. It is not a single-click package — the USD references assets by local path, so some path fixing on a fresh machine is expected (see Step 5).
 
---
 
## Prerequisites — install these first
 
### 1. NVIDIA Isaac Sim 5.1.0
Download and install from NVIDIA:
https://developer.nvidia.com/isaac-sim
 
This project was built and tested on **Isaac Sim 5.1.0**. Other 5.x versions may work but node names and fields can differ.
 
### 2. ROS 2 Jazzy (desktop)
Install the desktop variant on Ubuntu 24.04:
https://docs.ros.org/en/jazzy/Installation.html
 
```bash
# after adding the ROS 2 apt sources
sudo apt install ros-jazzy-desktop
```
 
Also useful for this project:
```bash
sudo apt install ros-jazzy-rqt-image-view ros-jazzy-teleop-twist-keyboard
```
 
### 3. Clearpath Husky description (the robot model source)
The Husky model in this project originates from Clearpath's open-source Husky description.
- Husky ROS packages: https://github.com/husky
- `husky_description` (URDF + meshes): https://github.com/husky/husky/tree/noetic-devel/husky_description
- Clearpath Husky tutorials: https://docs.clearpathrobotics.com/docs_robots/legacy/ros1_robots/outdoor_robots/husky/tutorials_husky
> The USD scene in this repo already contains the Husky geometry baked into the `configuration/` files, so you do **not** strictly need to re-import the URDF to run the provided scene. This reference is included so you know the model's origin and can regenerate or modify it if needed.
 
### 4. Stereolabs ZED Isaac Sim extension (the camera model source)
The ZED X camera model comes from the Stereolabs Isaac Sim extension. Install it so Isaac Sim can resolve the ZED model referenced by the scene:
- Extension repo: https://github.com/stereolabs/zed-isaac-sim
- Setup guide: https://docs.stereolabs.com/docs/integrations/isaac-sim/setting-up-the-zed-in-isaac-sim
Install via Isaac Sim's **Extension Manager → Third-Party tab** (search "ZED Camera"), or clone and build the repo and add its `exts` folder path in the Extension Manager settings.
 
> **Note:** This project uses the ZED **model** and publishes the camera over standard ROS 2 image topics via Isaac's own ROS2 Camera Helper — it does **not** require the ZED SDK. Installing the extension is only needed so the ZED X USD model resolves in the scene. (Full ZED-SDK depth streaming is a separate, heavier setup not used here.)
 
### 5. Isaac Sim built-in assets
The room environment in the scene is a standard Isaac Sim sample asset, resolved from Isaac's built-in asset library. No separate download — any working Isaac Sim install has access to it (an internet connection may be required the first time Isaac fetches Nucleus/sample assets).
 
---
 
## Setup — place the repository files
 
### 1. Clone this repository
```bash
git clone https://github.com/vladG16/husky_isaac_sim.git
cd husky_isaac_sim
```
 
### 2. Place the USD scene
Keep the `usd/` folder together as a unit — `husky_isaaclab.usd` references the files inside `configuration/`, so they must stay in the same relative structure:
```
usd/
├── husky_isaaclab.usd
└── configuration/
    ├── husky_isaaclab_base.usd
    ├── husky_isaaclab_physics.usd
    ├── husky_isaaclab_robot.usd
    └── husky_isaaclab_sensor.usd
```
You can keep this anywhere on disk; note the path for the next step.
 
### 3. Build the ROS 2 navigator package
Copy (or symlink) the `husky_sim_nav` package into your ROS 2 workspace and build:
```bash
cp -r husky_sim_nav ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select husky_sim_nav
source install/setup.bash
```
Package dependencies: `rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs` (declared in `package.xml`).
 
### 4. Open the scene in Isaac Sim
Launch Isaac Sim from a ROS-sourced terminal, then open `usd/husky_isaaclab.usd`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
~/isaacsim/isaac-sim.sh
```
 
### 5. Re-resolve asset paths (if needed)
The USD may reference the ZED model — and possibly other assets — by **absolute paths** from the machine it was authored on (e.g. `/home/<user>/...`). On a different machine these can fail to resolve, showing as missing geometry or errors in the console.
 
If the robot or camera geometry is missing when you open the scene:
- Select the affected prim in the **Stage** tree (e.g. `ZED_X`).
- In the **Property** panel, find the **References** / **Payloads** section and check the **Asset Path**.
- Update it to point at the asset's location on your machine (the ZED model from the Stereolabs extension's `data/usd` folder; the Husky geometry is inside this repo's `configuration/` files).
Alternatively, to avoid path issues entirely, you can flatten the scene into a single self-contained file on your machine via **File → Export** (Flatten), though this produces a larger file and is not required to run the project.
 
---
 
## Verify it works
 
With the scene open and **Play** pressed, in a ROS-sourced terminal:
```bash
ros2 topic list
```
Expected topics:
```
/cmd_vel      # drive commands (in)
/odom         # ground-truth pose (out)
/zed/image    # ZED left-camera RGB (out)
```
 
Drive it:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
View the camera:
```bash
ros2 run rqt_image_view rqt_image_view   # select /zed/image
```
Run autonomous navigation (from a folder containing a `waypoints.csv`):
```bash
ros2 run husky_sim_nav waypoint_nav
```
 
See `README.md` for the full pipeline documentation, node wiring, waypoint format, and troubleshooting.
 
---

