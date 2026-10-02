# RoboSub simulation

Build and run Duke RoboSub environments with Stonefish 1.6 and ROS 2 Jazzy, matching the ROS distribution in [`robosub-ros2`](../robosub-ros2).

The scene contains a configurable outdoor pool and a teleoperable Crush prototype built from the supplied CAD export. The default pool interior is **25 m × 15 m × 3 m deep**. These are development dimensions; replace them with measurements for the pool you want to reproduce.

## View the pool on Wayland, Windows or macOS

Install Docker Engine with Compose on Linux, or Docker Desktop with Linux containers on Windows/macOS. Run these commands from `robosub-simulation`. Initialize the existing Stonefish submodule if your checkout lacks its source:

```bash
git submodule update --init --recursive
docker compose up --build
```

Open **[http://localhost:8080](http://localhost:8080)** in your browser. The viewer streams the actual Stonefish window and forwards mouse and keyboard input. You do not need host display mounts, `xhost`, a Linux desktop, or a host GPU. KDE, GNOME, Wayland and X11 use the same browser interface; Windows and macOS run the simulator inside Docker's Linux VM. The image uses Mesa software OpenGL by default. The viewer has been tested on Linux with an AMD64 container; Windows, macOS and ARM64 builds still need verification on those hosts.

The first build compiles Stonefish and its ROS interface. Use `docker compose up -d` to run in the background, `docker compose logs -f simulation` to inspect logs, and `docker compose down` to stop it. The viewer port binds to the local machine.

### Drive the robot

Click the viewport, then hold the keys below. Motion follows the robot's current body axes, so forward follows its heading. Release a key to request zero velocity on that axis. You can hold several keys together.

| Keys | Motion |
| --- | --- |
| W / S | Forward / backward |
| A / D | Strafe left / right |
| Q / E | Rise / dive |
| Left / right arrows | Yaw left / right |
| Up / down arrows | Pitch nose up / down |
| Z / X | Roll left / right |
| Space or **Stop** | Request zero velocity on all six axes |

The provisional controller uses 0.45 m/s translation and 0.5 rad/s rotation. It compares requested velocity with built-in ground truth and allocates a bounded wrench across eight built-in Stonefish thrusters. The robot moves through forces and torques; teleop does not overwrite its pose. The controller brakes motion after key release; Space does not teleport the vehicle or erase momentum.

Browser focus loss clears input, and key pulses expire after 500 ms if the browser disconnects. The mixer cuts thruster setpoints after 350 ms without a valid command or 500 ms without ground truth. Each actuator also has Stonefish's 400 ms watchdog.

### Camera controls

- **Follow robot / F:** follow Crush at close range. This is the initial view.
- **Surface view / R:** frame the whole pool from above.
- **Underwater / U:** move the view into the water.
- **Right mouse drag:** orbit. **Middle mouse drag:** pan. **Scroll:** zoom.
- **H:** show or hide Stonefish's inspector. **Fullscreen:** enlarge the browser viewport. **Reconnect:** restore the stream after a connection interruption.

Camera mouse controls remain separate from robot keyboard controls. Follow mode tracks position while you orbit and zoom; switch to Surface or Underwater for a camera fixed in the pool.

### Robot XML and built-in sensors

[`scenarios/crush.xml`](src/robosub_simulation/scenarios/crush.xml) defines the body, buoyancy proxy, thrusters and sensors. Launch generates this XML from [`config/crush.yaml`](src/robosub_simulation/config/crush.yaml) and the CAD-derived [`crush_model.yaml`](src/robosub_simulation/config/crush_model.yaml), includes it in the pool scene, and adjusts buoyancy to the configured water density. Edit the YAML to change the provisional layout or controller settings; regenerate the checked-in XML with:

```bash
PYTHONPATH=src/robosub_simulation python3 -m robosub_simulation.robot \
  --config src/robosub_simulation/config/crush.yaml \
  --output src/robosub_simulation/scenarios/crush.xml
```

The CAD visual keeps the exported dimensions (about 0.616 × 0.652 × 0.380 m). The simulation uses a simplified visual mesh, a closed collision box, the exported 14.456 kg mass and full inertia tensor, and an internal neutral-buoyancy proxy. These buoyancy and drag settings need physical calibration. The eight-thruster layout supports six independent axes; it differs from the six-thruster hardware layout in `robosub-ros2`.

| Topic | Type / purpose |
| --- | --- |
| `/simulation/crush/cmd_vel` | `geometry_msgs/msg/Twist`, desired body velocity in forward/right/down axes |
| `/simulation/crush/thruster_setpoints` | `std_msgs/msg/Float64MultiArray`, eight normalized rotor setpoints in XML order |
| `/simulation/crush/thruster_state` | `stonefish_ros2/msg/ThrusterState`, actual rotor speeds and thrust |
| `/simulation/crush/ground_truth` | `nav_msgs/msg/Odometry`, pose in NED; twist in body axes at the centre of mass |
| `/simulation/crush/imu` | `sensor_msgs/msg/Imu` |
| `/simulation/crush/pressure` | `sensor_msgs/msg/FluidPressure` |
| `/simulation/crush/dvl` | `stonefish_ros2/msg/DVL` |
| `/simulation/crush/dvl/altitude` | `sensor_msgs/msg/Range`, distance to the floor |
| `/simulation/crush/camera/front/image_color` | `sensor_msgs/msg/Image`, built-in 320 × 240 camera at 5 Hz |

These are raw simulator topics in Stonefish's frame conventions. Production topic names and frame conversions for `robosub-ros2` remain a later integration step. Headless launch omits the camera while keeping the scalar sensors. Use `keyboard_teleop:=false` when commanding body velocities from a different ROS node, or `enable_teleop:=false` to disable the prototype controller and command thruster setpoints directly. Use `spawn_robot:=false` for the original pool-only scene. Compose mounts the configuration directory; edit the robot YAML and restart the service to apply it.

### Display settings

The default virtual display is 1280 × 720 with low rendering quality and a 20 Hz rendering limit. The browser scales that display to fit your window. Software rendering speed depends on available CPU; the physics update rate remains a separate setting.

Override `VIEW_WIDTH`, `VIEW_HEIGHT`, `RENDER_QUALITY` or `RENDER_RATE` in the Compose environment, or add them to a local `.env` file:

```dotenv
VIEW_WIDTH=960
VIEW_HEIGHT=540
RENDER_QUALITY=low
RENDER_RATE=15
SIMULATION_PORT=8080
BUILD_JOBS=4
```

Use `docker compose up -d --force-recreate` after changing these settings. You can also run the built image without Compose:

```bash
docker run --rm -it --init -p 127.0.0.1:8080:8080 --shm-size=512m robosub-simulation:pool
```

### Headless physics and native desktop viewing

Start physics without the browser or a display:

```bash
docker run --rm -it --network host robosub-simulation:pool \
  ros2 launch robosub_simulation pool.launch.py headless:=true
```

The browser viewer is the portable default. To use a native Linux desktop window with X11 or XWayland and a Mesa GPU, allow the container's root user to access your display, then launch:

```bash
xhost +si:localuser:root
docker run --rm -it --network host \
  --device /dev/dri \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  robosub-simulation:pool ros2 launch robosub_simulation pool.launch.py
xhost -si:localuser:root
```

Stonefish rendering requires OpenGL 4.3. For NVIDIA, configure NVIDIA Container Toolkit and use `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all` in place of `--device /dev/dri`. Headless mode runs physics and scalar sensors; camera and imaging sonar simulation need rendering.

The container build applies [`patches/stonefish-ocean-shading.patch`](patches/stonefish-ocean-shading.patch) to its copy of Stonefish. The patch links the shading model that the ocean shader references, fixing Mesa's unresolved `ShadingModel` error. The source submodule stays unchanged.

## Configure the pool

Edit [`config/pool.yaml`](src/robosub_simulation/config/pool.yaml) and restart with `docker compose restart simulation`. Compose mounts this file, so pool edits do not require an image rebuild. You can also override dimensions at launch:

```bash
docker run --rm -it --network host robosub-simulation:pool \
  ros2 launch robosub_simulation pool.launch.py \
  headless:=true pool_length:=50 pool_width:=25 pool_depth:=4
```

To load your own YAML without rebuilding, mount it and pass `pool_config`:

```bash
docker run --rm -it --network host \
  -v "$PWD/src/robosub_simulation/config/pool.yaml:/config/pool.yaml:ro" \
  robosub-simulation:pool \
  ros2 launch robosub_simulation pool.launch.py headless:=true pool_config:=/config/pool.yaml
```

Other launch arguments: `robot_config`, `spawn_robot`, `keyboard_teleop`, `simulation_rate` (100 Hz), `render_rate` (20 Hz), `window_res_x` (1280), `window_res_y` (720), and `rendering_quality` (`low`, `medium`, `high`; default `low`). Run `ros2 launch robosub_simulation pool.launch.py --show-args` for the full list.

We use Stonefish's NED coordinates: +X north, +Y east, +Z down. The origin is the pool centre at the water surface. The inner faces of the walls sit at `x = ±length/2` and `y = ±width/2`; the floor surface sits at `z = depth`. Walls extend above the water by `freeboard`. Wall and floor thicknesses extend outside the usable volume.

Stonefish models the water as an infinite half-space below `z = 0`. Collision geometry encloses the usable pool; water still exists outside its walls. We disable waves, currents and suspended particles. Adjust density, temperature, Jerlov water type and sun angles in the YAML. Indoor lighting, tiles, lane markings and competition obstacles remain future scene additions.

Each launch generates a scenario in a temporary directory and removes it on shutdown. [`scenarios/pool.scn`](src/robosub_simulation/scenarios/pool.scn) is the default scene for inspection and standalone parser checks. Regenerate it after changing the default YAML:

```bash
PYTHONPATH=src/robosub_simulation python3 -m robosub_simulation.pool \
  --config src/robosub_simulation/config/pool.yaml \
  --output src/robosub_simulation/scenarios/pool.scn
```

## Native Ubuntu / existing Jazzy development container

Install and source ROS 2 Jazzy, then run from this repository:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git patch pkg-config \
  libglm-dev libsdl2-dev libfreetype6-dev libgl1-mesa-dev \
  libgl1-mesa-dri libglx-mesa0 libegl-mesa0 \
  python3-colcon-common-extensions python3-vcstool python3-yaml python3-pytest \
  ros-jazzy-image-transport ros-jazzy-pcl-conversions ros-jazzy-tf2-ros ros-jazzy-ros2launch
git submodule update --init --recursive
git -C stonefish apply ../patches/stonefish-ocean-shading.patch
cmake -S stonefish -B build/stonefish -DBUILD_TESTS=OFF
cmake --build build/stonefish --parallel 4
sudo cmake --install build/stonefish
sudo ldconfig
vcs import . < dependencies.repos
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch robosub_simulation pool.launch.py
```

Apply the native shader patch once per fresh submodule checkout; the Docker build applies it to a separate copy. `dependencies.repos` pins the ROS interface to revision `6646e7ac25eed982f37f807abc7b545dbd2d6648` (version 1.6). Keep the engine and ROS interface versions aligned when upgrading.

Our `robosub_stonefish/pool_simulator` node reuses the [upstream ROS interface](https://github.com/patrykcieslak/stonefish_ros2) and provides pool camera presets and ordered shutdown. The browser layer uses [noVNC](https://github.com/novnc/noVNC) and websockify to stream an internal Xvfb display. All simulation and rendering run in the Linux container; the client browser displays the stream.

The image also applies [`patches/novnc-close-status.patch`](patches/novnc-close-status.patch). It sends a normal WebSocket close code when reconnecting, avoiding the packaged proxy's invalid response to a close frame without a status code.

## Validation

Test geometry, configuration errors and the default scene inside the image:

```bash
docker run --rm robosub-simulation:pool bash -c \
  'cd /workspace; PYTHONPATH=src/robosub_simulation python3 -m pytest -p no:cacheprovider src/robosub_simulation/test -q'
```

With Stonefish installed, also check the real parser, water physics and collisions:

```bash
cmake -S tests/stonefish -B build/pool_smoke
cmake --build build/pool_smoke
build/pool_smoke/pool_smoke src/robosub_simulation/scenarios/pool.scn 25 15 3
```

The smoke check casts rays against each wall and the floor, then advances 100 physics steps. ROS launch and service checks require a built `stonefish_ros2` package. The simulator node runs at `/simulation/stonefish_simulator`; inspect its services with `ros2 service list` in a terminal sharing its ROS domain.

Run the ROS integration checks in separate domains. They check pool services and dimension overrides, neutral buoyancy, sensor output, all six motion axes, command and odometry timeouts, parser output and clean shutdown:

```bash
docker run --rm -e ROS_DOMAIN_ID=88 -v "$PWD/tests:/checks:ro" \
  robosub-simulation:pool python3 /checks/ros/pool_smoke.py
docker run --rm -e ROS_DOMAIN_ID=89 -v "$PWD/tests:/checks:ro" \
  robosub-simulation:pool python3 /checks/ros/robot_smoke.py
```

The 40 Python tests cover pool geometry and configuration, robot XML, CAD inertia conversion, neutral displacement, thruster allocation and controller limits. The ROS checks exercise the actual Stonefish physics and built-in sensors.

With the browser viewer running, install the browser test dependencies and run the UI check:

```bash
npm --prefix tests/browser install
npx --prefix tests/browser playwright install chromium --only-shell
npm --prefix tests/browser test
```

For physical motion verification, start a ROS observer before the browser test, then validate its recording afterward:

```bash
docker compose cp tests/browser/observe.py simulation:/tmp/observe.py
docker compose exec -d simulation bash /simulation_entrypoint.sh \
  python3 /tmp/observe.py --duration 100
npm --prefix tests/browser test
docker compose cp simulation:/tmp/robot_browser_telemetry.jsonl /tmp/robot_browser_telemetry.jsonl
python3 tests/browser/check_telemetry.py /tmp/robot_browser_telemetry.jsonl
```

The Chromium check covers all twelve motion keys, simultaneous keys, Stop, camera presets, fullscreen, reconnect, mobile layout and disconnect while driving. The telemetry check verifies signed motion on every axis, bounded thruster commands, stable dynamics and forward camera images. Graphical checks use the same Mesa software rendering streamed to the browser; they do not require a host GPU.

## ROS integration stages

1. **Pool environment (this stage):** launch the configurable environment and validate rendering and collision boundaries.
2. **Vehicle prototype (implemented):** CAD-based body, provisional buoyancy and drag, eight built-in thrusters, six-axis keyboard control, and built-in sensors. Replace the provisional physical parameters and actuator layout with measurements before evaluating hardware behavior. See [the integration notes](docs/robot-integration.md).
3. **ROS adapters:** translate Stonefish messages and coordinates to the interfaces in `robosub-ros2`. Add a simulation launch for production controls, sensor fusion and task planning.

The default launch includes the vehicle prototype. Production control adapters and calibrated vehicle dynamics remain future work.
