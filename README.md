# RoboSub simulation

Build and run Duke RoboSub environments with Stonefish 1.6 and ROS 2 Jazzy, matching the ROS distribution in [`robosub-ros2`](../robosub-ros2).

The first stage is a configurable outdoor pool: calm freshwater, sunlight, a floor and four collision walls. The default interior is **25 m × 15 m × 3 m deep**. These are development dimensions; replace them with measurements for the pool you want to reproduce.

## View the pool on Wayland, Windows or macOS

Install Docker Engine with Compose on Linux, or Docker Desktop with Linux containers on Windows/macOS. Run these commands from `robosub-simulation`. Initialize the existing Stonefish submodule if your checkout lacks its source:

```bash
git submodule update --init --recursive
docker compose up --build
```

Open **[http://localhost:8080](http://localhost:8080)** in your browser. The viewer streams the actual Stonefish window and forwards mouse and keyboard input. You do not need host display mounts, `xhost`, a Linux desktop, or a host GPU. KDE, GNOME, Wayland and X11 use the same browser interface; Windows and macOS run the simulator inside Docker's Linux VM. The image uses Mesa software OpenGL by default. The viewer has been tested on Linux with an AMD64 container; Windows, macOS and ARM64 builds still need verification on those hosts.

The first build compiles Stonefish and its ROS interface. Use `docker compose up -d` to run in the background, `docker compose logs -f simulation` to inspect logs, and `docker compose down` to stop it. The viewer port binds to the local machine.

### Camera controls

- **Surface view / R:** frame the whole pool from above. This is the initial view.
- **Underwater / U:** move the view into the water.
- **Right mouse drag:** orbit. **Middle mouse drag:** pan. **Scroll:** zoom.
- **W/A/S/D:** move the camera horizontally along its viewing axes. **Q/Z:** move up/down. Hold Shift for larger steps.
- **H:** show or hide Stonefish's inspector. **Fullscreen:** enlarge the browser viewport. **Reconnect:** restore the stream after a connection interruption.

Click inside the viewport before using keyboard controls. On a trackpad without a middle button, use W/A/S/D/Q/Z to move the camera. Camera changes affect the view; pool geometry and physics stay the same.

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

Other launch arguments: `simulation_rate` (100 Hz), `render_rate` (20 Hz), `window_res_x` (1280), `window_res_y` (720), and `rendering_quality` (`low`, `medium`, `high`; default `low`). Run `ros2 launch robosub_simulation pool.launch.py --show-args` for the full list.

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

Run the ROS integration check in a separate domain. It checks service calls, dimension overrides, parser output, clean process exit and temporary scene cleanup:

```bash
docker run --rm -e ROS_DOMAIN_ID=88 -v "$PWD/tests:/checks:ro" \
  robosub-simulation:pool python3 /checks/ros/pool_smoke.py
```

All 18 configuration and geometry tests pass, covering both 25 × 15 × 3 m and 50 × 25 × 4 m scenes. The ROS service and shutdown check passes in the built image. Chromium browser checks verify real rendered frames, surface and underwater presets, mouse input, fullscreen, reconnect and a narrow-screen layout, without browser errors. Graphical validation uses Mesa software OpenGL on the same virtual desktop that the browser viewer streams. A hardware GPU can improve rendering speed, but the default viewer does not require one.

## Robot and ROS integration stages

1. **Pool environment (this stage):** launch the configurable environment and validate rendering and collision boundaries.
2. **Vehicle:** use the URDF's mesh, mass and inertia to define the Stonefish body. Add collision geometry, displaced volume, centre of buoyancy, and thrusters with the team's positions and order. See [the integration notes](docs/robot-integration.md).
3. **Sensors and ROS adapters:** simulate IMU, pressure, DVL and cameras, then translate Stonefish messages and coordinates to the interfaces in `robosub-ros2`. Add a simulation launch for controls, sensor fusion and task planning.

The pool-only launch starts the environment. Vehicle dynamics, sensor topics and control adapters belong to stages 2 and 3.
