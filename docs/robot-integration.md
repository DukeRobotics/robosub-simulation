# Vehicle integration notes

## Implemented prototype

The default launch now spawns Crush with eight built-in Stonefish thrusters, IMU, pressure, DVL, ground-truth odometry and a forward camera. [`crush.xml`](../src/robosub_simulation/scenarios/crush.xml) is the generated robot definition; [`crush.yaml`](../src/robosub_simulation/config/crush.yaml) contains the provisional actuator layout, drag settings and teleop controller parameters. Launch regenerates the XML and includes it in the pool scene.

The simulation treats the supplied URDF axes as forward/left/up and rotates the CAD coordinates by 180 degrees around X into forward/right/down. It preserves the dry mass and complete inertia tensor using principal moments and a rotated centre-of-mass frame. Stonefish adds hydrodynamic mass and inertia during simulation. Confirm the assumed CAD forward axis against the physical vehicle before hardware comparison.

[`tools/prepare_crush.py`](../tools/prepare_crush.py) reads the unchanged binary STL and URDF, clusters the visual mesh to 52,925 triangles, creates a closed bounding-box collision mesh, and writes the derived properties to `crush_model.yaml`. The model spans approximately 0.616 × 0.652 × 0.380 m. The source mesh checksum is recorded in the generated YAML. Run the script with NumPy and PyYAML available, or inside the simulation image with this repository mounted.

An internal buoyancy sphere supplies displacement equal to mass/water density, with its centre at the centre of mass. This gives neutral buoyancy without an invented restoring torque. The collision envelope and drag coefficients are development approximations. Skin friction uses 0.005 per axis: larger provisional coefficients caused unstable angular dynamics with the CAD inertia and Stonefish's 50 Hz fluid-force updates.

The eight-thruster layout has four horizontal vectored thrusters and four vertical thrusters. It provides full-rank six-axis allocation and uses first-order rotor dynamics with a quadratic thrust model. It differs from the six-thruster Crush hardware layout, whose current configuration disables pitch control. Thruster positions, order, force curves and centre of buoyancy need measurements before production controls are connected.

Keyboard commands request body velocities. The prototype mixer uses built-in ground truth for proportional velocity feedback and sends normalized rotor setpoints through the upstream ROS interface. It does not replace the production EKF or controls node. Raw sensor and actuator topics stay under `/simulation/crush`; see the README for topic types and keyboard mappings.

## Supplied CAD export

The archive `Full Model Assembly (1).zip` contains `Full Model Assembly (1)/urdf/Full Model Assembly (1).urdf`. It defines one `base_link` with visual and collision references to `meshes/base_link.STL`.

The exported mass is 14.4563329079817 kg. The centre of mass is `(-0.0678438517471901, -0.0264337175237818, 0.0078096355235136)` m in the exported link frame. The URDF also supplies an inertia tensor. Confirm the CAD origin and axes against the team's `base_link` before using those values in Stonefish.

The package name `Full Model Assembly (1)` contains spaces and parentheses. Give the imported assets a valid ROS package name and update their `package://` references. Preserve the CAD export as the source asset; use a lighter visual mesh and separate collision/hydrodynamic geometry for simulation. Verify mesh dimensions before choosing a scale.

The URDF includes no sensor frames, thruster poses, buoyancy geometry or actuator definitions. The source now lives under `assets/crush`, and the prototype uses it as Crush. Compare measured physical parameters with the corresponding configuration in `robosub-ros2`:

- `onboard/src/controls/config/{crush,oogway}.yaml`: mass, inertia and thruster configuration.
- `onboard/src/offboard_comms/config/{crush,oogway}.yaml`: thruster ordering and hardware conventions.
- `onboard/src/static_transforms/config/{crush,oogway}.yaml`: sensor frames.

Stonefish uses its own scenario robot definitions. Translate the URDF geometry and inertial data into a Stonefish body, then add marine physics and actuator definitions. Add the vehicle scene as a separate include so both pool and vehicle configurations can change without duplicating the environment.

## Interfaces to preserve

The existing controls and sensor-fusion code provides these integration targets:

| Interface | ROS 2 type | Existing consumer / source |
| --- | --- | --- |
| `/controls/thruster_allocs` | `custom_msgs/msg/ThrusterAllocs` (`float64[] allocs`) | `controls/src/controls.cpp`, `offboard_comms/thrusters.py` |
| `/vectornav/imu` | `sensor_msgs/msg/Imu` | `sensor_fusion/config/{crush,oogway}.yaml` |
| `/sensors/depth` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `offboard_comms/peripheral_sensors.py`, EKF configuration |
| `/sensors/dvl/odom` | `nav_msgs/msg/Odometry` | EKF configuration and DVL conversion node |
| `/sensors/gyro/angular_velocity/twist` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `offboard_comms/gyro.py`, EKF configuration |
| `/state` | `nav_msgs/msg/Odometry` | Output of sensor fusion; input to controls and task planning |

Use separate `/simulation/...` topics for ground truth and raw Stonefish messages. Feed simulated measurements through the production EKF when validating sensor fusion. Match frames, covariance, units, depth sign and timestamps in each adapter. Check camera topics and calibration against the chosen CV pipeline during the camera stage.

Stonefish uses NED; the ROS stack uses ROS frame conventions and its configured sensor transforms. Define and test the conversion at the simulation boundary, including angular velocity, orientation and covariance. Preserve the thruster order from the selected robot and verify each actuator's force direction before enabling closed-loop control.

The pinned upstream `ROS2SimulationManager` uses the node's ROS clock for timing and sensor stamps and does not publish `/clock`. Keep `use_sim_time=false` for this first stage. A later simulation-time implementation needs a monotonic physics clock, `/clock` publication and consistent message stamps before enabling `use_sim_time` throughout the stack.

Create a simulation-specific bringup launch for production controls, static transforms and sensor fusion. The existing `execute/launch/robot.launch.py` also launches serial devices and physical sensors; use the individual software launches for simulation. Set `enable_teleop:=false` when introducing a different actuator controller so only one node commands the thrusters.

## Acceptance checks for the next stages

- Spawn at `z = 1 m` inside the pool, verify attitude and mesh scale, then validate neutral buoyancy with the measured wet mass and displaced volume.
- Command each thruster in isolation and confirm its force direction and ordering. Check collisions against walls and the floor under motion.
- Compare static IMU, depth and DVL values against known poses. Compare trajectories against ground truth under noise and dropout.
- Run controls and the EKF together, then exercise task planning and CV with repeatable pool obstacles.
