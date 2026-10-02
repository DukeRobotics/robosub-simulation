# Vehicle integration notes

## Supplied CAD export

The archive `Full Model Assembly (1).zip` contains `Full Model Assembly (1)/urdf/Full Model Assembly (1).urdf`. It defines one `base_link` with visual and collision references to `meshes/base_link.STL`.

The exported mass is 14.4563329079817 kg. The centre of mass is `(-0.0678438517471901, -0.0264337175237818, 0.0078096355235136)` m in the exported link frame. The URDF also supplies an inertia tensor. Confirm the CAD origin and axes against the team's `base_link` before using those values in Stonefish.

The package name `Full Model Assembly (1)` contains spaces and parentheses. Give the imported assets a valid ROS package name and update their `package://` references. Preserve the CAD export as the source asset; use a lighter visual mesh and separate collision/hydrodynamic geometry for simulation. Verify mesh dimensions before choosing a scale.

The URDF includes no sensor frames, thruster poses, buoyancy geometry or actuator definitions. Identify which vehicle the export represents (Crush or Oogway), then compare it with the corresponding configuration in `robosub-ros2`:

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

Create a simulation-specific bringup launch for controls, static transforms and sensor fusion. The existing `execute/launch/robot.launch.py` also launches serial devices and physical sensors; use the individual software launches for simulation.

## Acceptance checks for the next stages

- Spawn at `z = 1 m` inside the pool, verify attitude and mesh scale, then validate neutral buoyancy with the measured wet mass and displaced volume.
- Command each thruster in isolation and confirm its force direction and ordering. Check collisions against walls and the floor under motion.
- Compare static IMU, depth and DVL values against known poses. Compare trajectories against ground truth under noise and dropout.
- Run controls and the EKF together, then exercise task planning and CV with repeatable pool obstacles.
