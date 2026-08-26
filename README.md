# RoboSub Simulation Environment

A simulation and rendering environment for RoboSub, providing a virtual competition pool for developing and testing navigation, perception, and control code without needing physical pool access.

## Status

Early development / planning stage. The simulation tool has not yet been selected.

## Goals

- Render a virtual pool environment matching the competition pool's dimensions, lighting, and underwater visibility
- Integrate the team's robot URDF model into the simulated environment
- Interface with ROS2 so existing control and perception code can run against the simulated robot without modification

## Roadmap

Tracked as GitHub issues:

1. **Underwater Physics Simulation and Rendering**: build the pool scene itself (dimensions, water and lighting rendering)
2. **Robot URDF Integration**: bring the vehicle's URDF model into the simulated pool
3. **Robot Integration with ROS2**: wire the simulated robot up to the same ROS2 topics and services as the real vehicle

## Getting Started

TBD. Setup instructions will be added once the simulation tool is selected and the initial pool scene is in place.

## Contributing

Open an issue before starting work on a new piece of functionality. Check the roadmap above for current scope.
