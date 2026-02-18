# Autonomous Rover (ROS 2 Humble, Ackermann)

Production-grade template for an Ackermann-steered autonomous rover.
Includes Nav2, localization, safety, CI, and AI-assisted workflows.

## Architecture Blueprints

The full system architecture (Gazebo ↔ RTAB-Map interfaces, topic contracts, and node graph) lives in the `docs/architecture` directory:

- [Architecture Overview](docs/architecture/overview.md)
- [Node Graph](docs/architecture/node_graph.md)
- [Interfaces Reference](docs/architecture/interfaces.md)

These documents stay current with the `robot_bringup` launch composition, ros_gz bridge topics, and RTAB-Map + Nav2 integrations—start there when wiring new hardware or extending the stack.
