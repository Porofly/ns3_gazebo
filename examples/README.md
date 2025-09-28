# NS3-Gazebo Examples and Demos

This directory contains practical examples and demonstrations of the NS3-Gazebo integration system. These examples are designed to help users understand and learn how to use the system effectively.

## Quick Start

Each example includes:
- Complete source code with comments
- Build instructions
- Runtime commands
- Expected output description
- Configuration files

## Available Examples

### Basic Examples

1. **[hello_world_network](01_hello_world_network/)**
   - Simple two-node WiFi network simulation
   - Packet transmission and reception
   - Basic NS-3 and Gazebo integration
   - **Difficulty**: Beginner
   - **Duration**: 5 minutes

2. **[simple_robot_navigation](02_simple_robot_navigation/)**
   - Single robot navigation in Gazebo
   - ROS2 navigation integration
   - Basic obstacle avoidance
   - **Difficulty**: Beginner
   - **Duration**: 10 minutes

3. **[wifi_performance_test](03_wifi_performance_test/)**
   - WiFi network performance analysis
   - Throughput and latency measurements
   - Multiple node configurations
   - **Difficulty**: Intermediate
   - **Duration**: 15 minutes

4. **[tap_bridge_connection](04_tap_bridge_connection/)**
   - TAP bridge device demonstration
   - Real network interface connection
   - Network namespace usage
   - **Difficulty**: Intermediate
   - **Duration**: 20 minutes

### Running Examples

Each example can be run using the provided scripts:

```bash
# Navigate to example directory
cd examples/01_hello_world_network

# Build the example
./build.sh

# Run the simulation
./run.sh

# Clean build files
./clean.sh
```

### Prerequisites

- Complete NS3-Gazebo installation
- ROS2 Jazzy environment sourced
- Gazebo Harmonic installed
- Network tools (for TAP bridge examples)

### Docker Support

All examples are Docker-ready:

```bash
# Build and run in Docker
./scripts/docker_run.sh -p examples/01_hello_world_network/run.sh

# Or use docker-compose
docker-compose -f examples/docker-compose.yml up hello-world
```

## Learning Path

**For Beginners:**
1. Start with `hello_world_network`
2. Progress to `simple_robot_navigation`
3. Try `wifi_performance_test`

**For Advanced Users:**
1. Begin with `tap_bridge_connection`
2. Explore multi-robot scenarios
3. Create custom examples

## Contributing Examples

We welcome new examples! Please follow these guidelines:

1. **Structure**: Use the standard example structure
2. **Documentation**: Include comprehensive README
3. **Testing**: Verify on clean systems
4. **Comments**: Add detailed code comments

See [CONTRIBUTING.md](../CONTRIBUTING.md) for detailed guidelines.

## Support

- **Issues**: Report problems in the main repository
- **Questions**: Use GitHub Discussions
- **Documentation**: Check individual example READMEs

## License

All examples are provided under the same license as the main project (MIT License).