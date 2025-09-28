# Docker Development Guide for NS3-Gazebo

## Overview

This guide provides comprehensive instructions for using Docker with the NS3-Gazebo integration system. Docker provides a consistent, isolated development environment that works across different operating systems and configurations.

## Quick Start

### 1. Build the Docker Image

```bash
# Build the Docker image
./scripts/docker_build.sh

# Build with custom tag
./scripts/docker_build.sh -t dev

# Build without cache (clean build)
./scripts/docker_build.sh --no-cache
```

### 2. Run the Development Environment

```bash
# Interactive development shell
./scripts/docker_run.sh

# With GUI support (for Gazebo visualization)
./scripts/docker_run.sh -g

# With network privileges (for network namespace testing)
./scripts/docker_run.sh -p

# Run specific commands
./scripts/docker_run.sh test        # Run verification tests
./scripts/docker_run.sh build       # Build all components
./scripts/docker_run.sh gazebo      # Start Gazebo simulation
```

### 3. Use Docker Compose

```bash
# Start full development environment
docker-compose up ns3-gazebo-dev

# Run tests
docker-compose up ns3-gazebo-test

# Run headless Gazebo server
docker-compose up gazebo-server

# Run all services
docker-compose up
```

## Docker Image Details

### Base System
- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalopy
- **Gazebo**: Harmonic (gz-sim8)
- **NS-3**: Version 3.45

### Installed Components
- ROS2 Jazzy desktop installation
- Gazebo Harmonic with all plugins
- NS-3 3.45 with full feature set
- Build tools (CMake, GCC, Python3)
- Network tools for TAP bridge testing
- Development tools (Git, debugging tools)

### Image Size
Approximately 4-6 GB (depending on build optimizations)

## Development Workflows

### Interactive Development

```bash
# Start development container
./scripts/docker_run.sh -g

# Inside container - build and test
source /opt/ros/jazzy/setup.bash
cd /workspace/ns3_gazebo
./install.sh --build-only
./scripts/verify_installation.sh
```

### Automated Testing

```bash
# Run comprehensive test suite
./scripts/docker_test.sh

# Test specific components
docker-compose up ns3-gazebo-test
```

### GUI Applications

For GUI applications like Gazebo visualization:

```bash
# Enable X11 forwarding (Linux/macOS)
xhost +local:docker

# Run with GUI support
./scripts/docker_run.sh -g gazebo
```

For Windows users, consider using:
- VcXsrv or Xming for X11 forwarding
- WSL2 with Docker Desktop

## Network Configuration

### Basic Networking

The container provides standard networking capabilities:
- Internet access for downloading packages
- Port forwarding for ROS2 communication
- Bridge networking between containers

### Advanced Networking (Privileged Mode)

For network namespace testing:

```bash
# Run with network privileges
./scripts/docker_run.sh -p

# Inside container - create network namespaces
sudo ip netns add test-ns
sudo ip netns exec test-ns ip addr show
sudo ip netns delete test-ns
```

### Docker Compose Network

The docker-compose setup creates a custom network:
- Subnet: 172.20.0.0/16
- Bridge driver for container communication
- Isolated from host network for security

## Volume Mounts

### Source Code Mount

By default, the container mounts your source code:
```
Host: /path/to/ns3_gazebo
Container: /workspace/ns3_gazebo
```

This allows:
- Real-time code editing on host
- Builds inside container
- Persistent changes across container restarts

### Data Persistence

```bash
# Named volume for user data
docker volume create ns3_gazebo_data

# Mount in container
docker run -v ns3_gazebo_data:/home/developer ...
```

## Troubleshooting

### Common Issues

1. **Image Build Fails**
   ```bash
   # Check available disk space (need 10GB+)
   df -h

   # Clean Docker cache
   docker system prune -a

   # Rebuild without cache
   ./scripts/docker_build.sh --no-cache
   ```

2. **GUI Applications Don't Start**
   ```bash
   # Check DISPLAY variable
   echo $DISPLAY

   # Enable X11 forwarding
   xhost +local:docker

   # Test X11 connection
   docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ubuntu:24.04 xclock
   ```

3. **Network Namespace Errors**
   ```bash
   # Run with privileged mode
   ./scripts/docker_run.sh -p

   # Check kernel capabilities
   docker run --rm --privileged ubuntu:24.04 ip netns add test
   ```

4. **Build Errors in Container**
   ```bash
   # Update package lists
   sudo apt update

   # Check available memory
   free -h

   # Reduce build parallelism
   export MAKEFLAGS="-j2"
   ```

### Performance Optimization

1. **Build Performance**
   ```bash
   # Use build arguments for proxy
   ./scripts/docker_build.sh --build-arg HTTP_PROXY=http://proxy:8080

   # Limit parallel builds
   ./scripts/docker_build.sh --build-arg MAKEFLAGS="-j2"
   ```

2. **Runtime Performance**
   ```bash
   # Allocate more memory to Docker
   # (Docker Desktop settings: 4GB+ recommended)

   # Use SSD storage for Docker
   # (significantly improves build and runtime performance)
   ```

## Security Considerations

### Privileged Mode

Use privileged mode only when necessary:
- Network namespace testing
- Low-level hardware access
- System debugging

### Network Isolation

- Containers use bridge networking by default
- No direct access to host network interfaces
- Port forwarding required for external access

### Volume Security

- Source code mounts are read-write by default
- Use read-only mounts for sensitive data:
  ```bash
  docker run -v /path/to/data:/data:ro ...
  ```

## Integration with IDEs

### VS Code

1. Install Docker extension
2. Open project in container:
   ```bash
   # From VS Code command palette
   > Dev Containers: Open Folder in Container
   ```

3. Configure devcontainer.json:
   ```json
   {
     "image": "ns3-gazebo:latest",
     "workspaceMount": "source=${localWorkspaceFolder},target=/workspace/ns3_gazebo,type=bind",
     "workspaceFolder": "/workspace/ns3_gazebo"
   }
   ```

### CLion/IntelliJ

1. Configure Docker toolchain
2. Set remote development environment
3. Use Docker as build environment

## CI/CD Integration

### GitHub Actions

```yaml
name: Docker Build Test
on: [push, pull_request]
jobs:
  docker-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Build Docker image
        run: ./scripts/docker_build.sh
      - name: Test Docker environment
        run: ./scripts/docker_test.sh
```

### Local CI

```bash
# Automated testing script
#!/bin/bash
set -e

# Build image
./scripts/docker_build.sh

# Run tests
./scripts/docker_test.sh

# Test specific functionality
docker-compose up --abort-on-container-exit ns3-gazebo-test
```

## Advanced Usage

### Multi-stage Builds

The Dockerfile uses multi-stage build concepts for optimization:
- Base system setup
- Development tools installation
- Application building
- Runtime optimization

### Custom Images

```dockerfile
# Extend the base image
FROM ns3-gazebo:latest

# Add custom tools
RUN apt-get update && apt-get install -y custom-tool

# Add custom configurations
COPY custom-config /etc/custom/
```

### Container Orchestration

```yaml
# docker-compose.override.yml
version: '3.8'
services:
  ns3-gazebo-dev:
    ports:
      - "8080:8080"  # Custom port forwarding
    environment:
      - CUSTOM_VAR=value
```

## Support and Documentation

- **Build Issues**: Check `docker_build.log`
- **Runtime Issues**: Use `docker logs <container-name>`
- **Performance**: Monitor with `docker stats`
- **Network**: Debug with `docker network ls` and `docker network inspect`

For additional support, see:
- Main project documentation: `README.md`
- Contributing guidelines: `CONTRIBUTING.md`
- Technical details: `UPGRADE_PLAN.md`