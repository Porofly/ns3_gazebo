#!/bin/bash

# =============================================================================
# Docker Run Script for NS3-Gazebo Development Environment
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="ns3-gazebo"
IMAGE_TAG="latest"
CONTAINER_NAME="ns3-gazebo-dev"

# Options
GUI_SUPPORT=false
PRIVILEGED=false
DETACH=false
REMOVE=true
CUSTOM_COMMAND=""
MOUNT_SOURCE=true

# Functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Show help
show_help() {
    echo "Usage: $0 [OPTIONS] [COMMAND]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -t, --tag TAG           Docker image tag (default: latest)"
    echo "  -n, --name NAME         Container name (default: ns3-gazebo-dev)"
    echo "  -g, --gui               Enable GUI support (X11 forwarding)"
    echo "  -p, --privileged        Run with privileged mode (for network namespaces)"
    echo "  -d, --detach            Run in detached mode"
    echo "  --no-rm                 Don't remove container when it exits"
    echo "  --no-mount              Don't mount source code"
    echo ""
    echo "Examples:"
    echo "  $0                      # Run interactive shell"
    echo "  $0 -g                   # Run with GUI support"
    echo "  $0 -p                   # Run with network privileges"
    echo "  $0 -d                   # Run in background"
    echo "  $0 \"bash\"               # Run specific command"
    echo "  $0 -g \"gazebo\"          # Run Gazebo with GUI"
    echo ""
    echo "Pre-defined commands:"
    echo "  shell                   # Interactive bash shell (default)"
    echo "  test                    # Run verification tests"
    echo "  build                   # Build all components"
    echo "  gazebo                  # Start Gazebo simulation"
    echo "  ros2                    # Start ROS2 environment"
    echo ""
}

# Parse command line arguments
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -t|--tag)
                IMAGE_TAG="$2"
                shift 2
                ;;
            -n|--name)
                CONTAINER_NAME="$2"
                shift 2
                ;;
            -g|--gui)
                GUI_SUPPORT=true
                shift
                ;;
            -p|--privileged)
                PRIVILEGED=true
                shift
                ;;
            -d|--detach)
                DETACH=true
                REMOVE=false
                shift
                ;;
            --no-rm)
                REMOVE=false
                shift
                ;;
            --no-mount)
                MOUNT_SOURCE=false
                shift
                ;;
            --)
                shift
                CUSTOM_COMMAND="$*"
                break
                ;;
            -*)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
            *)
                CUSTOM_COMMAND="$*"
                break
                ;;
        esac
    done
}

# Check requirements
check_requirements() {
    # Check Docker
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed"
        exit 1
    fi

    # Check if image exists
    if ! docker images | grep -q "$IMAGE_NAME.*$IMAGE_TAG"; then
        log_error "Docker image $IMAGE_NAME:$IMAGE_TAG not found"
        log_info "Build the image first: ./scripts/docker_build.sh"
        exit 1
    fi

    # Check GUI requirements
    if [[ "$GUI_SUPPORT" == "true" ]]; then
        if [[ -z "$DISPLAY" ]]; then
            log_error "DISPLAY environment variable not set"
            log_info "GUI support requires X11 display"
            exit 1
        fi

        # Enable X11 forwarding
        if command -v xhost &> /dev/null; then
            xhost +local:docker &> /dev/null || true
            log_info "X11 forwarding enabled"
        else
            log_warning "xhost not available, X11 forwarding may not work"
        fi
    fi
}

# Prepare run command
prepare_run_command() {
    local run_cmd="docker run"

    # Basic options
    run_cmd="$run_cmd -i"
    if [[ "$DETACH" != "true" ]]; then
        run_cmd="$run_cmd -t"
    fi

    # Container name
    run_cmd="$run_cmd --name $CONTAINER_NAME"

    # Remove container on exit
    if [[ "$REMOVE" == "true" ]]; then
        run_cmd="$run_cmd --rm"
    fi

    # Detach mode
    if [[ "$DETACH" == "true" ]]; then
        run_cmd="$run_cmd -d"
    fi

    # Mount source code
    if [[ "$MOUNT_SOURCE" == "true" ]]; then
        run_cmd="$run_cmd -v $PROJECT_DIR:/workspace/ns3_gazebo"
    fi

    # GUI support
    if [[ "$GUI_SUPPORT" == "true" ]]; then
        run_cmd="$run_cmd -e DISPLAY=$DISPLAY"
        run_cmd="$run_cmd -e QT_X11_NO_MITSHM=1"
        run_cmd="$run_cmd -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    fi

    # Privileged mode (for network namespaces)
    if [[ "$PRIVILEGED" == "true" ]]; then
        run_cmd="$run_cmd --privileged"
        run_cmd="$run_cmd --cap-add=NET_ADMIN"
        run_cmd="$run_cmd --cap-add=SYS_ADMIN"
        run_cmd="$run_cmd --device=/dev/net/tun"
    fi

    # Working directory
    run_cmd="$run_cmd -w /workspace/ns3_gazebo"

    # Image
    run_cmd="$run_cmd $IMAGE_NAME:$IMAGE_TAG"

    echo "$run_cmd"
}

# Prepare command to run inside container
prepare_container_command() {
    case "$CUSTOM_COMMAND" in
        ""|-|"shell")
            echo "/bin/bash"
            ;;
        "test")
            echo "bash -c 'source /opt/ros/jazzy/setup.bash && ./scripts/verify_installation.sh'"
            ;;
        "build")
            echo "bash -c 'source /opt/ros/jazzy/setup.bash && ./install.sh --build-only'"
            ;;
        "gazebo")
            if [[ "$GUI_SUPPORT" == "true" ]]; then
                echo "bash -c 'source /opt/ros/jazzy/setup.bash && export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/ns3_gazebo/ns3_gazebo_plugin/build && gz sim ns3_gazebo_plugin/gazebo_ros_diff_drive_ns3_gazebo.world'"
            else
                echo "bash -c 'source /opt/ros/jazzy/setup.bash && export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/ns3_gazebo/ns3_gazebo_plugin/build && gz sim ns3_gazebo_plugin/gazebo_ros_diff_drive_ns3_gazebo.world --headless'"
            fi
            ;;
        "ros2")
            echo "bash -c 'source /opt/ros/jazzy/setup.bash && source ns3_gazebo_ws/install/setup.bash && /bin/bash'"
            ;;
        *)
            echo "$CUSTOM_COMMAND"
            ;;
    esac
}

# Run container
run_container() {
    local run_cmd=$(prepare_run_command)
    local container_cmd=$(prepare_container_command)

    log_info "Starting container: $CONTAINER_NAME"
    log_info "Image: $IMAGE_NAME:$IMAGE_TAG"

    if [[ "$GUI_SUPPORT" == "true" ]]; then
        log_info "GUI support: enabled"
    fi

    if [[ "$PRIVILEGED" == "true" ]]; then
        log_info "Privileged mode: enabled"
    fi

    if [[ "$MOUNT_SOURCE" == "true" ]]; then
        log_info "Source mount: $PROJECT_DIR"
    fi

    echo ""
    log_info "Running: $run_cmd $container_cmd"
    echo ""

    # Remove existing container with same name
    if docker ps -a --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
        log_warning "Removing existing container: $CONTAINER_NAME"
        docker rm -f "$CONTAINER_NAME" &> /dev/null || true
    fi

    # Execute run command
    eval "$run_cmd $container_cmd"
}

# Show container info
show_info() {
    if [[ "$DETACH" == "true" ]]; then
        echo ""
        log_success "Container started in detached mode"
        echo ""
        echo "Useful commands:"
        echo "  docker exec -it $CONTAINER_NAME /bin/bash    # Attach to container"
        echo "  docker logs $CONTAINER_NAME                  # View container logs"
        echo "  docker stop $CONTAINER_NAME                  # Stop container"
        echo "  docker rm $CONTAINER_NAME                    # Remove container"
        echo ""
    fi
}

# Main function
main() {
    parse_arguments "$@"
    check_requirements
    run_container
    show_info
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi