#!/bin/bash

# =============================================================================
# Docker Build Script for NS3-Gazebo
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

# Options
BUILD_ARGS=""
NO_CACHE=false
VERBOSE=false

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

log_header() {
    echo -e "${PURPLE}[BUILD]${NC} $1"
}

# Show help
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -t, --tag TAG           Docker image tag (default: latest)"
    echo "  --no-cache              Build without using cache"
    echo "  -v, --verbose           Verbose output"
    echo "  --build-arg ARG=VALUE   Pass build argument to Docker"
    echo ""
    echo "Examples:"
    echo "  $0                      # Build with default settings"
    echo "  $0 -t dev               # Build with 'dev' tag"
    echo "  $0 --no-cache           # Build without cache"
    echo "  $0 --build-arg HTTP_PROXY=http://proxy:8080"
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
            --no-cache)
                NO_CACHE=true
                shift
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            --build-arg)
                BUILD_ARGS="$BUILD_ARGS --build-arg $2"
                shift 2
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

# Check Docker installation
check_docker() {
    log_header "Checking Docker installation"

    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed"
        log_info "Please install Docker: https://docs.docker.com/get-docker/"
        exit 1
    fi

    # Check if Docker daemon is running
    if ! docker info &> /dev/null; then
        log_error "Docker daemon is not running"
        log_info "Please start Docker daemon"
        exit 1
    fi

    log_success "Docker is available"
}

# Check system resources
check_resources() {
    log_header "Checking system resources"

    # Check available disk space (at least 10GB)
    AVAILABLE_SPACE=$(df -BG "$PROJECT_DIR" | tail -1 | awk '{print $4}' | sed 's/G//')
    if [[ $AVAILABLE_SPACE -lt 10 ]]; then
        log_warning "Low disk space: ${AVAILABLE_SPACE}GB (recommended: 10GB+)"
    else
        log_success "Sufficient disk space: ${AVAILABLE_SPACE}GB"
    fi

    # Check available memory
    TOTAL_MEM_GB=$(grep MemTotal /proc/meminfo | awk '{print int($2/1024/1024)}')
    if [[ $TOTAL_MEM_GB -lt 4 ]]; then
        log_warning "Low memory: ${TOTAL_MEM_GB}GB (recommended: 4GB+)"
    else
        log_success "Sufficient memory: ${TOTAL_MEM_GB}GB"
    fi
}

# Prepare build context
prepare_build() {
    log_header "Preparing build context"

    cd "$PROJECT_DIR"

    # Check if Dockerfile exists
    if [ ! -f "Dockerfile" ]; then
        log_error "Dockerfile not found in $PROJECT_DIR"
        exit 1
    fi

    log_success "Build context ready"
}

# Build Docker image
build_image() {
    log_header "Building Docker image: $IMAGE_NAME:$IMAGE_TAG"

    local build_cmd="docker build"

    # Add no-cache option
    if [[ "$NO_CACHE" == "true" ]]; then
        build_cmd="$build_cmd --no-cache"
        log_info "Building without cache"
    fi

    # Add build arguments
    if [[ -n "$BUILD_ARGS" ]]; then
        build_cmd="$build_cmd $BUILD_ARGS"
        log_info "Using build arguments: $BUILD_ARGS"
    fi

    # Add tag and context
    build_cmd="$build_cmd -t $IMAGE_NAME:$IMAGE_TAG ."

    log_info "Build command: $build_cmd"
    log_info "This process may take 20-60 minutes..."
    echo ""

    # Execute build
    if [[ "$VERBOSE" == "true" ]]; then
        eval "$build_cmd"
    else
        eval "$build_cmd" 2>&1 | tee docker_build.log
    fi

    if [[ ${PIPESTATUS[0]} -eq 0 ]]; then
        log_success "Docker image built successfully"
    else
        log_error "Docker build failed"
        log_info "Check docker_build.log for details"
        exit 1
    fi
}

# Verify image
verify_image() {
    log_header "Verifying Docker image"

    # Check if image exists
    if docker images | grep -q "$IMAGE_NAME.*$IMAGE_TAG"; then
        log_success "Image $IMAGE_NAME:$IMAGE_TAG created successfully"

        # Show image details
        IMAGE_SIZE=$(docker images --format "table {{.Size}}" "$IMAGE_NAME:$IMAGE_TAG" | tail -1)
        log_info "Image size: $IMAGE_SIZE"
    else
        log_error "Image $IMAGE_NAME:$IMAGE_TAG not found"
        exit 1
    fi

    # Quick functionality test
    log_info "Running quick functionality test..."
    if docker run --rm "$IMAGE_NAME:$IMAGE_TAG" bash -c "source /opt/ros/jazzy/setup.bash && ros2 --version" &> /dev/null; then
        log_success "ROS2 functionality verified"
    else
        log_warning "ROS2 functionality test failed"
    fi

    if docker run --rm "$IMAGE_NAME:$IMAGE_TAG" bash -c "gz sim --versions" &> /dev/null; then
        log_success "Gazebo functionality verified"
    else
        log_warning "Gazebo functionality test failed"
    fi
}

# Show usage instructions
show_usage() {
    echo ""
    echo "==============================================="
    echo "         DOCKER BUILD COMPLETE!"
    echo "==============================================="
    echo ""
    log_success "Docker image $IMAGE_NAME:$IMAGE_TAG built successfully!"
    echo ""
    echo "Usage examples:"
    echo ""
    echo "# Run interactive development environment"
    echo "docker run -it --rm $IMAGE_NAME:$IMAGE_TAG"
    echo ""
    echo "# Run with X11 forwarding (GUI support)"
    echo "xhost +local:docker"
    echo "docker run -it --rm \\"
    echo "  -e DISPLAY=\$DISPLAY \\"
    echo "  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\"
    echo "  $IMAGE_NAME:$IMAGE_TAG"
    echo ""
    echo "# Use docker-compose for full environment"
    echo "docker-compose up ns3-gazebo-dev"
    echo ""
    echo "# Run tests"
    echo "docker-compose up ns3-gazebo-test"
    echo ""
    echo "Build log saved to: docker_build.log"
    echo ""
}

# Main function
main() {
    echo "==============================================="
    echo "    NS3-Gazebo Docker Build Script"
    echo "==============================================="
    echo ""

    parse_arguments "$@"

    log_info "Building Docker image with tag: $IMAGE_TAG"
    echo ""

    check_docker
    check_resources
    prepare_build
    build_image
    verify_image
    show_usage
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi