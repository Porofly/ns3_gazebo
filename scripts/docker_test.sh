#!/bin/bash

# =============================================================================
# Docker Test Script for NS3-Gazebo
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

# Test tracking
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((PASSED_TESTS++))
}

log_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((FAILED_TESTS++))
}

log_test() {
    echo -e "${PURPLE}[TEST]${NC} $1"
    ((TOTAL_TESTS++))
}

# Test Docker image existence
test_image_exists() {
    log_test "Docker image existence"

    if docker images | grep -q "$IMAGE_NAME.*$IMAGE_TAG"; then
        log_success "Docker image $IMAGE_NAME:$IMAGE_TAG found"
    else
        log_fail "Docker image $IMAGE_NAME:$IMAGE_TAG not found"
        return 1
    fi
}

# Test basic container functionality
test_basic_container() {
    log_test "Basic container functionality"

    if docker run --rm "$IMAGE_NAME:$IMAGE_TAG" echo "Container test successful" &> /dev/null; then
        log_success "Container starts and runs commands"
    else
        log_fail "Container failed to start or run commands"
        return 1
    fi
}

# Test ROS2 installation
test_ros2_installation() {
    log_test "ROS2 installation"

    local output=$(docker run --rm "$IMAGE_NAME:$IMAGE_TAG" bash -c "source /opt/ros/jazzy/setup.bash && ros2 --version" 2>/dev/null)
    if [[ $? -eq 0 && "$output" == *"ros2 cli version"* ]]; then
        log_success "ROS2 is properly installed and functional"
    else
        log_fail "ROS2 installation or functionality issue"
        return 1
    fi
}

# Test Gazebo installation
test_gazebo_installation() {
    log_test "Gazebo installation"

    local output=$(docker run --rm "$IMAGE_NAME:$IMAGE_TAG" bash -c "gz sim --versions" 2>/dev/null | head -1)
    if [[ $? -eq 0 && "$output" == *"Gazebo"* ]]; then
        log_success "Gazebo is properly installed: $output"
    else
        log_fail "Gazebo installation or functionality issue"
        return 1
    fi
}

# Test NS-3 installation
test_ns3_installation() {
    log_test "NS-3 installation"

    if docker run --rm "$IMAGE_NAME:$IMAGE_TAG" bash -c "cd /workspace/ns3_gazebo/ns-allinone-3.45/ns-3.45 && ./ns3 run hello-simulator" &> /dev/null; then
        log_success "NS-3 is properly installed and functional"
    else
        log_fail "NS-3 installation or functionality issue"
        return 1
    fi
}

# Test build tools
test_build_tools() {
    log_test "Build tools availability"

    local tools=("cmake" "make" "gcc" "g++" "python3" "colcon")
    local missing_tools=()

    for tool in "${tools[@]}"; do
        if ! docker run --rm "$IMAGE_NAME:$IMAGE_TAG" command -v "$tool" &> /dev/null; then
            missing_tools+=("$tool")
        fi
    done

    if [[ ${#missing_tools[@]} -eq 0 ]]; then
        log_success "All required build tools are available"
    else
        log_fail "Missing build tools: ${missing_tools[*]}"
        return 1
    fi
}

# Test project build
test_project_build() {
    log_test "Project build functionality"

    # Test Gazebo plugin build
    if docker run --rm -v "$PROJECT_DIR:/workspace/ns3_gazebo" "$IMAGE_NAME:$IMAGE_TAG" bash -c "cd /workspace/ns3_gazebo/ns3_gazebo_plugin && cmake . && make" &> /dev/null; then
        log_success "Gazebo plugin builds successfully"
    else
        log_fail "Gazebo plugin build failed"
        return 1
    fi

    # Test ROS2 workspace build
    if docker run --rm -v "$PROJECT_DIR:/workspace/ns3_gazebo" "$IMAGE_NAME:$IMAGE_TAG" bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/ns3_gazebo/ns3_gazebo_ws && colcon build" &> /dev/null; then
        log_success "ROS2 workspace builds successfully"
    else
        log_fail "ROS2 workspace build failed"
        return 1
    fi
}

# Test network capabilities
test_network_capabilities() {
    log_test "Network capabilities"

    # Test basic networking
    if docker run --rm "$IMAGE_NAME:$IMAGE_TAG" ping -c 1 google.com &> /dev/null; then
        log_success "Basic network connectivity works"
    else
        log_fail "Basic network connectivity failed"
        return 1
    fi

    # Test network tools
    local tools=("ip" "ping" "netstat")
    for tool in "${tools[@]}"; do
        if ! docker run --rm "$IMAGE_NAME:$IMAGE_TAG" command -v "$tool" &> /dev/null; then
            log_fail "Network tool missing: $tool"
            return 1
        fi
    done

    log_success "Network tools are available"
}

# Test privileged capabilities
test_privileged_capabilities() {
    log_test "Privileged capabilities (network namespaces)"

    if docker run --rm --privileged --cap-add=NET_ADMIN "$IMAGE_NAME:$IMAGE_TAG" bash -c "ip netns add test-ns && ip netns delete test-ns" &> /dev/null; then
        log_success "Network namespace creation works in privileged mode"
    else
        log_fail "Network namespace functionality not available"
        return 1
    fi
}

# Test Docker Compose functionality
test_docker_compose() {
    log_test "Docker Compose functionality"

    cd "$PROJECT_DIR"

    if [ ! -f "docker-compose.yml" ]; then
        log_fail "docker-compose.yml not found"
        return 1
    fi

    # Validate compose file
    if docker-compose config &> /dev/null; then
        log_success "Docker Compose configuration is valid"
    else
        log_fail "Docker Compose configuration is invalid"
        return 1
    fi
}

# Test volume mounting
test_volume_mounting() {
    log_test "Volume mounting functionality"

    # Test source code mounting
    if docker run --rm -v "$PROJECT_DIR:/workspace/ns3_gazebo" "$IMAGE_NAME:$IMAGE_TAG" test -f "/workspace/ns3_gazebo/README.md"; then
        log_success "Source code volume mounting works"
    else
        log_fail "Source code volume mounting failed"
        return 1
    fi
}

# Generate test report
generate_report() {
    echo ""
    echo "==============================================="
    echo "           DOCKER TEST REPORT"
    echo "==============================================="
    echo ""

    echo "Test Results:"
    echo "  Total Tests: $TOTAL_TESTS"
    echo "  Passed: $PASSED_TESTS"
    echo "  Failed: $FAILED_TESTS"
    echo ""

    local success_rate=0
    if [[ $TOTAL_TESTS -gt 0 ]]; then
        success_rate=$((PASSED_TESTS * 100 / TOTAL_TESTS))
    fi

    if [[ $FAILED_TESTS -eq 0 ]]; then
        log_success "All Docker tests passed! (100%)"
        echo ""
        echo "🎉 Docker environment is ready to use!"
        echo ""
        echo "Quick start commands:"
        echo "  ./scripts/docker_run.sh                    # Interactive development"
        echo "  ./scripts/docker_run.sh -g                # With GUI support"
        echo "  ./scripts/docker_run.sh test              # Run verification tests"
        echo "  docker-compose up ns3-gazebo-dev          # Full environment"
        echo ""
    else
        log_fail "Docker tests failed: $FAILED_TESTS test(s) failed"
        echo ""
        echo "Common solutions:"
        echo "  - Rebuild the Docker image: ./scripts/docker_build.sh"
        echo "  - Check Docker installation and permissions"
        echo "  - Ensure sufficient system resources"
        echo ""
        return 1
    fi

    echo "Success rate: $success_rate%"
    echo ""

    return $FAILED_TESTS
}

# Main function
main() {
    echo "==============================================="
    echo "    NS3-Gazebo Docker Test Suite"
    echo "==============================================="
    echo ""

    log_info "Testing Docker environment for NS3-Gazebo..."
    log_info "This will take a few minutes..."
    echo ""

    # Run all tests
    test_image_exists || true
    test_basic_container || true
    test_ros2_installation || true
    test_gazebo_installation || true
    test_ns3_installation || true
    test_build_tools || true
    test_project_build || true
    test_network_capabilities || true
    test_privileged_capabilities || true
    test_docker_compose || true
    test_volume_mounting || true

    # Generate report
    generate_report
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi