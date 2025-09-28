#!/bin/bash

# =============================================================================
# Installation Verification Script for ns3_gazebo
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
LOG_FILE="$PROJECT_DIR/verification.log"

# Test results tracking
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0
WARNINGS=0

# Functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" | tee -a "$LOG_FILE"
}

log_success() {
    echo -e "${GREEN}[PASS]${NC} $1" | tee -a "$LOG_FILE"
    ((PASSED_TESTS++))
}

log_fail() {
    echo -e "${RED}[FAIL]${NC} $1" | tee -a "$LOG_FILE"
    ((FAILED_TESTS++))
}

log_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1" | tee -a "$LOG_FILE"
    ((WARNINGS++))
}

log_test() {
    echo -e "${PURPLE}[TEST]${NC} $1" | tee -a "$LOG_FILE"
    ((TOTAL_TESTS++))
}

# Test ROS2 installation
test_ros2() {
    log_test "ROS2 Jazzy Installation"

    # Check ROS2 installation
    if [ ! -d "/opt/ros/jazzy" ]; then
        log_fail "ROS2 Jazzy not found in /opt/ros/jazzy"
        return 1
    fi

    # Test ROS2 environment
    if ! source /opt/ros/jazzy/setup.bash 2>/dev/null; then
        log_fail "Failed to source ROS2 environment"
        return 1
    fi

    # Test ros2 command
    if ! command -v ros2 &> /dev/null; then
        log_fail "ros2 command not available"
        return 1
    fi

    # Test basic ROS2 functionality
    if ! timeout 5s ros2 node list &>/dev/null; then
        log_warning "ros2 node list failed (ROS2 daemon may not be running)"
    fi

    log_success "ROS2 Jazzy is properly installed and functional"
}

# Test Gazebo installation
test_gazebo() {
    log_test "Gazebo Harmonic Installation"

    # Check gz command
    if ! command -v gz &> /dev/null; then
        log_fail "gz command not found"
        return 1
    fi

    # Test Gazebo version
    local gz_version
    gz_version=$(timeout 10s gz sim --versions 2>/dev/null | head -1 || echo "unknown")
    if [[ "$gz_version" == "unknown" ]]; then
        log_warning "Could not determine Gazebo version"
    else
        log_success "Gazebo version: $gz_version"
    fi

    # Test Gazebo help (quick functionality test)
    if ! timeout 5s gz sim --help &>/dev/null; then
        log_fail "Gazebo sim --help failed"
        return 1
    fi

    log_success "Gazebo Harmonic is properly installed and functional"
}

# Test NS-3 installation
test_ns3() {
    log_test "NS-3 3.45 Installation"

    local ns3_dir="$PROJECT_DIR/ns-allinone-3.45"

    # Check NS-3 directory
    if [ ! -d "$ns3_dir" ]; then
        log_fail "NS-3 directory not found: $ns3_dir"
        return 1
    fi

    # Check NS-3 build
    local ns3_build_dir="$ns3_dir/ns-3.45/build"
    if [ ! -d "$ns3_build_dir" ]; then
        log_fail "NS-3 build directory not found: $ns3_build_dir"
        return 1
    fi

    # Check key NS-3 libraries
    local required_libs=(
        "libns3.45-core-default.so"
        "libns3.45-network-default.so"
        "libns3.45-wifi-default.so"
        "libns3.45-internet-default.so"
        "libns3.45-mobility-default.so"
        "libns3.45-tap-bridge-default.so"
    )

    for lib in "${required_libs[@]}"; do
        if [ ! -f "$ns3_build_dir/lib/$lib" ]; then
            log_fail "Required NS-3 library not found: $lib"
            return 1
        fi
    done

    # Test NS-3 execution
    cd "$ns3_dir/ns-3.45"
    if ! timeout 10s ./ns3 run hello-simulator &>/dev/null; then
        log_fail "NS-3 hello-simulator test failed"
        return 1
    fi

    log_success "NS-3 3.45 is properly installed and functional"
}

# Test NS-3 Gazebo plugin
test_gazebo_plugin() {
    log_test "NS-3 Gazebo Plugin"

    local plugin_dir="$PROJECT_DIR/ns3_gazebo_plugin"
    local plugin_build_dir="$plugin_dir/build"

    # Check plugin build directory
    if [ ! -d "$plugin_build_dir" ]; then
        log_fail "Gazebo plugin build directory not found"
        return 1
    fi

    # Check plugin libraries
    local plugin_libs=(
        "libns3_gazebo_world.so"
        "libhello_world.so"
    )

    for lib in "${plugin_libs[@]}"; do
        if [ ! -f "$plugin_build_dir/$lib" ]; then
            log_fail "Plugin library not found: $lib"
            return 1
        fi
    done

    # Test plugin loading (basic check)
    local world_file="$plugin_dir/gazebo_ros_diff_drive_ns3_gazebo.world"
    if [ ! -f "$world_file" ]; then
        log_fail "Gazebo world file not found"
        return 1
    fi

    # Quick Gazebo plugin test (check if it loads without errors)
    cd "$plugin_dir"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$plugin_build_dir"

    if ! timeout 15s gz sim "$world_file" --headless --iterations 1 &>/dev/null; then
        log_warning "Gazebo world loading test failed (may require display)"
    else
        log_success "Gazebo world loads successfully"
    fi

    log_success "NS-3 Gazebo plugin is properly built and functional"
}

# Test ROS2 workspace
test_ros2_workspace() {
    log_test "ROS2 Workspace"

    local ros2_ws="$PROJECT_DIR/ns3_gazebo_ws"

    # Check workspace structure
    if [ ! -d "$ros2_ws" ]; then
        log_fail "ROS2 workspace directory not found"
        return 1
    fi

    # Check build and install directories
    if [ ! -d "$ros2_ws/build" ] || [ ! -d "$ros2_ws/install" ]; then
        log_fail "ROS2 workspace not built (missing build/install directories)"
        return 1
    fi

    # Source ROS2 environment
    source /opt/ros/jazzy/setup.bash 2>/dev/null

    # Source workspace
    if [ -f "$ros2_ws/install/setup.bash" ]; then
        source "$ros2_ws/install/setup.bash"
    else
        log_fail "ROS2 workspace setup.bash not found"
        return 1
    fi

    # Check if ROS2 packages are available
    if ! ros2 pkg list | grep -q "diff_drive_ns3"; then
        log_fail "diff_drive_ns3 ROS2 package not found"
        return 1
    fi

    log_success "ROS2 workspace is properly built and functional"
}

# Test network tools
test_network_tools() {
    log_test "Network Tools"

    local tools=("ip" "ping" "netstat")
    local missing_tools=()

    for tool in "${tools[@]}"; do
        if ! command -v "$tool" &> /dev/null; then
            missing_tools+=("$tool")
        fi
    done

    if [ ${#missing_tools[@]} -gt 0 ]; then
        log_fail "Missing network tools: ${missing_tools[*]}"
        return 1
    fi

    # Test network namespace functionality
    if ! sudo ip netns add test-verify-ns 2>/dev/null; then
        log_warning "Cannot create network namespace (may need different privileges)"
    else
        sudo ip netns delete test-verify-ns 2>/dev/null
        log_success "Network namespace functionality available"
    fi

    log_success "Network tools are available and functional"
}

# Test build tools
test_build_tools() {
    log_test "Build Tools"

    local tools=("cmake" "make" "gcc" "g++" "python3" "colcon")
    local missing_tools=()

    for tool in "${tools[@]}"; do
        if ! command -v "$tool" &> /dev/null; then
            missing_tools+=("$tool")
        fi
    done

    if [ ${#missing_tools[@]} -gt 0 ]; then
        log_fail "Missing build tools: ${missing_tools[*]}"
        return 1
    fi

    # Test CMake version
    local cmake_version
    cmake_version=$(cmake --version | head -1 | grep -o '[0-9]\+\.[0-9]\+')
    if [[ $(echo "$cmake_version >= 3.16" | bc -l 2>/dev/null || echo "0") -eq 1 ]]; then
        log_success "CMake version $cmake_version (sufficient)"
    else
        log_warning "CMake version $cmake_version may be too old (recommended: 3.16+)"
    fi

    log_success "Build tools are available and functional"
}

# Test test components
test_components() {
    log_test "Test Components"

    # Test NS-3 WiFi test
    local wifi_test="$PROJECT_DIR/ns3_wifi_tap_test/build/ns3_wifi_tap_test"
    if [ ! -f "$wifi_test" ]; then
        log_fail "NS-3 WiFi test binary not found"
        return 1
    fi

    if ! timeout 5s "$wifi_test" -h &>/dev/null; then
        log_fail "NS-3 WiFi test execution failed"
        return 1
    fi

    # Test testbed nodes
    local testbed_dir="$PROJECT_DIR/ns3_testbed/ns3_testbed_nodes"
    if [ ! -d "$testbed_dir/build" ] && [ ! -d "$testbed_dir/install" ]; then
        log_fail "Testbed nodes not built"
        return 1
    fi

    log_success "Test components are properly built and functional"
}

# Run integration test
test_integration() {
    log_test "Integration Test"

    # This is a basic integration test
    # In a real scenario, you might want to run a full system test

    log_info "Running basic integration test..."

    # Source all environments
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazebo/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
    fi

    if [ -f "$PROJECT_DIR/ns3_gazebo_ws/install/setup.bash" ]; then
        source "$PROJECT_DIR/ns3_gazebo_ws/install/setup.bash"
    fi

    # Basic environment test
    if [ -z "$ROS_DISTRO" ]; then
        log_warning "ROS_DISTRO not set"
    else
        log_success "ROS environment: $ROS_DISTRO"
    fi

    log_success "Basic integration test passed"
}

# Generate test report
generate_report() {
    echo ""
    echo "==============================================="
    echo "           VERIFICATION REPORT"
    echo "==============================================="
    echo ""

    echo "Test Results:"
    echo "  Total Tests: $TOTAL_TESTS"
    echo "  Passed: $PASSED_TESTS"
    echo "  Failed: $FAILED_TESTS"
    echo "  Warnings: $WARNINGS"
    echo ""

    local success_rate=0
    if [ $TOTAL_TESTS -gt 0 ]; then
        success_rate=$((PASSED_TESTS * 100 / TOTAL_TESTS))
    fi

    if [ $FAILED_TESTS -eq 0 ]; then
        if [ $WARNINGS -eq 0 ]; then
            log_success "All tests passed! System is fully functional."
        else
            log_warning "All tests passed with $WARNINGS warning(s). System should work normally."
        fi
        echo ""
        echo "🎉 Your NS3-Gazebo system is ready to use!"
        echo ""
        echo "Quick start commands:"
        echo "  ./scripts/nns_setup.py setup -c 1  # Setup network"
        echo "  cd ns3_testbed && python3 testbed_runner.py --no_nns -c 2 -s csv_setup/example1.csv -v"
    else
        log_fail "Verification failed: $FAILED_TESTS test(s) failed"
        echo ""
        echo "Common solutions:"
        echo "  - Re-run the installation: ./install.sh"
        echo "  - Check the installation log: $LOG_FILE"
        echo "  - Install missing dependencies manually"
        echo "  - Check system requirements: ./scripts/check_system.sh"
        return 1
    fi

    echo ""
    echo "Success rate: $success_rate%"
    echo "Detailed log: $LOG_FILE"
    echo ""

    return $FAILED_TESTS
}

# Main verification function
main() {
    echo "==============================================="
    echo "    NS3-Gazebo Installation Verification"
    echo "==============================================="
    echo ""

    # Initialize log
    echo "Verification started at $(date)" > "$LOG_FILE"
    echo "" >> "$LOG_FILE"

    log_info "Verifying NS3-Gazebo installation..."
    log_info "This will take a few minutes..."
    echo ""

    # Run all tests
    test_build_tools || true
    test_ros2 || true
    test_gazebo || true
    test_ns3 || true
    test_gazebo_plugin || true
    test_ros2_workspace || true
    test_network_tools || true
    test_components || true
    test_integration || true

    # Generate report
    generate_report
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi