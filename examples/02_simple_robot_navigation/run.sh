#!/bin/bash

# =============================================================================
# Run Script for Simple Robot Navigation Example
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
LAUNCH_FILE="$SCRIPT_DIR/robot_navigation.launch.py"

# Options
HEADLESS=false
ENABLE_NETWORK=true
ROBOT_X=0.0
ROBOT_Y=0.0
ROBOT_YAW=0.0
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

log_step() {
    echo -e "${PURPLE}[STEP]${NC} $1"
}

# Show help
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  --headless              Run Gazebo in headless mode"
    echo "  --no-network            Disable network simulation"
    echo "  --robot-x X             Initial robot X position (default: 0.0)"
    echo "  --robot-y Y             Initial robot Y position (default: 0.0)"
    echo "  --robot-yaw YAW         Initial robot yaw orientation (default: 0.0)"
    echo "  -v, --verbose           Enable verbose output"
    echo ""
    echo "Examples:"
    echo "  $0                      # Run with default settings"
    echo "  $0 --headless           # Run without GUI"
    echo "  $0 --no-network         # Run without network simulation"
    echo "  $0 --robot-x 2 --robot-y 1  # Start robot at position (2,1)"
    echo ""
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --headless)
            HEADLESS=true
            shift
            ;;
        --no-network)
            ENABLE_NETWORK=false
            shift
            ;;
        --robot-x)
            ROBOT_X="$2"
            shift 2
            ;;
        --robot-y)
            ROBOT_Y="$2"
            shift 2
            ;;
        --robot-yaw)
            ROBOT_YAW="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        *)
            log_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Show header
echo "==============================================="
echo "  Simple Robot Navigation Example - Run"
echo "==============================================="
echo ""

# Check prerequisites
log_step "Checking prerequisites..."

# Check if built
if [ ! -f "$SCRIPT_DIR/maps/simple_map.yaml" ]; then
    log_error "Example not built. Please run: ./build.sh"
    exit 1
fi

# Check ROS2
if ! command -v ros2 &> /dev/null; then
    log_error "ROS2 not found"
    exit 1
fi

# Check Gazebo
if ! command -v gz &> /dev/null; then
    log_error "Gazebo not found"
    exit 1
fi

log_success "Prerequisites check completed"

# Setup environment
log_step "Setting up environment..."

# Source ROS2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    log_info "ROS2 Jazzy environment loaded"
else
    log_error "ROS2 setup script not found"
    exit 1
fi

# Source workspace if available
WS_DIR="$SCRIPT_DIR/../../ns3_gazebo_ws"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    log_info "NS3-Gazebo workspace loaded"
fi

# Set Gazebo plugin paths
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SCRIPT_DIR/../../ns3_gazebo_plugin/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GAZEBO_MODEL_PATH="$SCRIPT_DIR:$GAZEBO_MODEL_PATH"

# Create output directory
OUTPUT_DIR="$SCRIPT_DIR/output"
mkdir -p "$OUTPUT_DIR"
cd "$OUTPUT_DIR"

log_success "Environment setup completed"

# Function to cleanup processes
cleanup() {
    log_info "Cleaning up processes..."
    # Kill all background processes in this process group
    pkill -P $$ 2>/dev/null || true
    # Give processes time to clean up
    sleep 2
}

# Set trap for cleanup
trap cleanup EXIT INT

# Launch the system
log_step "Launching robot navigation system..."

echo ""
echo "Configuration:"
echo "  Headless mode: $HEADLESS"
echo "  Network simulation: $ENABLE_NETWORK"
echo "  Robot position: ($ROBOT_X, $ROBOT_Y, $ROBOT_YAW)"
echo "  Verbose output: $VERBOSE"
echo ""

log_info "Starting ROS2 launch system..."

# Build launch command
LAUNCH_CMD="ros2 launch $LAUNCH_FILE"
LAUNCH_CMD="$LAUNCH_CMD headless:=$HEADLESS"
LAUNCH_CMD="$LAUNCH_CMD enable_network:=$ENABLE_NETWORK"
LAUNCH_CMD="$LAUNCH_CMD x_pose:=$ROBOT_X"
LAUNCH_CMD="$LAUNCH_CMD y_pose:=$ROBOT_Y"
LAUNCH_CMD="$LAUNCH_CMD yaw:=$ROBOT_YAW"

if [ "$VERBOSE" = true ]; then
    log_info "Launch command: $LAUNCH_CMD"
fi

echo ""
echo "========================================"
echo "         STARTING SIMULATION"
echo "========================================"
echo ""

log_info "Robot navigation system starting..."
log_info "This may take 30-60 seconds to fully initialize"
echo ""

# Execute launch command
if [ "$VERBOSE" = true ]; then
    eval "$LAUNCH_CMD"
else
    eval "$LAUNCH_CMD" 2>&1 | while IFS= read -r line; do
        # Filter out common startup messages to reduce noise
        if [[ "$line" =~ (INFO|WARN|ERROR|FATAL) ]] && \
           [[ ! "$line" =~ (parameter|service|action|topic) ]]; then
            echo "$line"
        elif [[ "$line" =~ (Goal|Navigation|Robot|Network) ]]; then
            echo "$line"
        fi
    done
fi

LAUNCH_EXIT_CODE=$?

echo ""
echo "========================================"
echo "         SIMULATION ENDED"
echo "========================================"
echo ""

if [ $LAUNCH_EXIT_CODE -eq 0 ]; then
    log_success "Robot navigation example completed successfully"
    echo ""
    echo "🎉 What happened:"
    echo "  - Robot was spawned in Gazebo environment"
    echo "  - Navigation system was initialized"
    echo "  - Robot autonomously navigated to goal positions"
    echo "  - Network performance was monitored in real-time"
    echo ""
    echo "Generated files in output/:"
    ls -la "$OUTPUT_DIR" 2>/dev/null | grep -E '\.(log|bag|pcap)$' | while read -r line; do
        echo "  - $line"
    done
else
    log_error "Robot navigation example failed with exit code: $LAUNCH_EXIT_CODE"
    echo ""
    echo "Common solutions:"
    echo "  - Ensure all dependencies are installed"
    echo "  - Check that ROS2 navigation stack is installed"
    echo "  - Verify Gazebo Harmonic is working"
    echo "  - Try building again: ./build.sh"
    echo "  - Check logs in output/ directory"
fi

echo ""
echo "Next steps:"
echo "  - Review navigation performance logs"
echo "  - Analyze network trace files"
echo "  - Try different robot starting positions"
echo "  - Experiment with WiFi configuration parameters"
echo "  - Progress to the next example: 03_wifi_performance_test"
echo ""

# Show system resource usage
if command -v ps &> /dev/null; then
    log_info "System resource usage during simulation:"
    ps aux | grep -E "(gazebo|ros2|rviz)" | grep -v grep | while read -r line; do
        echo "  $line" | awk '{printf "  %-15s CPU: %s%% MEM: %s%%\n", $11, $3, $4}'
    done
fi

echo ""
exit $LAUNCH_EXIT_CODE