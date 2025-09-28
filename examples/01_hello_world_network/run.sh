#!/bin/bash

# =============================================================================
# Run Script for Hello World Network Example
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
BUILD_DIR="$SCRIPT_DIR/build"
EXECUTABLE="$BUILD_DIR/hello_world_network"
WORLD_FILE="$SCRIPT_DIR/hello_world.world"

# Options
GUI_MODE=true
HEADLESS=false
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
    echo "  -h, --help      Show this help message"
    echo "  --headless      Run without GUI (headless mode)"
    echo "  -v, --verbose   Enable verbose output"
    echo ""
    echo "Examples:"
    echo "  $0              # Run with GUI"
    echo "  $0 --headless   # Run without GUI"
    echo "  $0 -v           # Run with verbose output"
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
            GUI_MODE=false
            shift
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
echo "    Hello World Network Example - Run"
echo "==============================================="
echo ""

# Check if built
log_step "Checking build status..."

if [ ! -f "$EXECUTABLE" ]; then
    log_error "Executable not found: $EXECUTABLE"
    log_info "Please build the example first: ./build.sh"
    exit 1
fi

log_success "Executable found"

# Check world file
if [ ! -f "$WORLD_FILE" ]; then
    log_error "World file not found: $WORLD_FILE"
    exit 1
fi

log_success "World file found"

# Setup environment
log_step "Setting up environment..."

# Source ROS2 if available
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    log_info "ROS2 Jazzy environment loaded"
fi

# Set Gazebo plugin path
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SCRIPT_DIR/../../ns3_gazebo_plugin/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
log_info "Gazebo plugin path configured"

# Create output directory for traces
OUTPUT_DIR="$SCRIPT_DIR/output"
mkdir -p "$OUTPUT_DIR"
cd "$OUTPUT_DIR"

# Function to cleanup processes
cleanup() {
    log_info "Cleaning up processes..."
    if [ ! -z "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        kill "$GAZEBO_PID"
        wait "$GAZEBO_PID" 2>/dev/null || true
    fi
}

# Set trap for cleanup
trap cleanup EXIT

# Start Gazebo simulation
log_step "Starting Gazebo simulation..."

if [ "$HEADLESS" = true ]; then
    log_info "Running in headless mode"
    gz sim "$WORLD_FILE" --headless --iterations 1000 &
else
    log_info "Running with GUI"
    gz sim "$WORLD_FILE" &
fi

GAZEBO_PID=$!

# Wait for Gazebo to initialize
log_info "Waiting for Gazebo to initialize..."
sleep 3

# Check if Gazebo started successfully
if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    log_error "Gazebo failed to start"
    exit 1
fi

log_success "Gazebo started successfully (PID: $GAZEBO_PID)"

# Run NS-3 simulation
log_step "Starting NS-3 simulation..."

echo ""
echo "========================================"
echo "         NS-3 SIMULATION OUTPUT"
echo "========================================"
echo ""

# Run the simulation
if [ "$VERBOSE" = true ]; then
    "$EXECUTABLE" --verbose
else
    "$EXECUTABLE"
fi

SIMULATION_EXIT_CODE=$?

echo ""
echo "========================================"
echo "         SIMULATION COMPLETED"
echo "========================================"
echo ""

if [ $SIMULATION_EXIT_CODE -eq 0 ]; then
    log_success "NS-3 simulation completed successfully"
else
    log_error "NS-3 simulation failed with exit code: $SIMULATION_EXIT_CODE"
fi

# Show generated files
log_step "Checking output files..."

if [ -f "hello_world.tr" ]; then
    log_success "ASCII trace file generated: hello_world.tr"
fi

if ls hello_world-*.pcap 1> /dev/null 2>&1; then
    log_success "PCAP files generated: $(ls hello_world-*.pcap | wc -l) files"
fi

# Keep Gazebo running for a bit to see the visualization
if [ "$GUI_MODE" = true ] && [ $SIMULATION_EXIT_CODE -eq 0 ]; then
    log_info "Keeping Gazebo running for visualization..."
    log_info "Press Ctrl+C to exit, or wait 10 seconds"

    # Wait for either Ctrl+C or timeout
    timeout 10s sleep infinity 2>/dev/null || true
fi

# Final status
echo ""
if [ $SIMULATION_EXIT_CODE -eq 0 ]; then
    echo "🎉 Hello World Network Example completed successfully!"
    echo ""
    echo "What happened:"
    echo "  - Two WiFi nodes were created 5 meters apart"
    echo "  - Node 0 (green) sent 'Hello World' packets to Node 1 (blue)"
    echo "  - All packets were transmitted and received successfully"
    echo "  - Network trace files were generated for analysis"
    echo ""
    echo "Generated files:"
    echo "  - hello_world.tr (ASCII trace)"
    echo "  - hello_world-*.pcap (packet capture files)"
    echo ""
    echo "Next steps:"
    echo "  - Examine trace files with Wireshark or text editor"
    echo "  - Try modifying node positions in hello_world.world"
    echo "  - Experiment with different WiFi parameters"
    echo "  - Progress to the next example: 02_simple_robot_navigation"
else
    echo "❌ Example failed. Check the error messages above."
    echo ""
    echo "Common solutions:"
    echo "  - Ensure NS-3 is properly installed"
    echo "  - Check that Gazebo Harmonic is working"
    echo "  - Verify all dependencies are installed"
    echo "  - Try building again: ./build.sh"
fi

echo ""