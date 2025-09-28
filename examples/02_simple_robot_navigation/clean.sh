#!/bin/bash

# =============================================================================
# Clean Script for Simple Robot Navigation Example
# =============================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
OUTPUT_DIR="$SCRIPT_DIR/output"

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

# Show header
echo "==============================================="
echo "  Simple Robot Navigation Example - Clean"
echo "==============================================="
echo ""

log_info "Cleaning build and output files..."

# Clean build directory
if [ -d "$BUILD_DIR" ]; then
    log_info "Removing build directory: $BUILD_DIR"
    rm -rf "$BUILD_DIR"
    log_success "Build directory removed"
else
    log_info "Build directory does not exist"
fi

# Clean output directory
if [ -d "$OUTPUT_DIR" ]; then
    log_info "Removing output directory: $OUTPUT_DIR"
    rm -rf "$OUTPUT_DIR"
    log_success "Output directory removed"
else
    log_info "Output directory does not exist"
fi

# Clean generated map files
if [ -f "$SCRIPT_DIR/maps/simple_map.pgm" ]; then
    log_info "Removing generated map files..."
    rm -f "$SCRIPT_DIR/maps/simple_map.pgm"
    rm -f "$SCRIPT_DIR/maps/simple_map.yaml"
    log_success "Map files removed"
fi

# Clean RViz config if it was generated
if [ -f "$SCRIPT_DIR/config/robot_nav.rviz" ]; then
    log_info "Removing generated RViz config..."
    rm -f "$SCRIPT_DIR/config/robot_nav.rviz"
    log_success "RViz config removed"
fi

# Clean any trace files
TRACE_FILES=$(find "$SCRIPT_DIR" -maxdepth 1 -name "*.tr" -o -name "*.pcap" -o -name "*.bag" -o -name "*.log" 2>/dev/null)
if [ -n "$TRACE_FILES" ]; then
    log_info "Removing trace and log files..."
    echo "$TRACE_FILES" | while read -r file; do
        rm -f "$file"
        log_info "Removed: $(basename "$file")"
    done
    log_success "Trace and log files removed"
else
    log_info "No trace or log files found"
fi

# Clean Python cache
PYCACHE_DIRS=$(find "$SCRIPT_DIR" -type d -name "__pycache__" 2>/dev/null)
if [ -n "$PYCACHE_DIRS" ]; then
    log_info "Removing Python cache directories..."
    echo "$PYCACHE_DIRS" | while read -r dir; do
        rm -rf "$dir"
        log_info "Removed: $dir"
    done
    log_success "Python cache directories removed"
fi

# Clean ROS2 workspace if it exists
WS_DIR="$SCRIPT_DIR/../../ns3_gazebo_ws"
if [ -d "$WS_DIR/build" ] || [ -d "$WS_DIR/install" ] || [ -d "$WS_DIR/log" ]; then
    read -p "Clean ROS2 workspace build files? [y/N]: " clean_ws
    if [[ $clean_ws =~ ^[Yy]$ ]]; then
        log_info "Cleaning ROS2 workspace..."
        cd "$WS_DIR"
        rm -rf build install log
        log_success "ROS2 workspace cleaned"
    fi
fi

# Kill any remaining processes
log_info "Checking for running processes..."
RUNNING_PROCS=$(pgrep -f "robot_navigation|gazebo|rviz2" 2>/dev/null || true)
if [ -n "$RUNNING_PROCS" ]; then
    log_warning "Found running simulation processes"
    echo "$RUNNING_PROCS" | while read -r pid; do
        PROC_NAME=$(ps -p "$pid" -o comm= 2>/dev/null || echo "unknown")
        log_info "Process: $PROC_NAME (PID: $pid)"
    done

    read -p "Kill these processes? [y/N]: " kill_procs
    if [[ $kill_procs =~ ^[Yy]$ ]]; then
        echo "$RUNNING_PROCS" | xargs kill 2>/dev/null || true
        sleep 2
        # Force kill if still running
        STILL_RUNNING=$(pgrep -f "robot_navigation|gazebo|rviz2" 2>/dev/null || true)
        if [ -n "$STILL_RUNNING" ]; then
            echo "$STILL_RUNNING" | xargs kill -9 2>/dev/null || true
        fi
        log_success "Processes terminated"
    fi
else
    log_info "No running simulation processes found"
fi

# Clean temporary files
TEMP_FILES=$(find /tmp -name "*gazebo*" -o -name "*ros*" -o -name "*rviz*" 2>/dev/null | head -10)
if [ -n "$TEMP_FILES" ]; then
    log_info "Found temporary files (showing first 10):"
    echo "$TEMP_FILES" | while read -r file; do
        log_info "  $file"
    done

    read -p "Clean temporary files? [y/N]: " clean_temp
    if [[ $clean_temp =~ ^[Yy]$ ]]; then
        find /tmp -name "*gazebo*" -o -name "*ros*" -o -name "*rviz*" 2>/dev/null | xargs rm -rf 2>/dev/null || true
        log_success "Temporary files cleaned"
    fi
fi

echo ""
log_success "Cleanup completed successfully!"
echo ""
echo "What was cleaned:"
echo "  - Build directory and compiled files"
echo "  - Output directory and simulation logs"
echo "  - Generated map and configuration files"
echo "  - Trace files and network captures"
echo "  - Python cache directories"
echo "  - Running simulation processes (if requested)"
echo ""
echo "To rebuild the example:"
echo "  ./build.sh"
echo ""
echo "To run the example:"
echo "  ./run.sh"
echo ""