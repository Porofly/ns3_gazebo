#!/bin/bash

# =============================================================================
# Build Script for Hello World Network Example
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

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

# Show header
echo "==============================================="
echo "    Hello World Network Example - Build"
echo "==============================================="
echo ""

# Check dependencies
log_info "Checking dependencies..."

# Check if NS-3 is available
if ! pkg-config --exists ns3.45-core; then
    log_error "NS-3 3.45 not found"
    log_info "Please ensure NS-3 is installed and pkg-config can find it"
    exit 1
fi

log_success "NS-3 3.45 found"

# Check if we're in the right directory
if [ ! -f "$SCRIPT_DIR/hello_world.cc" ]; then
    log_error "hello_world.cc not found"
    log_info "Make sure you're running this script from the example directory"
    exit 1
fi

# Create build directory
log_info "Creating build directory..."
mkdir -p "$BUILD_DIR"

# Configure with CMake
log_info "Configuring with CMake..."
cd "$BUILD_DIR"

if ! cmake .. 2>&1; then
    log_error "CMake configuration failed"
    exit 1
fi

log_success "CMake configuration completed"

# Build the project
log_info "Building the project..."

# Determine number of build jobs
BUILD_JOBS=$(nproc 2>/dev/null || echo "4")
log_info "Using $BUILD_JOBS parallel jobs"

if ! make -j"$BUILD_JOBS" 2>&1; then
    log_error "Build failed"
    exit 1
fi

log_success "Build completed successfully"

# Verify executable
if [ -f "$BUILD_DIR/hello_world_network" ]; then
    log_success "Executable created: hello_world_network"
else
    log_error "Executable not found after build"
    exit 1
fi

echo ""
echo "==============================================="
echo "         BUILD COMPLETED SUCCESSFULLY!"
echo "==============================================="
echo ""
echo "Next steps:"
echo "  1. Run the example: ./run.sh"
echo "  2. Or run manually:"
echo "     cd build && ./hello_world_network"
echo ""
echo "Files created:"
echo "  - build/hello_world_network (executable)"
echo "  - build/Makefile (build configuration)"
echo ""