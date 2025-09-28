#!/bin/bash

# =============================================================================
# Clean Script for Hello World Network Example
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
echo "    Hello World Network Example - Clean"
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

# Clean any trace files in current directory
TRACE_FILES=$(find "$SCRIPT_DIR" -maxdepth 1 -name "*.tr" -o -name "*.pcap" 2>/dev/null)
if [ -n "$TRACE_FILES" ]; then
    log_info "Removing trace files..."
    echo "$TRACE_FILES" | while read -r file; do
        rm -f "$file"
        log_info "Removed: $(basename "$file")"
    done
    log_success "Trace files removed"
else
    log_info "No trace files found"
fi

# Clean CMake cache files
CMAKE_FILES=$(find "$SCRIPT_DIR" -maxdepth 1 -name "CMakeCache.txt" -o -name "CMakeFiles" 2>/dev/null)
if [ -n "$CMAKE_FILES" ]; then
    log_info "Removing CMake cache files..."
    rm -rf $CMAKE_FILES
    log_success "CMake cache files removed"
fi

echo ""
log_success "Cleanup completed successfully!"
echo ""
echo "What was cleaned:"
echo "  - Build directory and all compiled files"
echo "  - Output directory and trace files"
echo "  - CMake cache files"
echo ""
echo "To rebuild the example:"
echo "  ./build.sh"
echo ""