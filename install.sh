#!/bin/bash

# =============================================================================
# NS3-Gazebo System - Main Installation Script
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="$SCRIPT_DIR/install.log"
START_TIME=$(date +%s)

# Default options
SKIP_DEPS=false
SKIP_NS3=false
BUILD_ONLY=false
INTERACTIVE=true
INSTALL_MODE="full"  # full, minimal, dev

# Progress tracking
TOTAL_STEPS=8
CURRENT_STEP=0

# Functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" | tee -a "$LOG_FILE"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" | tee -a "$LOG_FILE"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$LOG_FILE"
}

log_step() {
    ((CURRENT_STEP++))
    echo -e "${PURPLE}[STEP $CURRENT_STEP/$TOTAL_STEPS]${NC} $1" | tee -a "$LOG_FILE"
}

log_header() {
    echo -e "${CYAN}$1${NC}" | tee -a "$LOG_FILE"
}

show_progress() {
    local current=$1
    local total=$2
    local width=50
    local percentage=$((current * 100 / total))
    local filled=$((current * width / total))

    printf "\r${WHITE}Progress: ["
    printf "%*s" $filled | tr ' ' '█'
    printf "%*s" $((width - filled)) | tr ' ' '░'
    printf "] %d%%${NC}" $percentage
}

# Show banner
show_banner() {
    echo -e "${CYAN}"
    echo "==============================================="
    echo "    NS3-Gazebo Integration System Installer"
    echo "==============================================="
    echo ""
    echo "  🌐 NS-3 Network Simulator 3.45"
    echo "  🤖 Gazebo Harmonic Physics Simulation"
    echo "  🔗 ROS2 Jazzy Integration"
    echo ""
    echo "==============================================="
    echo -e "${NC}"
}

# Show help
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -y, --yes              Non-interactive mode (use defaults)"
    echo "  --skip-deps            Skip dependency installation"
    echo "  --skip-ns3             Skip NS-3 installation"
    echo "  --build-only           Only build existing components"
    echo "  --mode MODE            Installation mode: full, minimal, dev"
    echo "  --log-file FILE        Custom log file path"
    echo ""
    echo "Installation Modes:"
    echo "  full                   Complete installation (default)"
    echo "  minimal                Basic installation without examples"
    echo "  dev                    Development setup with debug symbols"
    echo ""
    echo "Examples:"
    echo "  $0                     # Interactive full installation"
    echo "  $0 -y                  # Non-interactive installation"
    echo "  $0 --build-only        # Only build, skip dependency installation"
    echo "  $0 --mode minimal      # Minimal installation"
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
            -y|--yes)
                INTERACTIVE=false
                shift
                ;;
            --skip-deps)
                SKIP_DEPS=true
                shift
                ;;
            --skip-ns3)
                SKIP_NS3=true
                shift
                ;;
            --build-only)
                BUILD_ONLY=true
                SKIP_DEPS=true
                SKIP_NS3=true
                shift
                ;;
            --mode)
                INSTALL_MODE="$2"
                if [[ ! "$INSTALL_MODE" =~ ^(full|minimal|dev)$ ]]; then
                    log_error "Invalid mode: $INSTALL_MODE"
                    exit 1
                fi
                shift 2
                ;;
            --log-file)
                LOG_FILE="$2"
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

# Interactive configuration
interactive_config() {
    if [[ "$INTERACTIVE" == "false" ]]; then
        return 0
    fi

    echo ""
    log_header "Installation Configuration"
    echo ""

    # Installation mode
    echo "Available installation modes:"
    echo "  1) Full        - Complete installation with all features"
    echo "  2) Minimal     - Basic installation without examples"
    echo "  3) Development - Full installation with debug symbols"
    echo ""
    read -p "Select installation mode [1-3] (default: 1): " mode_choice

    case $mode_choice in
        2) INSTALL_MODE="minimal" ;;
        3) INSTALL_MODE="dev" ;;
        *) INSTALL_MODE="full" ;;
    esac

    # Dependencies
    if command -v ros2 &> /dev/null && command -v gz &> /dev/null; then
        read -p "Dependencies seem to be installed. Skip dependency installation? [y/N]: " skip_deps
        if [[ $skip_deps =~ ^[Yy]$ ]]; then
            SKIP_DEPS=true
        fi
    fi

    # NS-3
    if [ -d "ns-allinone-3.45" ]; then
        read -p "NS-3 directory exists. Skip NS-3 installation? [y/N]: " skip_ns3
        if [[ $skip_ns3 =~ ^[Yy]$ ]]; then
            SKIP_NS3=true
        fi
    fi

    echo ""
    log_info "Configuration:"
    log_info "  Mode: $INSTALL_MODE"
    log_info "  Skip dependencies: $SKIP_DEPS"
    log_info "  Skip NS-3: $SKIP_NS3"
    echo ""

    read -p "Proceed with installation? [Y/n]: " proceed
    if [[ $proceed =~ ^[Nn]$ ]]; then
        log_info "Installation cancelled by user"
        exit 0
    fi
}

# Check system requirements
check_requirements() {
    log_step "Checking system requirements"

    if [ ! -f "$SCRIPT_DIR/scripts/check_system.sh" ]; then
        log_error "System check script not found"
        return 1
    fi

    log_info "Running system requirements check..."
    if ! "$SCRIPT_DIR/scripts/check_system.sh" >> "$LOG_FILE" 2>&1; then
        log_warning "System check found issues (continuing anyway)"
        log_info "Check the log file for details: $LOG_FILE"
    else
        log_success "System requirements satisfied"
    fi
}

# Install dependencies
install_dependencies() {
    if [[ "$SKIP_DEPS" == "true" ]]; then
        log_step "Skipping dependency installation (as requested)"
        return 0
    fi

    log_step "Installing system dependencies"

    if [ ! -f "$SCRIPT_DIR/scripts/install_dependencies.sh" ]; then
        log_error "Dependencies installation script not found"
        return 1
    fi

    log_info "Installing system dependencies (this may take 20-60 minutes)..."
    if ! "$SCRIPT_DIR/scripts/install_dependencies.sh" >> "$LOG_FILE" 2>&1; then
        log_error "Failed to install dependencies"
        return 1
    fi

    log_success "Dependencies installed successfully"
}

# Install NS-3
install_ns3() {
    if [[ "$SKIP_NS3" == "true" ]]; then
        log_step "Skipping NS-3 installation (as requested)"
        return 0
    fi

    log_step "Installing NS-3 3.45"

    if [ ! -f "$SCRIPT_DIR/scripts/install_ns3.sh" ]; then
        log_error "NS-3 installation script not found"
        return 1
    fi

    log_info "Installing NS-3 3.45 (this may take 10-30 minutes)..."
    if ! "$SCRIPT_DIR/scripts/install_ns3.sh" >> "$LOG_FILE" 2>&1; then
        log_error "Failed to install NS-3"
        return 1
    fi

    log_success "NS-3 3.45 installed successfully"
}

# Build NS-3 Gazebo plugin
build_gazebo_plugin() {
    log_step "Building NS-3 Gazebo plugin"

    cd "$SCRIPT_DIR/ns3_gazebo_plugin"

    log_info "Configuring CMake for Gazebo plugin..."
    if ! cmake . >> "$LOG_FILE" 2>&1; then
        log_error "Failed to configure Gazebo plugin"
        return 1
    fi

    log_info "Building Gazebo plugin..."
    local make_jobs=$(nproc)
    if ! make -j"$make_jobs" >> "$LOG_FILE" 2>&1; then
        log_error "Failed to build Gazebo plugin"
        return 1
    fi

    log_success "NS-3 Gazebo plugin built successfully"
}

# Build ROS2 workspace
build_ros2_workspace() {
    log_step "Building ROS2 workspace"

    cd "$SCRIPT_DIR/ns3_gazebo_ws"

    # Source ROS2 environment
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    else
        log_error "ROS2 Jazzy not found"
        return 1
    fi

    log_info "Building ROS2 workspace with colcon..."
    local build_type="Release"
    if [[ "$INSTALL_MODE" == "dev" ]]; then
        build_type="Debug"
    fi

    if ! colcon build --cmake-args -DCMAKE_BUILD_TYPE=$build_type >> "$LOG_FILE" 2>&1; then
        log_error "Failed to build ROS2 workspace"
        return 1
    fi

    log_success "ROS2 workspace built successfully"
}

# Build test components
build_test_components() {
    log_step "Building test components"

    # Build NS-3 WiFi test
    log_info "Building NS-3 WiFi test..."
    cd "$SCRIPT_DIR/ns3_wifi_tap_test"

    if [ ! -d "build" ]; then
        mkdir build
    fi

    cd build

    if ! cmake .. >> "$LOG_FILE" 2>&1; then
        log_error "Failed to configure NS-3 WiFi test"
        return 1
    fi

    if ! make >> "$LOG_FILE" 2>&1; then
        log_error "Failed to build NS-3 WiFi test"
        return 1
    fi

    # Build testbed nodes
    log_info "Building testbed nodes..."
    cd "$SCRIPT_DIR/ns3_testbed/ns3_testbed_nodes"

    # Source ROS2 environment
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    fi

    if ! colcon build >> "$LOG_FILE" 2>&1; then
        log_error "Failed to build testbed nodes"
        return 1
    fi

    log_success "Test components built successfully"
}

# Run post-installation setup
post_install_setup() {
    log_step "Running post-installation setup"

    # Create shortcuts and desktop files if in full mode
    if [[ "$INSTALL_MODE" == "full" ]]; then
        log_info "Creating desktop shortcuts..."
        # Could add desktop file creation here
    fi

    # Setup environment
    log_info "Setting up environment..."
    if [ -f "$SCRIPT_DIR/scripts/setup_environment.sh" ]; then
        source "$SCRIPT_DIR/scripts/setup_environment.sh"
    fi

    log_success "Post-installation setup complete"
}

# Verify installation
verify_installation() {
    log_step "Verifying installation"

    if [ -f "$SCRIPT_DIR/scripts/verify_installation.sh" ]; then
        log_info "Running installation verification..."
        if ! "$SCRIPT_DIR/scripts/verify_installation.sh" >> "$LOG_FILE" 2>&1; then
            log_warning "Installation verification found issues"
            log_info "Check the log file for details: $LOG_FILE"
        else
            log_success "Installation verification passed"
        fi
    else
        log_warning "Verification script not found, skipping verification"
    fi
}

# Show installation summary
show_summary() {
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    local minutes=$((duration / 60))
    local seconds=$((duration % 60))

    echo ""
    echo "==============================================="
    echo "         INSTALLATION COMPLETE!"
    echo "==============================================="
    echo ""
    log_success "NS3-Gazebo system installed successfully!"
    echo ""
    echo "Installation details:"
    echo "  📦 Mode: $INSTALL_MODE"
    echo "  ⏱️  Duration: ${minutes}m ${seconds}s"
    echo "  📋 Log file: $LOG_FILE"
    echo ""
    echo "Next steps:"
    echo "  1. Restart your terminal or run: source ~/.bashrc"
    echo "  2. Test the installation: ./scripts/verify_installation.sh"
    echo "  3. Read the documentation: README.md"
    echo "  4. Try the examples in the testbed directory"
    echo ""
    echo "Quick start:"
    echo "  # Setup network namespace"
    echo "  sudo python3 scripts/nns_setup.py setup -c 1"
    echo ""
    echo "  # Run complete testbed"
    echo "  cd ns3_testbed"
    echo "  python3 testbed_runner.py --no_nns -c 2 -s csv_setup/example1.csv -v"
    echo ""
    echo "For help and documentation:"
    echo "  📖 README.md - User guide and examples"
    echo "  📋 UPGRADE_PLAN.md - Technical details"
    echo "  🤝 CONTRIBUTING.md - How to contribute"
    echo ""
}

# Error handling
handle_error() {
    local exit_code=$?
    log_error "Installation failed at step $CURRENT_STEP"
    log_error "Check the log file for details: $LOG_FILE"
    echo ""
    echo "Common solutions:"
    echo "  - Check internet connectivity"
    echo "  - Ensure sufficient disk space (20GB+)"
    echo "  - Run: sudo apt update && sudo apt upgrade"
    echo "  - Try: ./install.sh --skip-deps (if dependencies are already installed)"
    echo ""
    exit $exit_code
}

# Main installation function
main() {
    # Setup error handling
    trap handle_error ERR

    # Initialize
    show_banner
    parse_arguments "$@"

    # Initialize log file
    echo "Installation started at $(date)" > "$LOG_FILE"
    echo "Command: $0 $*" >> "$LOG_FILE"
    echo "Mode: $INSTALL_MODE" >> "$LOG_FILE"
    echo "" >> "$LOG_FILE"

    # Configuration
    interactive_config

    echo ""
    log_info "Starting installation in $INSTALL_MODE mode..."
    log_info "Installation log: $LOG_FILE"
    echo ""

    # Installation steps
    check_requirements
    show_progress $CURRENT_STEP $TOTAL_STEPS

    install_dependencies
    show_progress $CURRENT_STEP $TOTAL_STEPS

    install_ns3
    show_progress $CURRENT_STEP $TOTAL_STEPS

    build_gazebo_plugin
    show_progress $CURRENT_STEP $TOTAL_STEPS

    build_ros2_workspace
    show_progress $CURRENT_STEP $TOTAL_STEPS

    build_test_components
    show_progress $CURRENT_STEP $TOTAL_STEPS

    post_install_setup
    show_progress $CURRENT_STEP $TOTAL_STEPS

    verify_installation
    show_progress $CURRENT_STEP $TOTAL_STEPS

    echo "" # New line after progress bar

    # Show summary
    show_summary

    log_success "Installation completed successfully!"
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi