#!/bin/bash

# =============================================================================
# Dependencies Installation Script for ns3_gazebo
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
LOG_FILE="$PROJECT_DIR/install_dependencies.log"

# Progress tracking
TOTAL_STEPS=6
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

show_progress() {
    local current=$1
    local total=$2
    local width=50
    local percentage=$((current * 100 / total))
    local filled=$((current * width / total))

    printf "\rProgress: ["
    printf "%*s" $filled | tr ' ' '='
    printf "%*s" $((width - filled)) | tr ' ' '-'
    printf "] %d%%" $percentage
}

# Check if script is run with sudo when needed
check_sudo() {
    if [[ $EUID -eq 0 ]]; then
        log_warning "Running as root. This is not recommended."
        log_warning "Consider running as regular user (script will request sudo when needed)"
    fi
}

# Update system packages
update_system() {
    log_step "Updating system packages"

    log_info "Updating package lists..."
    if ! sudo apt update >> "$LOG_FILE" 2>&1; then
        log_error "Failed to update package lists"
        return 1
    fi

    log_info "Upgrading existing packages..."
    if ! sudo apt upgrade -y >> "$LOG_FILE" 2>&1; then
        log_warning "Some packages failed to upgrade (continuing anyway)"
    fi

    log_success "System packages updated"
}

# Install basic build tools
install_build_tools() {
    log_step "Installing build tools and dependencies"

    local packages=(
        "build-essential"
        "cmake"
        "git"
        "curl"
        "wget"
        "python3"
        "python3-dev"
        "python3-pip"
        "pkg-config"
        "software-properties-common"
        "apt-transport-https"
        "ca-certificates"
        "gnupg"
        "lsb-release"
        "bc"
    )

    log_info "Installing basic build tools: ${packages[*]}"

    if ! sudo apt install -y "${packages[@]}" >> "$LOG_FILE" 2>&1; then
        log_error "Failed to install basic build tools"
        return 1
    fi

    # Install Python packages
    log_info "Installing Python packages..."
    local python_packages=(
        "setuptools"
        "wheel"
        "colcon-common-extensions"
        "vcstool"
    )

    for package in "${python_packages[@]}"; do
        if ! pip3 install --user "$package" >> "$LOG_FILE" 2>&1; then
            log_warning "Failed to install Python package: $package"
        fi
    done

    log_success "Build tools installed"
}

# Install ROS2 Jazzy
install_ros2() {
    log_step "Installing ROS2 Jazzy"

    # Check if already installed
    if [ -d "/opt/ros/jazzy" ]; then
        log_success "ROS2 Jazzy already installed"
        return 0
    fi

    log_info "Adding ROS2 repository..."

    # Add ROS2 GPG key
    if ! curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add - >> "$LOG_FILE" 2>&1; then
        log_error "Failed to add ROS2 GPG key"
        return 1
    fi

    # Add ROS2 repository
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list >> "$LOG_FILE"

    # Update package list
    sudo apt update >> "$LOG_FILE" 2>&1

    log_info "Installing ROS2 Jazzy (this may take several minutes)..."
    if ! sudo apt install -y ros-jazzy-desktop >> "$LOG_FILE" 2>&1; then
        log_error "Failed to install ROS2 Jazzy desktop"
        return 1
    fi

    # Install additional ROS2 packages
    log_info "Installing additional ROS2 packages..."
    local ros_packages=(
        "ros-jazzy-gazebo-ros-pkgs"
        "ros-jazzy-robot-state-publisher"
        "ros-jazzy-joint-state-publisher"
        "ros-jazzy-joint-state-publisher-gui"
        "ros-jazzy-xacro"
        "ros-jazzy-nav2-bringup"
        "ros-jazzy-navigation2"
        "ros-jazzy-nav2-msgs"
        "python3-rosdep"
        "python3-rosinstall"
        "python3-rosinstall-generator"
        "python3-wstool"
    )

    for package in "${ros_packages[@]}"; do
        if ! sudo apt install -y "$package" >> "$LOG_FILE" 2>&1; then
            log_warning "Failed to install ROS2 package: $package"
        fi
    done

    # Initialize rosdep
    log_info "Initializing rosdep..."
    if ! sudo rosdep init >> "$LOG_FILE" 2>&1; then
        log_warning "rosdep init failed (may already be initialized)"
    fi

    if ! rosdep update >> "$LOG_FILE" 2>&1; then
        log_warning "rosdep update failed"
    fi

    log_success "ROS2 Jazzy installed"
}

# Install Gazebo Harmonic
install_gazebo() {
    log_step "Installing Gazebo Harmonic"

    # Check if already installed
    if command -v gz &> /dev/null; then
        local gz_version=$(gz sim --versions 2>/dev/null | head -1 || echo "unknown")
        log_success "Gazebo already installed: $gz_version"
        return 0
    fi

    log_info "Adding Gazebo repository..."

    # Add Gazebo GPG key
    if ! curl https://packages.osrfoundation.org/gazebo.gpg | sudo apt-key add - >> "$LOG_FILE" 2>&1; then
        log_error "Failed to add Gazebo GPG key"
        return 1
    fi

    # Add Gazebo repository
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >> "$LOG_FILE"

    # Update package list
    sudo apt update >> "$LOG_FILE" 2>&1

    log_info "Installing Gazebo Harmonic (this may take several minutes)..."
    if ! sudo apt install -y gz-harmonic >> "$LOG_FILE" 2>&1; then
        log_error "Failed to install Gazebo Harmonic"
        return 1
    fi

    log_success "Gazebo Harmonic installed"
}

# Install network tools (for Docker environments)
install_network_tools() {
    log_step "Installing network tools"

    local network_packages=(
        "iproute2"
        "net-tools"
        "iputils-ping"
        "bridge-utils"
        "iptables"
        "netcat-openbsd"
    )

    log_info "Installing network tools: ${network_packages[*]}"

    if ! sudo apt install -y "${network_packages[@]}" >> "$LOG_FILE" 2>&1; then
        log_error "Failed to install network tools"
        return 1
    fi

    log_success "Network tools installed"
}

# Install NS-3 specific dependencies
install_ns3_dependencies() {
    log_step "Installing NS-3 dependencies"

    local ns3_packages=(
        "sqlite3"
        "libsqlite3-dev"
        "libxml2-dev"
        "libgtk-3-dev"
        "qtbase5-dev"
        "qtchooser"
        "qt5-qmake"
        "qtbase5-dev-tools"
        "gir1.2-goocanvas-2.0"
        "python3-gi"
        "python3-gi-cairo"
        "python3-pygraphviz"
        "gir1.2-gtk-3.0"
        "ipython3"
        "openmpi-bin"
        "openmpi-common"
        "openmpi-doc"
        "libopenmpi-dev"
        "mercurial"
        "unzip"
        "gdb"
        "valgrind"
        "graphviz"
        "libgraphviz-dev"
        "python3-pygraphviz"
        "python3-pydot"
    )

    log_info "Installing NS-3 dependencies: ${ns3_packages[*]}"

    if ! sudo apt install -y "${ns3_packages[@]}" >> "$LOG_FILE" 2>&1; then
        log_error "Failed to install NS-3 dependencies"
        return 1
    fi

    log_success "NS-3 dependencies installed"
}

# Setup environment
setup_environment() {
    log_info "Setting up environment..."

    # Create environment setup script
    local env_script="$PROJECT_DIR/scripts/setup_environment.sh"

    cat > "$env_script" << 'EOF'
#!/bin/bash
# NS3-Gazebo Environment Setup

# ROS2 setup
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "ROS2 Jazzy environment loaded"
fi

# Gazebo setup
if command -v gz &> /dev/null; then
    export GZ_VERSION=harmonic
    echo "Gazebo Harmonic environment ready"
fi

# NS-3 setup (if available)
if [ -f "$HOME/.local/share/ns3_gazebo/scripts/ns3_env.sh" ]; then
    source "$HOME/.local/share/ns3_gazebo/scripts/ns3_env.sh"
fi

# Add local bin to PATH for pip installed packages
export PATH="$HOME/.local/bin:$PATH"

echo "NS3-Gazebo environment setup complete"
EOF

    chmod +x "$env_script"

    # Add to bashrc if not already present
    local bashrc_line="source $env_script"
    if ! grep -Fxq "$bashrc_line" "$HOME/.bashrc"; then
        echo "" >> "$HOME/.bashrc"
        echo "# NS3-Gazebo environment" >> "$HOME/.bashrc"
        echo "$bashrc_line" >> "$HOME/.bashrc"
        log_info "Added environment setup to ~/.bashrc"
    fi

    log_success "Environment setup complete"
}

# Clean up temporary files
cleanup() {
    log_info "Cleaning up temporary files..."
    sudo apt autoremove -y >> "$LOG_FILE" 2>&1
    sudo apt autoclean >> "$LOG_FILE" 2>&1
    log_success "Cleanup complete"
}

# Main installation function
main() {
    echo "==============================================="
    echo "    NS3-Gazebo Dependencies Installation"
    echo "==============================================="
    echo ""

    # Initialize log file
    echo "Installation started at $(date)" > "$LOG_FILE"

    log_info "Installation log: $LOG_FILE"
    log_info "This process may take 20-60 minutes depending on your system"
    echo ""

    # Check sudo access
    check_sudo

    # Run installation steps
    update_system || { log_error "Failed to update system"; exit 1; }
    show_progress $CURRENT_STEP $TOTAL_STEPS

    install_build_tools || { log_error "Failed to install build tools"; exit 1; }
    show_progress $CURRENT_STEP $TOTAL_STEPS

    install_ros2 || { log_error "Failed to install ROS2"; exit 1; }
    show_progress $CURRENT_STEP $TOTAL_STEPS

    install_gazebo || { log_error "Failed to install Gazebo"; exit 1; }
    show_progress $CURRENT_STEP $TOTAL_STEPS

    install_network_tools || { log_error "Failed to install network tools"; exit 1; }
    show_progress $CURRENT_STEP $TOTAL_STEPS

    install_ns3_dependencies || { log_error "Failed to install NS-3 dependencies"; exit 1; }
    show_progress $CURRENT_STEP $TOTAL_STEPS

    echo "" # New line after progress bar

    setup_environment
    cleanup

    echo ""
    echo "==============================================="
    echo "         DEPENDENCIES INSTALLATION COMPLETE"
    echo "==============================================="
    echo ""
    log_success "All dependencies installed successfully!"
    log_info "Next steps:"
    log_info "1. Restart your terminal or run: source ~/.bashrc"
    log_info "2. Install NS-3: ./scripts/install_ns3.sh"
    log_info "3. Build the project: ./install.sh --build-only"
    echo ""
    log_info "Installation log saved to: $LOG_FILE"
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi