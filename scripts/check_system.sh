#!/bin/bash

# =============================================================================
# System Requirements Checker for ns3_gazebo
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Global variables
ERRORS=0
WARNINGS=0
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Minimum requirements
MIN_MEMORY_GB=4
MIN_DISK_GB=20
MIN_UBUNTU_VERSION="20.04"

# Functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
    ((WARNINGS++))
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
    ((ERRORS++))
}

log_header() {
    echo -e "${PURPLE}[CHECK]${NC} $1"
}

# Check if running on supported OS
check_operating_system() {
    log_header "Operating System"

    if [[ "$OSTYPE" != "linux-gnu"* ]]; then
        log_error "This system only supports Linux"
        return 1
    fi

    # Check for Ubuntu
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        if [[ "$ID" == "ubuntu" ]]; then
            log_success "Ubuntu detected: $VERSION"

            # Check Ubuntu version
            VERSION_NUM=$(echo $VERSION_ID | tr -d '"')
            if [[ $(echo "$VERSION_NUM >= $MIN_UBUNTU_VERSION" | bc -l) -eq 1 ]]; then
                log_success "Ubuntu version $VERSION_NUM is supported"
            else
                log_warning "Ubuntu version $VERSION_NUM may not be fully supported (minimum: $MIN_UBUNTU_VERSION)"
            fi
        else
            log_warning "Non-Ubuntu distribution detected: $ID $VERSION_ID"
            log_warning "This may work but is not officially supported"
        fi
    else
        log_error "Cannot determine Linux distribution"
        return 1
    fi

    # Check architecture
    ARCH=$(uname -m)
    if [[ "$ARCH" == "x86_64" ]]; then
        log_success "Architecture: $ARCH (supported)"
    elif [[ "$ARCH" == "aarch64" ]]; then
        log_warning "Architecture: $ARCH (experimental support)"
    else
        log_error "Architecture: $ARCH (not supported)"
        return 1
    fi
}

# Check system resources
check_system_resources() {
    log_header "System Resources"

    # Check memory
    TOTAL_MEM_KB=$(grep MemTotal /proc/meminfo | awk '{print $2}')
    TOTAL_MEM_GB=$((TOTAL_MEM_KB / 1024 / 1024))

    if [[ $TOTAL_MEM_GB -ge $MIN_MEMORY_GB ]]; then
        log_success "Memory: ${TOTAL_MEM_GB}GB (minimum: ${MIN_MEMORY_GB}GB)"
    else
        log_error "Insufficient memory: ${TOTAL_MEM_GB}GB (minimum: ${MIN_MEMORY_GB}GB)"
    fi

    # Check disk space
    AVAILABLE_DISK_GB=$(df -BG . | tail -1 | awk '{print $4}' | sed 's/G//')

    if [[ $AVAILABLE_DISK_GB -ge $MIN_DISK_GB ]]; then
        log_success "Disk space: ${AVAILABLE_DISK_GB}GB available (minimum: ${MIN_DISK_GB}GB)"
    else
        log_error "Insufficient disk space: ${AVAILABLE_DISK_GB}GB (minimum: ${MIN_DISK_GB}GB)"
    fi

    # Check CPU cores
    CPU_CORES=$(nproc)
    if [[ $CPU_CORES -ge 2 ]]; then
        log_success "CPU cores: $CPU_CORES (recommended: 2+)"
    else
        log_warning "CPU cores: $CPU_CORES (recommended: 2+ for better performance)"
    fi
}

# Check essential tools
check_essential_tools() {
    log_header "Essential Tools"

    local tools=("curl" "wget" "git" "python3" "cmake" "make" "gcc" "g++")
    local missing_tools=()

    for tool in "${tools[@]}"; do
        if command -v "$tool" &> /dev/null; then
            VERSION_INFO=$(timeout 5s $tool --version 2>/dev/null | head -1 || echo "version unknown")
            log_success "$tool: available ($VERSION_INFO)"
        else
            log_warning "$tool: not found"
            missing_tools+=("$tool")
        fi
    done

    if [[ ${#missing_tools[@]} -gt 0 ]]; then
        log_info "Missing tools will be installed: ${missing_tools[*]}"
    fi
}

# Check Python environment
check_python_environment() {
    log_header "Python Environment"

    # Check Python 3
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
        log_success "Python 3: $PYTHON_VERSION"

        # Check pip
        if command -v pip3 &> /dev/null; then
            PIP_VERSION=$(pip3 --version | cut -d' ' -f2)
            log_success "pip3: $PIP_VERSION"
        else
            log_warning "pip3: not found (will be installed)"
        fi
    else
        log_error "Python 3: not found"
    fi
}

# Check network connectivity
check_network() {
    log_header "Network Connectivity"

    # Check internet connectivity
    if ping -c 1 google.com &> /dev/null; then
        log_success "Internet connectivity: available"
    else
        log_error "Internet connectivity: not available"
        log_error "Internet access is required for downloading dependencies"
        return 1
    fi

    # Check specific repositories
    local repos=(
        "github.com"
        "packages.ros.org"
        "packages.osrfoundation.org"
        "archive.ubuntu.com"
        "www.nsnam.org"
    )

    for repo in "${repos[@]}"; do
        if ping -c 1 "$repo" &> /dev/null; then
            log_success "Repository access: $repo"
        else
            log_warning "Repository access: $repo (may cause issues)"
        fi
    done
}

# Check existing installations
check_existing_installations() {
    log_header "Existing Installations"

    # Check ROS2
    if [ -d "/opt/ros/jazzy" ]; then
        log_success "ROS2 Jazzy: installed"
    elif [ -d "/opt/ros" ]; then
        ROS_DISTROS=$(ls /opt/ros/)
        log_warning "ROS2: found other distributions ($ROS_DISTROS), Jazzy preferred"
    else
        log_info "ROS2: not installed (will be installed)"
    fi

    # Check Gazebo
    if command -v gz &> /dev/null; then
        GZ_VERSION=$(gz sim --versions 2>/dev/null | head -1 || echo "unknown")
        log_success "Gazebo: $GZ_VERSION"
    else
        log_info "Gazebo: not installed (will be installed)"
    fi

    # Check Docker
    if command -v docker &> /dev/null; then
        DOCKER_VERSION=$(docker --version | cut -d' ' -f3 | tr -d ',')
        log_success "Docker: $DOCKER_VERSION"

        # Check if user is in docker group
        if groups | grep -q docker; then
            log_success "Docker permissions: user in docker group"
        else
            log_warning "Docker permissions: user not in docker group (may need sudo)"
        fi
    else
        log_info "Docker: not installed (optional)"
    fi
}

# Check permissions
check_permissions() {
    log_header "Permissions"

    # Check sudo access
    if timeout 1s sudo -n true 2>/dev/null; then
        log_success "Sudo access: available (cached)"
    else
        log_info "Sudo access: will be requested during installation"
    fi

    # Check write permissions in current directory
    if [ -w "." ]; then
        log_success "Write permissions: current directory"
    else
        log_error "Write permissions: cannot write to current directory"
    fi

    # Check /opt permissions (for potential ROS2 installation)
    if [ -w "/opt" ] || sudo -n test -w "/opt" 2>/dev/null; then
        log_success "System install permissions: available"
    else
        log_info "System install permissions: will require sudo"
    fi
}

# Generate system report
generate_report() {
    echo ""
    echo "==============================================="
    echo "           SYSTEM CHECK SUMMARY"
    echo "==============================================="

    if [[ $ERRORS -eq 0 && $WARNINGS -eq 0 ]]; then
        log_success "System is ready for ns3_gazebo installation!"
    elif [[ $ERRORS -eq 0 ]]; then
        log_warning "System is compatible with $WARNINGS warning(s)"
        log_info "Installation should proceed normally"
    else
        log_error "System has $ERRORS error(s) and $WARNINGS warning(s)"
        log_error "Please resolve errors before proceeding"
        echo ""
        echo "Common solutions:"
        echo "- Update your system: sudo apt update && sudo apt upgrade"
        echo "- Free up disk space if needed"
        echo "- Ensure internet connectivity"
        echo "- Install missing tools: sudo apt install curl wget git build-essential"
        return 1
    fi

    echo ""
    echo "Next steps:"
    echo "1. Run: ./install.sh"
    echo "2. Or install dependencies first: ./scripts/install_dependencies.sh"
    echo ""

    return $ERRORS
}

# Main function
main() {
    echo "==============================================="
    echo "    NS3-Gazebo System Requirements Check"
    echo "==============================================="
    echo ""

    check_operating_system
    check_system_resources
    check_essential_tools
    check_python_environment
    check_network
    check_existing_installations
    check_permissions

    generate_report
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi