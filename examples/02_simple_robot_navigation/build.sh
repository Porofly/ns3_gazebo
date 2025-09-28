#!/bin/bash

# =============================================================================
# Build Script for Simple Robot Navigation Example
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
WS_DIR="$SCRIPT_DIR/../../ns3_gazebo_ws"

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
echo "  Simple Robot Navigation Example - Build"
echo "==============================================="
echo ""

# Check dependencies
log_info "Checking dependencies..."

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    log_error "ROS2 not found"
    log_info "Please install ROS2 Jazzy and source the setup script"
    exit 1
fi

# Check if Nav2 is available
if ! ros2 pkg list | grep -q nav2_bringup; then
    log_error "Nav2 not found"
    log_info "Please install ROS2 navigation stack: sudo apt install ros-jazzy-nav2-bringup"
    exit 1
fi

# Check if Gazebo is available
if ! command -v gz &> /dev/null; then
    log_error "Gazebo not found"
    log_info "Please install Gazebo Harmonic"
    exit 1
fi

log_success "All dependencies found"

# Source ROS2 environment
log_info "Sourcing ROS2 environment..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    log_success "ROS2 Jazzy environment loaded"
else
    log_error "ROS2 setup script not found"
    exit 1
fi

# Create maps directory and simple map
log_info "Creating navigation maps..."
mkdir -p "$SCRIPT_DIR/maps"

# Create a simple occupancy grid map
cat > "$SCRIPT_DIR/maps/simple_map.yaml" << EOF
image: simple_map.pgm
resolution: 0.05
origin: [-5.0, -5.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF

# Create simple PGM map file (10x10 meters at 0.05 resolution = 200x200 pixels)
python3 << EOF
import numpy as np
from PIL import Image

# Create a 200x200 map (10x10 meters at 0.05m resolution)
map_size = 200
map_data = np.ones((map_size, map_size), dtype=np.uint8) * 255  # Free space (white)

# Add walls (black pixels for occupied space)
# Outer walls
map_data[0:5, :] = 0      # North wall
map_data[-5:, :] = 0     # South wall
map_data[:, 0:5] = 0     # West wall
map_data[:, -5:] = 0     # East wall

# Add some obstacles
# Obstacle 1 (corresponds to gazebo obstacle at 2,2)
x1, y1 = int((2 + 5) / 0.05), int((1 + 5) / 0.05)  # Convert to map coordinates
map_data[y1-10:y1+10, x1-10:x1+10] = 0

# Obstacle 2 (corresponds to gazebo obstacle at -2,-2)
x2, y2 = int((-2 + 5) / 0.05), int((-2 + 5) / 0.05)
for i in range(-10, 11):
    for j in range(-10, 11):
        if i*i + j*j <= 100:  # Circular obstacle
            if 0 <= y2+i < map_size and 0 <= x2+j < map_size:
                map_data[y2+i, x2+j] = 0

# Obstacle 3 (corresponds to gazebo obstacle at -1,3)
x3, y3 = int((-1 + 5) / 0.05), int((3 + 5) / 0.05)
map_data[y3-5:y3+5, x3-20:x3+20] = 0

# Save as PGM
img = Image.fromarray(map_data, mode='L')
img.save('$SCRIPT_DIR/maps/simple_map.pgm')

print("Map created successfully")
EOF

log_success "Navigation map created"

# Create RViz configuration
log_info "Creating RViz configuration..."
mkdir -p "$SCRIPT_DIR/config"

cat > "$SCRIPT_DIR/config/robot_nav.rviz" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /Map1
        - /LaserScan1
        - /Path1
      Splitter Ratio: 0.5
    Tree Height: 557
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic: /map
      Unreliable: false
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic: /scan
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic: /plan
      Unreliable: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 1.5697963237762451
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0
    Saved: ~
EOF

log_success "RViz configuration created"

# Build workspace if it exists
if [ -d "$WS_DIR" ]; then
    log_info "Building ROS2 workspace..."
    cd "$WS_DIR"

    if ! colcon build --packages-select diff_drive_ns3 2>&1; then
        log_warning "ROS2 workspace build failed, but continuing with example setup"
    else
        log_success "ROS2 workspace built successfully"
    fi
else
    log_warning "ROS2 workspace not found, skipping workspace build"
fi

# Return to example directory
cd "$SCRIPT_DIR"

# Check Python dependencies
log_info "Checking Python dependencies..."
python3 -c "import rclpy, geometry_msgs, nav_msgs, sensor_msgs, std_msgs, nav2_msgs" 2>/dev/null || {
    log_warning "Some Python dependencies may be missing"
    log_info "Install with: pip3 install -r requirements.txt (if available)"
}

log_success "Python dependencies check completed"

# Validate configuration files
log_info "Validating configuration files..."

# Check URDF
if [ -f "$SCRIPT_DIR/urdf/diff_drive_robot.urdf" ]; then
    log_success "Robot URDF found"
else
    log_error "Robot URDF not found"
    exit 1
fi

# Check world file
if [ -f "$SCRIPT_DIR/robot_world.world" ]; then
    log_success "World file found"
else
    log_error "World file not found"
    exit 1
fi

# Check configuration files
config_files=("robot_config.yaml" "nav_params.yaml" "wifi_config.yaml")
for config in "${config_files[@]}"; do
    if [ -f "$SCRIPT_DIR/config/$config" ]; then
        log_success "Configuration file found: $config"
    else
        log_error "Configuration file not found: $config"
        exit 1
    fi
done

echo ""
echo "==============================================="
echo "         BUILD COMPLETED SUCCESSFULLY!"
echo "==============================================="
echo ""
echo "Next steps:"
echo "  1. Run the example: ./run.sh"
echo "  2. Or launch manually:"
echo "     ros2 launch robot_navigation.launch.py"
echo ""
echo "Files created/verified:"
echo "  - Navigation map: maps/simple_map.{yaml,pgm}"
echo "  - RViz config: config/robot_nav.rviz"
echo "  - Robot URDF: urdf/diff_drive_robot.urdf"
echo "  - Configuration files in config/"
echo ""