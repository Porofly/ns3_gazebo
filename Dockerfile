# NS3-Gazebo Integration Docker Image
# Based on Ubuntu 24.04 with ROS2 Jazzy, Gazebo Harmonic, and NS-3 3.45

FROM ubuntu:24.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV GZ_VERSION=harmonic
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install basic packages
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Add Gazebo repository
RUN curl https://packages.osrfoundation.org/gazebo.gpg | apt-key add - \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Update package lists
RUN apt-get update

# Install ROS2 Jazzy
RUN apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-nav2-bringup \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-msgs \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool

# Install Gazebo Harmonic
RUN apt-get install -y gz-harmonic

# Install build tools and dependencies
RUN apt-get install -y \
    build-essential \
    cmake \
    git \
    python3 \
    python3-dev \
    python3-pip \
    pkg-config \
    bc \
    sqlite3 \
    libsqlite3-dev \
    libxml2-dev \
    libgtk-3-dev \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    gir1.2-goocanvas-2.0 \
    python3-gi \
    python3-gi-cairo \
    python3-pygraphviz \
    gir1.2-gtk-3.0 \
    ipython3 \
    openmpi-bin \
    openmpi-common \
    openmpi-doc \
    libopenmpi-dev \
    mercurial \
    unzip \
    gdb \
    valgrind \
    graphviz \
    libgraphviz-dev \
    python3-pydot

# Install network tools
RUN apt-get install -y \
    iproute2 \
    net-tools \
    iputils-ping \
    bridge-utils \
    iptables \
    netcat-openbsd \
    sudo

# Install Python packages
RUN pip3 install --upgrade pip \
    && pip3 install \
    setuptools \
    wheel \
    colcon-common-extensions \
    vcstool

# Initialize rosdep
RUN rosdep init || true \
    && rosdep update || true

# Create workspace
WORKDIR /workspace/ns3_gazebo

# Copy project files
COPY . .

# Create a user for development
RUN useradd -m -s /bin/bash developer \
    && echo "developer:developer" | chpasswd \
    && usermod -aG sudo developer \
    && echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set ownership
RUN chown -R developer:developer /workspace

# Switch to developer user
USER developer

# Source ROS2 environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/developer/.bashrc

# Setup Gazebo environment
RUN echo "export GZ_VERSION=harmonic" >> /home/developer/.bashrc

# Add workspace setup to bashrc
RUN echo "export PATH=\"/home/developer/.local/bin:\$PATH\"" >> /home/developer/.bashrc

# Install NS-3 using our script
RUN cd /workspace/ns3_gazebo && bash scripts/install_ns3.sh

# Build the project components
RUN cd /workspace/ns3_gazebo && \
    source /opt/ros/jazzy/setup.bash && \
    # Build Gazebo plugin
    cd ns3_gazebo_plugin && \
    cmake . && \
    make && \
    # Build ROS2 workspace
    cd ../ns3_gazebo_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Build test components
    cd ../ns3_wifi_tap_test && \
    mkdir -p build && cd build && \
    cmake .. && make && \
    cd ../../ns3_testbed/ns3_testbed_nodes && \
    colcon build

# Setup workspace environment
RUN echo "source /workspace/ns3_gazebo/ns3_gazebo_ws/install/setup.bash" >> /home/developer/.bashrc \
    && echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/ns3_gazebo/ns3_gazebo_plugin/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> /home/developer/.bashrc

# Expose common ports
EXPOSE 11311 11345 8080

# Set working directory
WORKDIR /workspace/ns3_gazebo

# Default command
CMD ["/bin/bash"]