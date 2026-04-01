# Dockerfile for ackermann_rover_humble
# Ubuntu 24.04 + ROS 2 (configurable) + Gazebo (gz) release (configurable)
# NOTE: Defaults target ROS 2 Jazzy on Ubuntu 24.04 (Noble).

ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ARG UBUNTU_VERSION

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Build-time configuration for ROS and Gazebo distributions
ARG ROS_DISTRO=jazzy
ARG GZ_DISTRO=harmonic
ARG ROS_UBUNTU_CODENAME=noble

# Locale setup
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
# Silence Python DeprecationWarnings (e.g., pkg_resources in rosdep)
ENV PYTHONWARNINGS=ignore::DeprecationWarning

# Base tools (from Ubuntu repos)
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 apt repository (default: Jazzy on Ubuntu Noble)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${ROS_UBUNTU_CODENAME} main" \
    > /etc/apt/sources.list.d/ros2.list

# Add Gazebo (gz) Harmonic apt repository (ubuntu-stable, per official docs)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# Install ROS 2, Gazebo (gz), ROS–gz bridge, and ROS dev tools
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    gz-${GZ_DISTRO} \
    ros-$ROS_DISTRO-ros-gz-sim \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-$ROS_DISTRO-ros2controlcli \
    ros-$ROS_DISTRO-rtabmap-* \
    ros-$ROS_DISTRO-turtlebot3* \
    ros-$ROS_DISTRO-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && \
    rosdep update

# Create and populate workspace
WORKDIR /workspace

# Import additional repos if ros2.repos is present (for dependencies/stack)
COPY ros2.repos ./

RUN mkdir -p src && \
    if [ -f ros2.repos ] && grep -q "repositories:" ros2.repos; then \
    vcs import src < ros2.repos; \
    else \
    echo "Skipping vcs import: ros2.repos is empty or has no repositories"; \
    fi

# NOTE: Project sources are NOT copied into the image.
# Bind-mount your workspace at runtime, e.g.:
#   docker run -it --rm \
#     -v "$PWD":/workspace/src/ackermann_rover_humble \
#     ${IMAGE_NAME:-ackermann_rover}_${ROS_DISTRO}

# PX4 SITL build dependencies (gz-harmonic dev libs already installed above)
# cmake, ninja-build, build-essential, git, python3-pip already installed above
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ccache \
    astyle \
    libssl-dev \
    libxml2-dev \
    libxml2-utils \
    ninja-build \
    libunwind-dev \
    cppzmq-dev \
    libeigen3-dev \
    libopencv-dev \
    protobuf-compiler \
    pkg-config \
    bc \
    gdb \
    dmidecode \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    libusb-1.0-0-dev \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# CUDA 12.6 toolkit (nvcc + cudart headers) for GPU-accelerated depth alignment.
# The NVIDIA runtime driver (libcuda.so) is provided at container start via
# nvidia-container-runtime; only the compiler toolkit is needed in the image.
# CUDA 11.4 (JetPack 5 host) is incompatible with Ubuntu 24.04 glibc headers,
# so we install CUDA 12.6 for compilation. The compiled kernels target sm_72
# (Xavier) and sm_87 (Orin) and run on the JetPack 5 driver via PTX JIT.
RUN curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/sbsa/cuda-keyring_1.1-1_all.deb \
    -o /tmp/cuda-keyring.deb && \
    dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
    cuda-nvcc-12-6 \
    cuda-cudart-dev-12-6 \
    && rm -rf /var/lib/apt/lists/*
ENV PATH=/usr/local/cuda-12.6/bin:${PATH}

# PX4 Python dependencies
COPY docker/px4_requirements.txt /tmp/px4_requirements.txt
RUN python3 -m pip install --break-system-packages -r /tmp/px4_requirements.txt \
    && rm /tmp/px4_requirements.txt

# Micro-XRCE-DDS-Agent (PX4 ↔ ROS 2 DDS bridge)
# Uses SUPERBUILD=ON to bundle its own FastRTPS/spdlog/fmt (avoids fmt v9
# incompatibility with Ubuntu 24.04 system packages). Installed to /opt/microxrce
# to isolate from the system ROS 2 FastRTPS and prevent ABI conflicts.
# Compatible with PX4's built-in uxrce_dds_client v2.4.x.
RUN cd /tmp && \
    git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/microxrce \
    -DUAGENT_SUPERBUILD=ON -DUAGENT_FAST_PROFILE=ON -DUAGENT_CED_PROFILE=OFF \
    -DUAGENT_USE_SYSTEM_LOGGER=OFF -DUAGENT_BUILD_TESTS=OFF && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# Intel RealSense SDK v2.51.1
# FORCE_RSUSB_BACKEND=ON uses the libusb user-space backend (no kernel UVC patch needed in Docker)
# Installed to /usr/local so find_package(realsense2) works for downstream ROS 2 packages
RUN cd /tmp && \
    git clone --depth 1 --branch v2.51.1 https://github.com/IntelRealSense/librealsense.git && \
    find /tmp/librealsense \( -name "*.h" -o -name "*.cpp" \) \
        -exec sed -i "s/\xe2\x80\x98/'/g; s/\xe2\x80\x99/'/g" {} + && \
    cd librealsense && mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_WITH_CUDA=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/librealsense

# (Optional) Install package dependencies for any repos brought in via ros2.repos.
# Your bind-mounted project can run rosdep / colcon inside the container.
RUN apt-get update && source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || \
    echo "WARNING: rosdep install encountered errors (some ros-jazzy-* packages may not be available); continuing image build."

# (Optional) Build any repos already present under /workspace/src at image build time.
RUN apt-get update && source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install || \
    echo "WARNING: colcon build failed during image build; please run colcon build inside the container after resolving dependencies."

# Additional tools
RUN apt-get update && apt-get install -y \
    libpaho-mqtt-dev \
    libpaho-mqttpp-dev \
    mosquitto \
    mosquitto-clients \
    python3-protobuf \
    tmux \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user matching the host user so bind-mounted files have
# consistent ownership and git does not complain about dubious ownership.
ARG USERNAME=nvidia
ARG USER_UID=1000
ARG USER_GID=1000
RUN userdel -r ubuntu 2>/dev/null || true && \
    groupadd --gid ${USER_GID} ${USERNAME} 2>/dev/null || true && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    usermod -aG dialout ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

ENV HOME=/home/${USERNAME}
USER ${USERNAME}

# Lightweight entrypoint script: provided from the local repo for easy editing
COPY --chown=${USERNAME}:${USERNAME} docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
