# Dockerfile for ackermann_rover_humble
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo (gz) Harmonic
# NOTE: ROS 2 Jazzy is officially targeted at Ubuntu 24.04 (Noble).
# If you strictly require Ubuntu 22.04, we would need to build Jazzy from source,
# which is significantly heavier and non-standard.

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

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

# Add ROS 2 Jazzy apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list

# Add Gazebo (gz) Harmonic apt repository (ubuntu-stable, per official docs)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO

# Install ROS 2 Jazzy, Gazebo Harmonic, ROS–gz bridge, and ROS dev tools
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    gz-harmonic \
    ros-$ROS_DISTRO-ros-gz-sim \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-$ROS_DISTRO-ros2controlcli \
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
#     ackermann_rover_jazzy

# (Optional) Install package dependencies for any repos brought in via ros2.repos.
# Your bind-mounted project can run rosdep / colcon inside the container.
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || \
    echo "WARNING: rosdep install encountered errors (some ros-jazzy-* packages may not be available); continuing image build."

# (Optional) Build any repos already present under /workspace/src at image build time.
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install || \
    echo "WARNING: colcon build failed during image build; please run colcon build inside the container after resolving dependencies."

# Source ROS and the overlay by default in interactive shells
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace

CMD ["bash"]
