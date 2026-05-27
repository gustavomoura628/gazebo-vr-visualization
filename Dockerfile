ARG ROS_DISTRO=humble

# ---
# Base stage
# ---
FROM ros:$ROS_DISTRO-ros-base AS base

ARG DEBIAN_FRONTEND=noninteractive

# Change default shell to bash
# Needed to allow later build steps to source the workspace
SHELL [ "/bin/bash" , "-c" ]

ARG ROS_DISTRO
ENV ROS_DISTRO=$ROS_DISTRO

# Install necessary packages
RUN apt-get update \
    && apt-get install -y \
        build-essential \
        cmake \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        ros-$ROS_DISTRO-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

# Create a ROS 2 workspace
ENV ROS_WS=/ros2_ws
RUN mkdir -p $ROS_WS/src

#  Clone unitree ROS packages
WORKDIR /tmp
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git /tmp/unitree_ros2 \
    && mkdir -p $ROS_WS/src/unitree \
    && cp -r /tmp/unitree_ros2/cyclonedds_ws/src/* $ROS_WS/src/unitree/ \
    && rm -rf /tmp/unitree_ros2

# Copy local ROS packages
COPY packages/ $ROS_WS/src/akcit

# Install ROS dependencies
RUN apt-get update \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install \
        --default-yes \
        --ignore-packages-from-source \
        --from-paths $ROS_WS/src \
    && rm -rf /var/lib/apt/lists/*

# Build the ROS 2 workspace
WORKDIR $ROS_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace setup script
RUN echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

# ---
# Devcontainer stage
# ---
FROM base AS devcontainer

# Create new user
ARG USERNAME=akcit
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV HOME=/home/$USERNAME

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && sudo chsh -s /bin/bash $USERNAME \
    # Install sudo and add user to sudoers
    && apt-get update \
    && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/* \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# [Optional] Set the default user for the container
USER $USERNAME

# ---
# Default build stage
# ---
FROM base AS workspace

# Set the entrypoint to source the workspace setup script
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source $ROS_WS/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
