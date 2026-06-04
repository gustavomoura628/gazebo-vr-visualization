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

# Quest teleop bridge (web_teleop/teleop_bridge.py). Added here, after the
# expensive colcon build in `base`, so editing it doesn't bust the build cache.
# WebRTC stack: aiortc (UDP video track + control datachannel) + aiohttp
# (signaling/static/HTTPS). av provides the video encoder; numpy for frames.
RUN pip3 install --no-cache-dir aiortc aiohttp numpy

# Bump the OAK-D sim camera from 320x240 to 640x480 for noticeably better image
# quality (WebRTC handles the extra bitrate fine).
RUN sed -i -E \
    -e 's#<width>320</width>#<width>640</width>#' \
    -e 's#<height>240</height>#<height>480</height>#' \
    /opt/ros/humble/share/turtlebot4_description/urdf/sensors/oakd.urdf.xacro

# Force the sim SERVER's sensor rendering to OGRE2. The default fell back to
# OGRE1 (couldn't get a GL3.3 context), and OGRE1 silently breaks GPU lidar
# (all gpu_lidar sensors return 0). OGRE2 needs a real GL3.3+ context, which we
# get by rendering on the NVIDIA GPU (run with --gpus all; see run_simulation.sh).
# Patch the hardcoded ign_args in ignition.launch.py to add the engine flag.
RUN sed -i "s/' -r',/' -r --render-engine-server ogre2',/" \
    /opt/ros/humble/share/turtlebot4_ignition_bringup/launch/ignition.launch.py

# Make the rplidar a 3D lidar approximating a Livox MID-360: 16 vertical rings
# over the MID-360's asymmetric vertical FOV (-7 deg .. +52 deg = -0.122 .. +0.908
# rad), and ~40 m range. gpu_lidar with vertical>1 auto-publishes a 3D PointCloud2
# on .../rplidar/scan/points (bridged to /lidar/points in our launch).
RUN sed -i 's# h_samples="640"# h_samples="640" v_samples="16" v_min_angle="-0.122" v_max_angle="0.908"#' \
    /opt/ros/humble/share/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro \
 && sed -i 's#r_max="12.0"#r_max="40.0"#' \
    /opt/ros/humble/share/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro

# Second lidar: replicate the real robot's two-MID-360 setup, mounted at the
# tower TOP SENSOR PLATE (~25 cm up) so they see over the chassis instead of
# being buried inside it. Top lidar sits just above the plate (normal + forward
# tilt); bottom lidar just below it (upside-down roll pi + forward tilt). The two
# forward-tilt angles are xacro properties to measure on the real robot later.
# (TurtleBot4 has no "mouth"/"neck", so positions are approximate.)
RUN python3 - <<'PY'
import pathlib
f = pathlib.Path("/opt/ros/humble/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro")
s = f.read_text()
# tilt parameters (rad) — MUST MATCH TOP/BOTTOM_LIDAR_PITCH in teleop_bridge.py
s = s.replace('value="${9.8715*cm2m}"/>',
    'value="${9.8715*cm2m}"/>\n'
    '  <xacro:property name="top_lidar_pitch"    value="0.30"/>\n'
    '  <xacro:property name="bottom_lidar_pitch" value="0.30"/>', 1)
old = ('  <xacro:rplidar name="rplidar" parent_link="shell_link" gazebo="$(arg gazebo)">\n'
       '    <origin xyz="${rplidar_x_offset} ${rplidar_y_offset} ${rplidar_z_offset}"\n'
       '            rpy="0 0 ${pi/2}"/>\n'
       '  </xacro:rplidar>')
new = ('  <xacro:rplidar name="rplidar" parent_link="shell_link" gazebo="$(arg gazebo)">\n'
       '    <origin xyz="${rplidar_x_offset} ${rplidar_y_offset} ${tower_sensor_plate_z_offset + 0.03}"\n'
       '            rpy="0 ${top_lidar_pitch} ${pi/2}"/>\n'
       '  </xacro:rplidar>\n'
       '  <xacro:rplidar name="lidar_bottom" parent_link="shell_link" gazebo="$(arg gazebo)">\n'
       '    <origin xyz="${rplidar_x_offset} ${rplidar_y_offset} ${tower_sensor_plate_z_offset - 0.03}"\n'
       '            rpy="${pi} ${bottom_lidar_pitch} ${pi/2}"/>\n'
       '  </xacro:rplidar>')
assert old in s, "rplidar instantiation block not found"
s = s.replace(old, new, 1)
assert s.count('xacro:rplidar name=') == 2, "expected exactly 2 lidars"
f.write_text(s)
print("two lidars injected")
PY

COPY web_teleop/ /web_teleop/

# NOTE on speed: the TurtleBot4 is a Create3 base, hard-limited to ~0.46 m/s by
# its motion_control safety layer. Raising the diff-drive controller's body-twist
# limits does nothing (the cap is upstream). The real lever is the Create3
# "safety_override" parameter — set to "full" at runtime by run_teleop_bridge.sh,
# which unlocks the base's true max (~0.46 m/s) and removes the safe-speed throttle.
# (Combined max-forward + max-turn is still wheel-saturation limited; that's
# inherent to the Create3 base and won't affect the real Go2W.)

# Set the entrypoint to source the workspace setup script
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source $ROS_WS/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
