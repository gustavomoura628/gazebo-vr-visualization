#!/usr/bin/env python3
"""Replace the TurtleBot4's stock single 2D rplidar with TWO 3D MID-360-style lidars
(top: normal + forward tilt; bottom: upside-down + forward tilt), mounted at the
tower sensor plate. Run as a Docker build step (see Dockerfile).

IMPORTANT: this is a COPY'd script run with `RUN python3 inject_two_lidars.py`, NOT a
`RUN python3 - <<HEREDOC`. Heredocs in RUN are only honored by BuildKit; on a
non-BuildKit (or differently-configured) Docker the heredoc is silently dropped,
python reads empty stdin, does nothing, and the build SUCCEEDS with the stock single
lidar -- a wrong-but-working image. That happened on a fresh machine and produced a
tilted cloud + a "missing" second lidar. A plain script file is build-reproducible.
"""
import pathlib

f = pathlib.Path("/opt/ros/humble/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro")
s = f.read_text()

# tilt parameters (rad) -- MUST MATCH TOP/BOTTOM_LIDAR_PITCH in web_teleop/teleop_bridge.py
s = s.replace('value="${9.8715*cm2m}"/>',
    'value="${9.8715*cm2m}"/>\n'
    '  <xacro:property name="top_lidar_pitch"    value="0.30"/>\n'
    '  <xacro:property name="bottom_lidar_pitch" value="-0.30"/>', 1)

old = ('  <xacro:rplidar name="rplidar" parent_link="shell_link" gazebo="$(arg gazebo)">\n'
       '    <origin xyz="${rplidar_x_offset} ${rplidar_y_offset} ${rplidar_z_offset}"\n'
       '            rpy="0 0 ${pi/2}"/>\n'
       '  </xacro:rplidar>')
# mounted at the FRONT EDGE of the sensor plate (plate r=0.137, centered ~x=-0.02,
# so front edge ~+0.11), with a 0.06 m gap above/below so the bodies clear the plate.
new = ('  <xacro:rplidar name="rplidar" parent_link="shell_link" gazebo="$(arg gazebo)">\n'
       '    <origin xyz="0.10 ${rplidar_y_offset} ${tower_sensor_plate_z_offset + 0.06}"\n'
       '            rpy="0 ${top_lidar_pitch} 0"/>\n'
       '  </xacro:rplidar>\n'
       '  <xacro:rplidar name="lidar_bottom" parent_link="shell_link" gazebo="$(arg gazebo)">\n'
       '    <origin xyz="0.10 ${rplidar_y_offset} ${tower_sensor_plate_z_offset - 0.06}"\n'
       '            rpy="${pi} ${bottom_lidar_pitch} 0"/>\n'
       '  </xacro:rplidar>')

assert old in s, "rplidar instantiation block not found (turtlebot4_description changed?)"
s = s.replace(old, new, 1)
assert s.count('xacro:rplidar name=') == 2, "expected exactly 2 lidars after injection"
f.write_text(s)
print("two lidars injected (top + lidar_bottom)")
