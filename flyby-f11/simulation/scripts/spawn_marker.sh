#!/bin/bash
#
# Spawn a simple visual marker in Gazebo to represent drone position
#

# Simple sphere model at spawn location
gz service -s /world/flyby_training/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req "sdf: '<?xml version=\"1.0\" ?>
<sdf version=\"1.9\">
  <model name=\"drone_marker\">
    <pose>0 0 5 0 0 0</pose>
    <static>false</static>
    <link name=\"link\">
      <visual name=\"visual\">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      <collision name=\"collision\">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>'"

echo "Spawned red marker sphere at (0, 0, 5)"
echo "Note: This is just a visual marker, not connected to SITL physics"
