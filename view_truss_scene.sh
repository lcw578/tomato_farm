#!/bin/bash
# 番茄串场景查看（spawn 式插入 tomato1_truss 果串模型）
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; sleep 1
source /opt/ros/humble/setup.bash 2>/dev/null
source $HOME/aoc_tomato_farm/install/setup.bash 2>/dev/null
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=$HOME/aoc_tomato_farm/install/aoc_tomato_farm_gazebo/share/aoc_tomato_farm_gazebo/models/22mx14m:$HOME/.gazebo/models:/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
export GAZEBO_MASTER_URI=http://localhost:11390
pkill -9 -f gzserver 2>/dev/null; sleep 1
cat > /tmp/truss_scene.world <<'W'
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="truss_scene">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <gui><camera name="user_camera"><pose>1.6 -1.5 1.4 0 0.15 0.85</pose></camera></gui>
  </world>
</sdf>
W
gzserver /tmp/truss_scene.world > /tmp/ts_s.log 2>&1 &
sleep 6
gz model -m truss_a -f $HOME/.gazebo/models/tomato1_truss/model.sdf -x 0 -y 0 -z 0 2>/dev/null
gz model -m truss_b -f $HOME/.gazebo/models/tomato1_truss/model.sdf -i -x 0.45 -y 0.3 -z 0 -Y 1.3 2>/dev/null
gz model -m truss_c -f $HOME/.gazebo/models/tomato1_truss/model.sdf -i -x -0.4 -y 0.35 -z 0 -Y 2.7 2>/dev/null
echo "3 株果串已插入"
gzclient &
wait
