#!/bin/bash
# 番茄串查看（最终版：全部走默认路径，无 /tmp 依赖）
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; sleep 2
source /opt/ros/humble/setup.bash 2>/dev/null
source $HOME/aoc_tomato_farm/install/setup.bash 2>/dev/null
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=$HOME/.gazebo/models:/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH

echo "=== 启动 Gazebo Server ==="
gzserver $HOME/aoc_tomato_farm/truss_show.world &
SPID=$!
sleep 8

echo "=== 插入 3 株果串 ==="
gz model -m truss_a -f $HOME/.gazebo/models/tomato1_truss/model.sdf -x 0 -y 0 -z 0.3
gz model -m truss_b -f $HOME/.gazebo/models/tomato1_truss/model.sdf -x 0.35 -y -0.25 -z 0.3 -Y 1.2
gz model -m truss_c -f $HOME/.gazebo/models/tomato1_truss/model.sdf -x -0.3 -y 0.3 -z 0.3 -Y 2.5

echo "=== 启动 Gazebo Client ==="
gzclient &
sleep 3
echo "=== 完成（Ctrl+C 退出）==="
wait $SPID
