#!/bin/bash
# 一键查看温室串收番茄农场（纯环境，不启动机械臂）
# 用法: bash ~/aoc_tomato_farm/view_farm.sh
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

# 项目专用端口（11390），与 HEIGHT_CAG 等其他项目隔离
export GAZEBO_MASTER_URI=http://localhost:11390
export GAZEBO_MODEL_DATABASE_URI=""
# 串收番茄/温室大棚资产在 aoc 模型树，机械臂自身模型在 franka 模型树
export GAZEBO_MODEL_PATH="$HOME/aoc_tomato_farm/install/aoc_tomato_farm_gazebo/share/aoc_tomato_farm_gazebo/models/22mx14m:$HOME/aoc_tomato_farm/install/franka/share/franka/models:$GAZEBO_MODEL_PATH"

WORLD="$HOME/aoc_tomato_farm/install/franka/share/franka/worlds/tomato_farm_22mx14m_gazebo_classic.world"

# 只清理绑定在本项目端口(11390)上的旧实例，不碰其他项目的 Gazebo
for pid in $(pgrep -x gzserver; pgrep -x gzclient); do
  if tr '\0' '\n' < /proc/$pid/environ 2>/dev/null | grep -q "GAZEBO_MASTER_URI=http://localhost:11390"; then
    kill -9 "$pid" 2>/dev/null && echo "已清理旧实例 $pid"
  fi
done
sleep 1

echo "启动 gzserver（世界含 75 棵随机化果串番茄（12 变体混排+朝向抖动）+ 温室，加载约 10~30 秒）..."
nohup gzserver "$WORLD" --verbose > /tmp/aoc_gzserver.log 2>&1 &
sleep 15

echo "启动 gzclient..."
nohup gzclient "$WORLD" > /tmp/aoc_gzclient.log 2>&1 &

echo "完成。Gazebo 窗口即将出现。"
echo "日志: /tmp/aoc_gzserver.log  /tmp/aoc_gzclient.log"
echo "重新运行本脚本即可自动清理旧实例并重启（不影响其他项目的 Gazebo）。"
