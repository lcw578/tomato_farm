#!/bin/bash
# 单株串收番茄查看（倾斜藤+红/红/半/青四串+顶部叶花）
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
export GAZEBO_MASTER_URI=http://localhost:11390
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="$HOME/aoc_tomato_farm/aoc_tomato_farm_gazebo/models/22mx14m:$GAZEBO_MODEL_PATH"
WORLD="$HOME/aoc_tomato_farm/vine_view.world"
for pid in $(pgrep -x gzserver; pgrep -x gzclient); do
  if tr '\0' '\n' < /proc/$pid/environ 2>/dev/null | grep -q "GAZEBO_MASTER_URI=http://localhost:11390"; then
    kill -9 "$pid" 2>/dev/null
  fi
done
sleep 1
nohup gzserver "$WORLD" --verbose > /tmp/vine_gzserver.log 2>&1 &
sleep 8
nohup gzclient "$WORLD" > /tmp/vine_gzclient.log 2>&1 &
echo "Gazebo 打开中（单株查看）"
