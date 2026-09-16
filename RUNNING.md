# 克隆后如何运行（RUNNING.md）

本仓库为 ROS 2 Humble + Gazebo Classic 11 的番茄农场仿真。克隆后按以下步骤运行。

## 0. 环境要求

- **Ubuntu 22.04**（ROS 2 Humble 仅支持 22.04，24.04 无法安装本项目所需环境）
- ROS 2 Humble + Gazebo Classic 11

```bash
sudo apt update
sudo apt install ros-humble-desktop gazebo11 libgazebo11-dev \
  ros-humble-gazebo-ros-pkgs ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
```

## 1. 克隆（必须克隆到 ~/aoc_tomato_farm）

所有脚本硬编码了 `$HOME/aoc_tomato_farm` 路径，克隆时必须指定目标目录：

```bash
git clone <仓库地址> ~/aoc_tomato_farm
```

## 2. 编译

```bash
cd ~/aoc_tomato_farm
colcon build --packages-select aoc_tomato_farm_gazebo franka --symlink-install
source install/setup.bash
```

## 3. 运行

| 场景 | 命令 |
|---|---|
| 完整温室农场（153 棵番茄，一键） | `bash ~/aoc_tomato_farm/view_farm.sh` |
| 标准 ROS 2 启动方式（默认 22mx14m 农场） | `ros2 launch aoc_tomato_farm_gazebo tomato_farm_world.launch.py` |
| 单株番茄（免编译，最快） | `bash ~/aoc_tomato_farm/view_vine.sh` |
| 番茄果串查看 | 先建软链接（见下），再 `bash ~/aoc_tomato_farm/view_truss.sh` |

果串查看所需的模型软链接：

```bash
ln -s ~/aoc_tomato_farm/aoc_tomato_farm_gazebo/models/22mx14m/tomato1_truss ~/.gazebo/models/tomato1_truss
```

## 已知问题

- `panda_ign_moveit2/` 为第三方仓库，未包含在本仓库内；MoveIt 示例需单独克隆。
