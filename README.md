# Environment setup

* JetPack 6.0 (L4T 36.3 - Ubuntu 22.04 based)

## Upgrade packages

```
sudo apt update
sudo apt upgrade
sudo reboot
```

## Install ROS2

Install ROS2 desktop and development tools. ([source](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))

```
sudo apt install software-properties-common curl

sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

## Install uv

Install uv. ([source](https://docs.astral.sh/uv/getting-started/installation/))

```
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Restart the shell.

## Install lerobot_ros2_so101

Create a ROS2 workspace.

```
mkdir -p ~/ros2_ws/src
```

Clone the ROS2 package into ROS2 workspace.

```
cd ~/ros2_ws/src
git clone https://github.com/SeeedJP/lerobot_ros2_so101.git
```

Install dependency packages.

```
cd ~/ros2_ws/src/lerobot_ros2_so101
uv sync
```

## Build lerobot_ros2_so101

Build the ROS2 package.

```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/src/lerobot_ros2_so101/.venv/bin/activate
colcon build --packages-select lerobot_ros2_so101 --symlink-install
```

# Run Leader Arm -> lerobot_ros2_so101 -> RViz

Set permissions for /dev/ttyACM0.

```
sudo chmod 666 /dev/ttyACM0
```

Run the ROS2 node.

```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ./install/setup.sh
source ~/ros2_ws/src/lerobot_ros2_so101/.venv/bin/activate
ros2 run lerobot_ros2_so101 leader_node.py
```

Visualize SO-101 using RViz.

```
cd ~/Lerobot_ros2
source /opt/ros/humble/setup.bash
source ./install/setup.sh
export DISPLAY=:0.0
ros2 launch so101_follower_description display.launch.py use_gui:=false joint_states_topic:=/joint_states
```

# Appendix

## Calibration

### Calibrate the leader arm

```
cd ~/ros2_ws/src/lerobot_ros2_so101
uv run -- python -m lerobot.calibrate \
    --teleop.id=lerobot_leader \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0
```

### Calibrate the follower arm

```
cd ~/ros2_ws/src/lerobot_ros2_so101
uv run -- python -m lerobot.calibrate \
    --robot.id=lerobot_follower \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0
```

### Calibration file

`/home/jetson/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/lerobot_leader.json`

```
{
    "shoulder_pan": {
        "id": 1,
        "drive_mode": 0,
        "homing_offset": -1206,
        "range_min": 1720,
        "range_max": 2418
    },
    "shoulder_lift": {
        "id": 2,
        "drive_mode": 0,
        "homing_offset": 987,
        "range_min": 1706,
        "range_max": 2392
    },
    "elbow_flex": {
        "id": 3,
        "drive_mode": 0,
        "homing_offset": 1722,
        "range_min": 1753,
        "range_max": 2437
    },
    "wrist_flex": {
        "id": 4,
        "drive_mode": 0,
        "homing_offset": 752,
        "range_min": 1755,
        "range_max": 2463
    },
    "wrist_roll": {
        "id": 5,
        "drive_mode": 0,
        "homing_offset": 1960,
        "range_min": 1780,
        "range_max": 2377
    },
    "gripper": {
        "id": 6,
        "drive_mode": 0,
        "homing_offset": 378,
        "range_min": 1713,
        "range_max": 2383
    }
}
```

## Create ROS2 package + uv

Create a ROS2 workspace.

```
mkdir -p ~/ros2_ws/src
```

Create a ROS2 package.

```
source /opt/ros/humble/setup.bash
ros2 pkg create lerobot_ros2_so101 --license MIT
```

Add uv to the ROS package.

```
cd ~/ros2_ws/src/lerobot_ros2_so101
uv init . --lib --python-preference only-system
uv venv --system-site-packages
```

Add dependency packages.

```
uv add 'lerobot[feetech]'
uv sync
```

Edit files.

```
mkdir ~/ros2_ws/src/lerobot_ros2_so101/scripts
vi ~/ros2_ws/src/lerobot_ros2_so101/scripts/leader_node.py
chmod +x ~/ros2_ws/src/lerobot_ros2_so101/scripts/leader_node.py
vi ~/ros2_ws/src/lerobot_ros2_so101/src/lerobot_ros2_so101/__init__.py
vi ~/ros2_ws/src/lerobot_ros2_so101/CMakeLists.txt
```

# Links

* [LeRobot SO-101 テレオペから学習データ取得まで](https://zenn.dev/karaage0703/articles/8042463b476fbf)
* [Jetson OrinでROS2 Humble入門](https://zenn.dev/karaage0703/articles/d778d660703c0b)
* [PythonのuvをROS2で使う](https://qiita.com/GesonAnko/items/510eeade1f8ada302b9b)
* [LeRobot ROS2 - SO101 Follower Arm](https://github.com/AgRoboticsResearch/Lerobot_ros2)
