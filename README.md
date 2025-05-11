# Installation

### 1. Creating a ROS2 workspace
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```
### 2. Clone repository
```bash
git clone https://github.com/vladislav-parkhachev/turtlebot3_burger_ros2.git
```
### 3. Install dependencies
```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

