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



###  Run teleop_twist_keyboard or teleop_twist_joy
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True -r cmd_vel:=/diff_drive_base_controller/cmd_vel
```
```bash
ros2 launch robot_teleop joystick.launch.py
```


