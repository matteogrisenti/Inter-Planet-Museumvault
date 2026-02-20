Docker:
docker build --platform linux/amd64 --rm  --tag plansys2 . --file Dokerfile-humble

# se non c'è ros

colcon build --symlink-install
source /opt/ros/$ROS_DISTRO/setup.bash

# se mancano dependences
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

source /opt/ros/humble/setup.bash
source install/setup.bash

# For museum

## Terminal 1
cd src/museum/launch
ros2 launch museum launch.py

(se da problemi al launch) rm -rf build/ install/ log/ src/build/ src/install/ src/log/
e poi ricompila  colcon build --symlink-install

# Terminal 2
cd root/plansys2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
cd src/museum
bash commands.sh


----------

## Terminal 1
cd museum_plansys2/launch
ros2 launch museum launch.py

# Example 

TERMINAL 1 (copy in block)
cd plansys2_ws/plansys2_simple_example/launch
ros2 launch plansys2_simple_example plansys2_simple_example_launch.py

TERMINAL 2
cd root/plansys2_ws/
bash install_terminal.sh
cd src/museum
bash commands.sh


or
ros2 run plansys2_terminal plansys2_terminal < launch/commands