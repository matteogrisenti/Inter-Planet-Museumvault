run scirpts to open first and second terminal with docker

then for each terminal:

cd ~/plansys2_ws
rosdep install --from-paths src/museum --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# For museum_plansys2

cd src/museum

## Terminal 1
cd launch
ros2 launch museum launch.py

(se da problemi al launch) rm -rf build/ install/ log/ src/build/ src/install/ src/log/
e poi ricompila  colcon build --symlink-install

# Terminal 2
cd root/plansys2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
cd src/museum
bash commands_easy.sh


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