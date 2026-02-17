run scirpts to open first and second terminal with docker

then for each terminal:

cd plansys2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash

# For museum_plansys2

cd plansys2_ws_museum


## Terminal 1
cd museum_plansys2/launch
ros2 launch museum launch.py

# Example 

TERMINAL 1 (copy in block)
cd ../../plansys2_ws/plansys2_simple_example/launch
ros2 launch plansys2_simple_example plansys2_simple_example_launch.py

TERMINAL 2
cd ../../plansys2_ws/
bash install_terminal.sh
cd plansys2_simple_example/
bash commands.sh


or
ros2 run plansys2_terminal plansys2_terminal < launch/commands