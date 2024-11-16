#!/bin/sh

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../worlds/Project1_MyHomeWorld.world " &

sleep 15

# launch amcl_demo.launch for localization
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../maps/myrobot0125map.yaml " &

sleep 2

# launch rviz for visualization
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch add_markers home_service_rviz.launch rviz_config_file:=$(pwd)/../rvizConfig/home_service.rviz" &


sleep 15 # wait until visualization is ready

# launch add_markers node
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun add_markers add_markers " &

sleep 2

# launch pick_objects node
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun pick_objects pick_objects" &