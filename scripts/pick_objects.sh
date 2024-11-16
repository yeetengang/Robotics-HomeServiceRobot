#!/bin/sh

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../worlds/Project1_MyHomeWorld.world " & 

sleep 5

# launch amcl_demo.launch for localization
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../maps/myrobot0125map.yaml " &

sleep 5

# launch view_navigation for rviz
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 20 # Wait until visualization is ready

# launch pick_objects node
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun pick_objects pick_objects " &