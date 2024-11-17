# Robotics-HomeServiceRobot

Home Service Robot is a project that simulated in ROS and Gazebo. The robot perform SLAM to map an indoor environment, localizes, and autonomously navigate to pick up item and place it to drop off point around the house. 

This project is part of course 6 in Udacity Robotics Software Engineer Nanodegree.

<img src="images/home_service_robot.gif" alt="homeServiceRobot" width="" height="400"></a>

## Project Info
This project deploy turtlebot.
The project consist of:
1. Indoor environment created from Gazebo building editor.
2. 2D occupacy grid map generated from gmapping package.
3. Use Adaptive Monte Carlo Localization (AMCL) from amcl package to localize robot.
4. pick_objects node to navigate robot to pick up and drop off zone.
5. add_markers node that subscribe to robot odometry and publish markers to simulate object pick up and drop off.

<img src="images/gazebo.PNG" alt="gazebo" width="" height="250"></a>

## Prerequisites
1. Linux (Ubuntu 20.04)
2. ROS (Noetic) packages
3. Gazebo

## Dependencies
The following dependencies need to be installed:
```
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get install ros-noetic-turtlebot3
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
$ sudo apt-get install ros-noetic-openslam-gmapping
$ sudo apt-get install ros-noetic-navigation
```

## Project Structure
```
. Robotics-HomeServiceRobot
├── README.md
├── add_markers                         # Publishes markers to Rviz to simulate the object pick-up and drop-off
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── add_markers.cpp
├── maps                                # Simulation maps
|   ├── myrobot0125map.pgm
|   └── myrobot0125map.yaml
├── rvizConfig                          # Simulation Rviz file with marker
|   └── home_service.rviz
├── scripts                             # Shell scripts
|   ├── add_markers.sh
|   ├── home_service.sh
|   ├── pick_objects.sh
|   ├── test_navigation.sh
|   └── test_slam.sh
├── worlds                              # Simulation world
|   └── Project1_MyHomeWorld.world
├── images                              # Simulation images
│   ├── gazebo.PNG
│   ├── slam_testing.PNG
│   ├── slam_testing_map_sample.PNG
│   ├── home_service_robot.gif
│   └── sampleResultAfterDelivered.PNG
└── pick_objects                        # Commands the robot to navigate to the desired pick-up and drop-off zones
    ├── CMakeLists.txt
    ├── package.xml
    └── src
        └── pick_objects.cpp
```

## How to build
1. Initialize a catkin workspace and clone this project to catkin src directory
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.com/yeetengang/Robotics-MapMyWorld.git
$ mv Robotics-HomeServiceRobot/* ./
```

2. Build
```
$ cd ..
$ catkin_make
```

3. Source environment
```
$ source devel/setup.bash
```

5. Make scripts to be executable
```
$ cd src/scripts
$ sudo chmod +x *.sh
```

## SLAM Testing
Perform SLAM by teleoperating robot via teleop keyboard.
```
$ ./test_slam.sh
```
This shell script will launch:
1. turtlebot_world.launch to deploy turtlebot into my world.
2. gmapping_demo.launch to perform SLAM.
3. view_navigation.launch to observe map in Rviz.
4. keyboard_teleop.launch to allow teleoperating robot via keyboard.

<img src="images/slam_testing.PNG" alt="sampleresult_slam" width="" height="250"></a>

Use keyboard to navigate around the world, save the map via commands below:
```
rosrun map_server map_saver -f <map-location-and-name>
```

<img src="images/slam_testing_map_sample.PNG" alt="sampleresultMap" width="" height="250"></a>

Below is the sample result after I navigate some part of my world:

## Localization and Navigation Testing
Check robot's ability to reach selected goal.
```
$ ./test_navigation.sh
```
This shell script will launch:
1. turtlebot_world.launch to deploy turtlebot into my world.
2. amcl_demo.launch to localize turtlebot with my previously generated map file.
3. view_navigation.launch to observe map in Rviz.

Press the 2D Nav Goal button in Rviz and click on map to select goal for robot to navigate.
With the supports of AMCL package, it implement Adaptive Monte Carlo Localization approach, which uses particle filter for tracking pose of the robot with respect to the known map provided. 

## Navigation Goal Node
Autonomously navigate robot to pick-up zone and drop-off zone.
```
$ ./pick_objects.sh
```
This shell script will launch:
1. turtlebot_world.launch to deploy turtlebot into my world.
2. amcl_demo.launch to localize turtlebot with my previously generated map file.
3. home_service_rviz.launch which will take my Rviz configuration to have better view of pick-up zone and drop-off zone.
4. pick_objects node with pick_objects.cpp function.

pick_objects.cpp include function to navigate robot from starting location to pick-up location. Once robot reached pick-up location, it stay there for 5 seconds and then proceed to move to drop-off location. 

## Virtual Objects
Model a virtual object to simulate item pick-up and drop off events.
```
$ ./add_markers.sh
```
This shell script will launch:
1. turtlebot_world.launch to deploy turtlebot into my world.
2. amcl_demo.launch to localize turtlebot with my previously generated map file.
3. home_service_rviz.launch which will take my Rviz configuration to have better view of pick-up zone and drop-off zone.
4. add_markers node with add_markers_time.cpp function, pick-up drop-off location params are defined in marker_config.yaml file.

add_markers_time.cpp include function to spawn marker at pick-up location for 5 seconds. Then, hide the marker for another 5 seconds to simulate item being picked up. Finally, re-spawn the marker at drop-off location to simulate item being dropped off at drop-off point.

## Home Service Robot
Simulate full home service robot which will autonomously navigate to pick up zone, pick up item, navigate to drop off point, and drop the item at drop-off point:
```
$ ./home_service.sh
```
This shell script combine pick_objects and add_markers nodes to simulate a complete event of a robot navigate, pick up item at point A, carrying the item, and drop off item to point B.
It takes the function from pick_objects.cpp and add_markers.cpp.
add_markers.cpp works similar as add_markers_time.cpp, but instead of hidding the virtual object after 5 seconds, it subscribe to odometry values to know robot current location. With this data, the function will hide the virtual object only when the robot is close enough to the virtual objects, or re-spawn the virtual object only when the robot is close enough to the drop-off point.

The robot will use the generated map and localize itself with acml package. 
Robot will navigate to a virtual object, pick-up the object, and navigate back to drop-off zone.
The virtual object, representing as a green cube, will disappear when robot get close to it (To indicate pick-up action), and re-appear at drop-off zone when robot navigate to drop off point.
Result can be seen from the gif displayed at above.

<img src="images/sampleResultAfterDelivered.PNG" alt="sampleresult" width="" height="250"></a>

