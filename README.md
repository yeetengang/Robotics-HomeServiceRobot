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
|   ├── myrobot0125map.yaml
├── rvizConfig                          # Simulation Rviz file with marker
|   ├── home_service.rviz
├── scripts                             # Shell scripts
|   ├── add_markers.sh
|   ├── home_service.sh
|   ├── pick_objects.sh
|   ├── test_navigation.sh
|   ├── test_slam.sh
├── worlds                              # Simulation world
|   ├── Project1_MyHomeWorld.world
├── images                              # Simulation images
│   ├── home.png
│   ├── home_service.gif
│   ├── home_service_map.png
│   └── rviz.png
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

## Navigation and Navigation Testing
Check robot's ability to reach selected goal.
```
$ ./test_navigation.sh
```
Press the 2D Nav Goal button in Rviz and click on map to select goal for robot to navigate.

## Home Service Robot
To test with home service robot which will autonomously navigate to pick up zone and return to drop off point:
```
$ ./home_service.sh
```
The robot will use the generated map and localize itself with acml package. 
Robot will navigate to a virtual object, pick-up the object, and navigate back to drop-off zone. 
The virtual object, representing as a green cube, will disappear when robot get close to it (Picked up indication), and appear at drop-off zone when robot navigate to drop off point.

<img src="images/sampleResultAfterDelivered.PNG" alt="sampleresult" width="" height="250"></a>

