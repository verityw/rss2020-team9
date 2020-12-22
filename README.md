# 6.141J/16.405J - Robotics: Science and Systems - Spring 2020
## Team 9: SDWCH
## William Chen, Henry Hu, Christina Jung, Silvia Knappe, Diego Mendieta

## THIS PAGE IS CURRENTLY A WORK IN PROGRESS

## Contents
1. [Introduction](#introduction)
0. [Technical Details](#technical-details)
0. [Lab 3: Wall Following and Safety Controller](lab-3-wall-following-and-safety-controller)
0. [Lab 4: Vision and Line Following](lab-4-vision-and-line-following)
0. [Lab 5: Localization](#lab-5-localization)
0. [Lab 6:]
0. [Final Challenge](#final-challenge)

## Introduction
This repository contains the code, lab reports, and lab briefing presentations for Team 9 of MIT's capstone undergraduate robotics class, Robotics: Science and Systems (RSS), as offered in Spring 2020.
We implemented several key robotics and autonomous driving algorithms in simulation and on a real robot platform. These algorithms include several computer vision techniques (color segmentation, SIFT, and camera homography transformations), pure pursuit controllers, [RRT*](https://arxiv.org/abs/1105.1186) for pathfinding, and particle filters for localization. Each assignment culminated in a lab report and team presentation discussing both the theory and experimental results of each of our algorithms. At the end of the class, we had a final autonomous racing challenge in simulation, wherein our team placed second.

## Technical Details
We implemented the following algorithms using the Python bindings for the [Robot Operating System (ROS)](ros.org) message-passing framework. Development, simulation testing, and the final challenge were all done in [Ubuntu 18.04](https://releases.ubuntu.com/18.04.5/). The initial labs were also implemented on a physical robot racecar platform sporting a [ZED Stereo Camera](https://www.stereolabs.com/zed/), [Hokuyo UST-10LX Range Finder](https://hokuyo-usa.com/products/lidar-obstacle-detection/ust-10lx) LIDAR, and [NVIDIA Jetson TX2](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-tx2/) as the onboard computer, also running the Ubuntu 18.04. More details regarding the system can be found [here](https://racecar.mit.edu/).

## Lab 3: Wall Following and Safety Controller
Paper · Briefing · Code

In this assignment, we wrote a basic wall following algorithm and safety controller, both using the racecar's LIDAR sensor. We initially decide whether the robot should follow the wall on its left or right. From there, we only considered the LIDAR points to that side of the car, as well as those directly in front of it. In order to model the wall that generated said LIDAR data, we used linear regression to fit a straight line to the perceived data. This then allowed us to find a path for the car to follow that is parallel to the wall. Finally, we created two simple control algorithms for the car -- one using pure pursuit to chase a point on the line and the other using PID control to ensure that the car at the correct distance from the wall. Both controllers use the Ackermann two-wheel drive model for the dynamics of the car.

## Lab 4: Vision and Line Following
Paper · Briefing · Code

This lab focused on simple visual control. The end goal was to have the car chase/back up and park in front of an orange miniature traffic cone in its field of view. To achieve this task, we completed four modules.

### 1. Color Segmentation
To find the cone in an image, we simply look at which observed pixels fall into some range of color values close to the true orange color of the cone.
### 2. SIFT and Template Matching
Scale invariant feature transforms (SIFT) is used for giving local descriptors/feature values for regions of an image. It is especially useful for for finding matching features in two images (perhaps of the same scene, but from different views). We tested our implementation of SIFT on various provided images.
### 3. Camera Homography Calibration
The above modules provide ways of locating the cone in an image, but not of determing where it is in relation to the racecar. One can determine the relative location of an object to a camera by finding the camera's homography matrix, a projection matrix from 3D points to 2D pixel coordinates. To do this, we sampled numerous positions on the ground in front of the robot at known, ground-truth positions, then looked at the robot's vision feed to determine the points' corresponding pixel coordinates. From there, a regressive model found the matrix which would best project the points to the 2D image coordinates. Altogether, this allows one to determine the approximate cartesian coordinates of any pixel in the field of view, meaning that the system can now localize the cone when it is within the camera's field of view.
### 4. Parking and Line Following
Finally, we adapted the pure pursuit controller from Lab 3 to chase and park in front of a desired point relative to the robot. Specifically, if the point is too far from the robot, it attempts to drive towards the point while turning to face it directly, stopping once at the desired relative distance. On the other hand, when the point is too close, the car drives back and reorients itself to face the point. To switch between these two modes, we implemented a simple state machine that evolves upon the robot reaching certain distance thresholds from the point. For line following, this simply involved setting the point to be slightly ahead of the robot on the desired path line.
### Integration
The full pipeline thus involves segmenting an object of interest in the robot's visual feed with the first two modules, using the computed homography matrix to figure out the object's relative position and distance from the robot, and finally using the state machine and controller to try and chase/park in front of the object.

*We intended on testing this pipeline for both line following and cone chasing on the physical robot platform. However, as we were in the process of working on this assignment when the COVID-19 shifts to remote work/schooling began in March 2020, we did not end up finishing the physical experiments and lab report.*

## Lab 5: Localization
Paper · Briefing · Code

The goal of this lab was to develop an algorithm for the racecar to localize itself in a known environment. We develop a probabilistic model of the car's motion based off of the one presented in [*Probabilistic Robotics*](http://www.probabilistic-robotics.org/) -- effectively, after moving, the robot uses its (potentially noisy) odometry readings to compute how it likely moved. The movement then has a degree of stochasticity injected in, as the predicted motion randomly deviates from the odometricly calculated one in terms of both translation and rotation.

This motion model is then used in our particle filter localizer. When initialized, the robot generates numerous particles, each representing estimates of its pose. After moving, each of those particles' poses are updated semi-randomly with the above motion model. Finally, we the theoretical LIDAR observations the robot would get if it were at each of those estimated poses with the true LIDAR data. This assigns each of the estimates a score -- effectively a probability of the particle being (close to) the true pose. Finally, we create a new set of points by sampling (with replacement) from the old ones, with higher-scoring particles being more likely to be chosen. Repeating this process as the robot moves, we thus get a discrete estimate of a probability distribution over the possible set of robot states.

## Lab 6: Path Planning and Pure Pursuit
Paper · Briefing · Code

For the final lab, we had to implement both a path planning algorithm and a pure pursuit controller to follow said path. For the former implementation, we had several options, including optimal discrete space graph search algorithms and continuous probabilistic planners. We chose the second option, specifically writing a function that runs the [optimal rapidly-exploring random tree](https://arxiv.org/abs/1105.1186) search algorithm.

The core of the algorithm is to grow an optimally-connected tree towards randomly sampled regions of space, performing "rewirings" to the connections of the tree such that the connected path from the starting state to any node is as short as possible (by some metric). Doing so generally requires updating the cost of all descendants of a rewired node, but provides the mathematical guarantee that, as runtime increases, the path from the start to any node converges to the optimal one. Nevertheless, initial implementations thus had high computational complexity and were generally inefficient; we thus changed the algorithm to not perform as many recursive cost updates, thereby losing optimality but also speeding up exploration and expansion of the tree via vectorization. Ultimately, this heuristic algorithm proved to be effective at rapidly finding near-optimal paths from start to end in our considered map. For the robot to follow said path, we extended the pure pursuit controller from previous labs to work with the piecewise-linear path computed by RRT*. This similarly required vectorization to complete in an efficient manner.

## Final Challenge:
Briefing · Code
