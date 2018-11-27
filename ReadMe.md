[![Build Status](https://travis-ci.com/mayankpathak10/AGV_Navigation.svg?branch=master)](https://travis-ci.com/mayankpathak10/AGV_Navigation) [![Coverage Status](https://coveralls.io/repos/github/mayankpathak10/AGV_Navigation/badge.svg?branch=master)](https://coveralls.io/github/mayankpathak10/AGV_Navigation?branch=master) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)



## Overview:
The main idea of this project is to implement a path planning algorithm on an Automated Guided Vehicle (AGV) to plan its route from a start point to end point. An AGV is an advanced mobile robot that follows wires, markers, invisible UV markers, cameras and lasers to move. AGVs follow a defined path every time, they are not suitable for non-repetitive tasks and decrease the flexibility of operations overall.

We aim to demonstrate that the above-mentioned limitations of AGV can be alleviated by implementing a novel path planning technique (A* algorithm) for searching the path in a known environment. To get the information about the environment, this method is topped by a g-mapping technique using SLAM and develop a binary map of the environment to feed to the A* algorithm.

The AGV (robot here) drives around in a simulated world, navigating through obstacles using LIDAR sensor. Whenever an obstacle is recognized, the g-mapping algorithm prepares a binary 2D map of the environment. For example, if the robot finds an obstacle while moving around, like a wall or any solid surface, it refers that coordinates as 1 , meaning that there is a obstacle in front of the robot.

After preparing a map of the environment, the robot uses path planning algorithm (A* algorithm) and finds a optimal path between its current position and the goal point.

## Project Approach:
1. Develop a custom map in the gazebo.
	A map containing walls and other obstacles representing industrial workshop scenario is prepared.  
2. Use g-mapping to get occupancy grid of the map.
	Using the sensors (lidar for gmapping) avaiable on the turtle bot, 2D g-map is prepared.
3. Using this map, a path between two given points (Start point and Destination) is calculated using the below path planning algorithm.
4. Implement Astar algorithm to find the path.
	A* algorithm is one of the best algorithm to approximate the shortest path between two points on a map or graph.  At each step, it proceeds by selecting the node which has lowest sum of the two parameters: movement cost from start node to current node, and heuristic distance between current node and goal node.
5. TurtleBot navigates, avoiding obstacles and reaches the destination. 
	After the path is known, turtlebot navigates in the simulated gazebo world using the node coordinates.
6. Quality is assured by delivering Unit Tests for overall source code and full coverage in coveralls. 

## Deveopment Using Solo Iterative Process (SIP) and Test-Driven Development (TDD)
In development of this software module, along with test-driven development(TDD), solo interative process (SIP) was followed. Using SIP this software module is divided into two parts: the product backlog, and code of the software.

First, the product backlog was developed as per the requirements. From these entries, highest priority requirements were selected for first-week sprint. In the product backlog, estimated time of completion was allotted to every task. Actual completion time was compared with estimated completion time and based on that, time allotment for future tasks was modified.

After the planning was done, TDD process was used in this which UML activity and class diagrams were developed according to the requirements of the project. Based on UML class diagram, unit tests will written. Then stub classes were written with functions matching the test cases.

Following is the link to the spreadsheet that contains detailed entries of the product backlog, time log, error log and release backlog: [Product Backlog](https://docs.google.com/spreadsheets/d/1xaJ7oK8ZxaKgJi8yriTa_Gf3EtpvXDz0xDdI2Yvn0hE/edit?usp=sharing)

## Sprint Planning
This project development is divided into 3 weekly sprints. Detailed Sprint planning can be found in the below file:
[Sprint Planning Notes](https://docs.google.com/document/d/1IQnlFH5AhvnU3X79OxwiI05bf_QO1nF9Tf4WWig5hUM/edit?usp=sharing)

## Dependencies
- Ubuntu 16.04
- ROS Kinetic
- Catkin
- Gazebo
- Turtlebot Gazebo package
- Packages included in ROS Kinetic:
	- roscpp
	- std_msgs
	- genmsg
	- geometry_msgs

## How to build
< will be updated >

## How to run
< will be updated >

## Testing
< will be updated >

## Doxygen Documentation
< will be updated >

## About Authors
Bhargav Dandamudi: Graduate Student in M.Eng., Robotics, working as Teaching Assitant of Grad. Level Robotics Course. I am intrested to pursue career in Computer vision, planning and Medical Robotics related field. 

Mayank Pathak: Graduate Student in M.Eng., Robotics, working as Research Assistant in Sensors and Actuators Lab in the University of Maryland, College Park. I am interested to pursue career in the field of Computer Vision and Machine Learning.

## License
MIT License

Copyright (c) 2018 Mayank Pathak, Bhargav Dandamudi

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.