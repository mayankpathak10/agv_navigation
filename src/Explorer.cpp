/*
 * @copyright (c) MIT License 2018 Bhargav Dandamudi, Mayank Pathak
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the 'Software'), to deal in the Software without
 * restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so,subject to
 * the following conditions:
 * The above copyright notice and this permission notice shall
 * be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED ''AS IS'', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM,OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @file  Explorer.cpp
 * @brief agv_navigation package
 *
 * @section DESCRIPTION
 *
 *  This is the main file to implement RRT algorithm on the turtlebot
 *  simulation in Gazebo. It uses a DWA global planner to generate a
 *  path to the given goal point and then navigate the robot to the goal
 *  in the gazebo environment.
 *
 *  @Dependencies: This file depends on "Explorer.hpp"
 *
 * @author Bhargav Dandamudi and Mayank Pathak
 * @version 1
 * @date 2018-12-16
 */

#include "../include/Explorer.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

Explorer::Explorer() : rotaFlag(true), distObst(0), rotateCount(0) {}

float Explorer::obstDist() { return distObst; }

bool Explorer::rotateBot() { return rotaFlag; }

void Explorer::setRotation() { rotaFlag = false; }

void Explorer::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& input) {
    float min = 10;
    for (const auto& dist : input->ranges) {
        if (min > dist) {
            min = dist;
        }
    }
    distObst = min;   // Finds distance to obstacle
    ROS_DEBUG_STREAM("Distance to obstacle: " << distObst);
}

void Explorer::RotatetimerCallback(const ros::TimerEvent& event) {
    if (distObst > .75) {
        // sets rotate flag every 45 seconds if no obstacle present
        rotaFlag = true;
        ROS_INFO_STREAM("Rotating in place to aid mapping");
    }
}

geometry_msgs::Twist Explorer::direction() {
    // Initialize the twist message
    action.linear.x = 0.0;
    action.linear.y = 0.0;
    action.linear.z = 0.0;
    action.angular.x = 0.0;
    action.angular.y = 0.0;
    action.angular.z = 0.0;
    rotateCount++;

    std::random_device rd;    // used to initialize (seed) engine //
    std::mt19937 rng(rd());   // random-number engine used (Mersenne-Twister)
    std::uniform_int_distribution<int> uni(-50, 100);
    float randomAngle = uni(rng);   // random number between 30 and 100
    // Stops and rotates the turtleBot in place for 5 times
    if (rotaFlag) {
        action.angular.z = 360 * (3.14 / 180);
    }
    if (rotateCount == 3) {
        setRotation();
        rotateCount = 0;
    }
    if (distObst > 0.75 && rotaFlag == false) {
        //  Linear motion in forward direction
        action.linear.x = 0.3;
        ROS_INFO_STREAM_THROTTLE(5, "Moving Forward");
    } else if (distObst < 0.75 && rotaFlag == false) {
        //  2D Rotation of about 90 degress
        action.angular.z = randomAngle * (3.14 / 180);
        ROS_WARN_STREAM("OBSTACLE DETECTED! Turning randomly: "
                        << action.angular.z * (180 / 3.14));
    }
    return action;
}

Explorer::~Explorer() {}
