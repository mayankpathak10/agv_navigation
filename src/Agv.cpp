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
 * @file  Agv.cpp
 * @brief agv_navigation package
 *
 * @section DESCRIPTION
 *
 *  This file implements the exploration algorithm for turtlebot.
 *  it subscribes to laser scan values from turtlebot and passes
 *  it to explorer class to provide direction to the turtlebot.
 *
 *  @Dependencies: This file depends on "Agv.hpp" and "Explorer.hpp"
 *
 * @author Bhargav Dandamudi and Mayank Pathak
 * @version 1
 * @date 2018-12-15
 */

#include "../include/Agv.hpp"
#include <ros/ros.h>
#include <iostream>
#include "../include/Explorer.hpp"


/**
 * @brief      Class to subscribe to laserscan and publish velocities.
 */
Agv::Agv() {
    subLaserScan =
        n.subscribe("/scan", 1000, &Explorer::ScanCallback, &explorer);

    Rotatetimer = n.createTimer(ros::Duration(40),
                                &Explorer::RotatetimerCallback, &explorer);
    actionPub = n.advertise<geometry_msgs::Twist>(
                    "/mobile_base/commands/velocity", 100);
}

bool Agv::explore() {
    // publishes twist messages to make trutleBot move
    actionPub.publish(explorer.direction());
    return true;
}

Agv::~Agv() {}
