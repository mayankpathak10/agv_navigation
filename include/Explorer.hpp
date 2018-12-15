/**
 * MIT License
 *
 * Copyright (c) 2018 Mayank Pathak, Bhargav Dandamudi
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the software is furnished to do so,
 * subject to the following conditions:

 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
 * THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef INCLUDE_EXPLORER_HPP_
#define INCLUDE_EXPLORER_HPP_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Explorer {
 public:
    // default constructor
    Explorer();

    float obstDist();
    bool rotateBot();
    void setRotation();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& input);
    void RotatetimerCallback(const ros::TimerEvent& event);
    geometry_msgs::Twist direction();

    ~Explorer();

 private:
    bool rotaFlag;
    float distObst;
    int rotateCount;
    geometry_msgs::Twist action;
};

#endif   // INCLUDE_EXPLORER_HPP_
