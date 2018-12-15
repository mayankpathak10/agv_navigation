#include "../include/Agv.hpp"
#include <ros/ros.h>
#include <iostream>
#include "../include/Explorer.hpp"

Agv::Agv() {
    subLaserScan =
        n.subscribe("/scan", 1000, &Explorer::ScanCallback, &Explorer);

    Rotatetimer = n.createTimer(ros::Duration(40),
                                &Explorer::RotatetimerCallback, &Explorer);
    actionPub = n.advertise<geometry_msgs::Twist>(
        "/mobile_base/commands/velocity", 100);
}

bool Agv::explore() {
    // publishes twist messages to make trutleBot move
    actionPub.publish(Explorer.direction());
    return true;
}

Agv::~Agv() {}
