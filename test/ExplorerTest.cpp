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

#include "../include/Explorer.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * @brief Unit test to check initial linear motion
 *
 * Checks if the turtlebot has no movement in x directionection
 * as we only want it to rotate initially
 */
TEST(ExplorerTest, InitialLinearTest) {
    Explorer exp = Explorer();
    EXPECT_EQ(0, exp.direction().linear.x);
}

/**
 * @brief Unit test to check initial angular rotation
 *
 * Checks if the action initially is to rotate the
 * turtlebot in place to explore environment
 */
TEST(ExplorerTest, InitialAngularTest) {
    Explorer exp = Explorer();
    EXPECT_NEAR(6.28, exp.direction().angular.z, 0.1);
}

/**
 * @brief Unit test for scan callback
 *
 * Checks if the obstacle distance is in the desired range
 */
TEST(ExplorerTest, CallbackTest) {
    ros::NodeHandle tn;
    Explorer exp = Explorer();
    ros::Subscriber subLaserScan =
        tn.subscribe("/scan", 1000, &Explorer::ScanCallback, &exp);
    EXPECT_LE(exp.obstDist(), 10);
}

/**
 * @brief Unit test for rotate flag
 *
 * Checks the setter and getter for rotate flag
 */
TEST(ExplorerTest, RotateFlagTest) {
    Explorer exp = Explorer();
    exp.setRotation();
    EXPECT_EQ(exp.rotateBot(), false);
}
/**
 * @brief Unit test to check timer callback functionality
 *
 * Checks if the timer callback was called and the necessary
 * flag was set
 */
TEST(ExplorerTest, InitialRotateTest) {
    ros::NodeHandle tn;
    Explorer exp = Explorer();
    EXPECT_EQ(exp.rotateBot(), true);
}
/**
 * @brief Unit test for random angle rotation
 *
 * Checks if the robot moves with the desired
 * range of random angle when obstacle is detected
 */
TEST(ExplorerTest, RandomAngleTest) {
    Explorer exp = Explorer();
    exp.setRotation();
    EXPECT_GE(101 * (3.14 / 180), exp.direction().angular.z);
}
