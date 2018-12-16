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
 * @file Explorer.hpp
 * @brief agv_navigation package
 *
 * @section DESCRIPTION
 *
 *  This file is a header file to declare all the class variables and
 *  functions used for implementing frontier exploration.
 *
 * @author Bhargav Dandamudi and Mayank Pathak
 * @version 1
 * @date 2018-12-15
 */


#ifndef INCLUDE_EXPLORER_HPP_
#define INCLUDE_EXPLORER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Explorer {
  public:
    /**
     * @brief  Default Constructor for Explorer Class
     */
    Explorer();

    /**
     * @brief  Get distance from obstacles
     *
     * @Returns   distObst
     */
    float obstDist();

    /**
     * @brief  Get Rota flag which sets the rotation
     *
     * @Returns   Rota Flag
     */
    bool rotateBot();

    /**
     * @brief Set RotaFlag to false
     */
    void setRotation();

    /**
     * @brief scan subscriber to find closest obstacle
     *
     * @Param input is te pointer to array with obstacle distances
     */
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& input);

    /**
     * @brief      sets the rota flag to true every 45 sec
     * @Param event is the ros::TimerEvent structure
     * @return None
     */
    void RotatetimerCallback(const ros::TimerEvent& event);

    /**
     * @brief  generate twist messages to move agv
     *
     * @Returns   Twist messages(action)
     */
    geometry_msgs::Twist direction();

    /**
     * @brief  Destructor for navigator class
     */
    ~Explorer();

  private:
    /**
     * @brief flag for rotation of agv
     */
    bool rotaFlag;

    /**
     * @brief  to contain distance from obstacles
     */
    float distObst;

    /**
     * @brief counts the number of rotations
     */
    int rotateCount;

    /**
     * @brief  twist messgage for agv to move
     */
    geometry_msgs::Twist action;
};
#endif   // INCLUDE_EXPLORER_HPP_
