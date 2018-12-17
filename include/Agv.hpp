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
 * @file Agv.hpp
 * @brief agv_navigation package
 *
 * @section DESCRIPTION
 *
 *  This file is a header file to declare all the class variables and
 *  functions that will be used for frontier exploration on turtlebot.
 *
 * @dependencies: This file depends on Explorer.hpp
 * @author Bhargav Dandamudi and Mayank Pathak
 * @version 1
 * @date 2018-12-15
 */



#include "./Explorer.hpp"

#ifndef INCLUDE_AGV_HPP_
#define INCLUDE_AGV_HPP_
/**
 * @brief Agv class
 *
 * Initializes subscriber, publishers and timers
 * Has a method to publish twist messges.
 *
 */
class Agv {
  public:  // NOLINT
    /**
     * @brief Constructor for Agv Class
     */
    Agv();
    /**
     *
     * @brief Publishes twist messages
     *
     * @return true if successful
     *
     */
    bool explore();
    /**
     * @brief Destructor for Agv Class
     */
    ~Agv();

  private:  // NOLINT
    ros::NodeHandle n;
    /**
      * @brief Creates Explorer Object
      */
    Explorer explorer = Explorer();
    /**
     * @brief registers subscriber for image pointers
     */
    ros::Subscriber subLaserScan;
    /**
     * @brief registers timer for rotating the turtleBot
     */
    ros::Timer Rotatetimer;
    /**
     * @brief registers timer for camera service
     */
    ros::Timer camTimer;
    /**
     * @brief registers publisher for action(twist) messages
     */
    ros::Publisher actionPub;
};

#endif  // INCLUDE_AGV_HPP_
