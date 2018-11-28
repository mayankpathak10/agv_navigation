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
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

class explorer {
  public:
    // default constructor
    explorer();
    // default destructor
    ~explorer();

    void serviceFunction(void);

  private:
    ros::NodeHandle nh_;
    std::vector<std::vector<int4d>> grid_;
    LaserScan laser_scan_;
    ros::ServiceServer server_;
};




#endif  // INCLUDE_EXPLORER_HPP_
