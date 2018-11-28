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

#ifndef INCLUDE_ASTAR_NAVIGATOR_HPP_
#define INCLUDE_ASTAR_NAVIGATOR_HPP_

#include <geometry_msgs/Twist.h>
#include<ros/ros.h>
#include<vector>
#include<sensor_msgs/LaserScan.h>

class astar_navigator {
  public:
    // default constructor
    astar_navigator();
    // default destructor
    ~astar_navigator();

    vector astar(std::vector<std::vector<int4d>>);

  private:
    std::vector<double> open_list_;
    std::vector<double> closed_list_;
    double row;
    double column;
    std::vector<double> dest;
    std::vector<double> source;
};

#endif  // INCLUDE_ASTAR_NAVIGATOR_HPP_
